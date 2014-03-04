#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import signal
import threading
import zmq
import msgpack
import argparse
from ahrs_sensors import ADXL345, HMC5883L, ITG3200, Nav440
from time import time, sleep
from madgwick import Madgwick

SENSORS = {'ADXL345': ADXL345,
           'HMC5883L': HMC5883L,
           'ITG-3200': ITG3200,
           'NAV440': Nav440}

SENSOR_MAP = {'ADXL345': 'accel',
              'HMC5883L': 'mag',
              'ITG3200': 'gyro',
              'Nav440': None}

BAUDRATES = (110,
             300,
             600,
             1200,
             2400,
             4800,
             9600,
             14400,
             19200,
             28800,
             38400,
             56000,
             57600,
             115200,
             128000,
             153600,
             230400,
             256000,
             460800,
             921600)


class SensorWorker(threading.Thread):

    def __init__(self,
                 bus=1,
                 device=None,
                 sensors=None,
                 raw=False,
                 port=5666,
                 baudrate=9600,
                 gain=1.0,
                 sample_rate=100):
        threading.Thread.__init__(self)
        self.context = zmq.Context()
        self.pub = self.context.socket(zmq.PUB)
        self.pub.bind("tcp://*:%s" % port)
        self.pub.setsockopt(zmq.LINGER, 40)
        self.pub.setsockopt(zmq.SNDHWM, 1)
        signal.signal(signal.SIGINT, self.clean_up)
        self.raw = raw
        self.sample_rate = sample_rate
        self.gain = gain
        if device is not None:
            if device.startswith('/dev/i2c-'):
                bus = int(device[9:])
            else:
                bus = device
        self.sensors = self.Sensor_Info(bus, sensors, baudrate)

    class Sensor_Info(object):

        def __init__(self, bus=1, sensors=None, baudrate=9600):
            self.bus = bus
            self.sensors = [] if sensors is None else sensors
            for i, sensor in enumerate(self.sensors):
                self.sensors[i] = SENSORS[sensor](bus=bus, baudrate=baudrate)

    def run(self):
        read_time = time()
        quaternion = Madgwick(self.gain, self.sample_rate)
        try:
            while True:
                time_since_last_reading = time() - read_time
                if 0 < time_since_last_reading < 0.006:
                    sleep(0.006 - time_since_last_reading)
                read_time = time()
                sensors_data = dict([(sensor.__class__.__name__,
                                      sensor.poll())
                                     for sensor in self.sensors.sensors])
                sensors_data_renamed = {}
                for key in sensors_data.keys():
                    key_name = SENSOR_MAP[key]
                    if key is None:
                        sensors_data_renamed = sensors_data[key]
                        break
                    else:
                        sensors_data_renamed[key_name] = sensors_data[key]

                sensors_data = sensors_data_renamed
                sensors_data['quaternion'] = self.update_quaternion(
                    quaternion,
                    sensors_data)
                sensors_data['read_time'] = time() - read_time
                sensors_data['time'] = time()
                sensors_data['id'] = 0x01
                packed_data = msgpack.dumps(sensors_data)
                self.pub.send(packed_data)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)
        finally:
            print('Finishing')
            self.clean_up()

    def update_quaternion(self, quaternion, sensors):
        accel = sensors['accel']
        gyro = sensors['gyro']
        mag = sensors['mag']
        quaternion.update(accel, gyro, mag)
        return quaternion.as_dict()

    def clean_up(self):
        self.pub.close()
        self.context.term()
        sys.exit(0)


class IntRange(object):

    def __init__(self, start, stop=None):
        if stop is None:
            start, stop = 0, start
        self.start, self.stop = start, stop

    def __call__(self, value):
        try:
            value = int(value)
        except ValueError:
            raise argparse.ArgumentTypeError(
                '"%s" is not a valid integer' % value)
        if value < self.start or value >= self.stop:
            raise argparse.ArgumentTypeError(
                '"%s" is outside of range' % value)
        return value


class ValidChoice(object):
    def __init__(self, choices, choice_type=str):
        self.valid_choices = tuple(choices)
        self.choice_type = choice_type

    def __call__(self, value):
        try:
            value = self.choice_type(value)
        except ValueError:
            raise argparse.ArgumentTypeError(
                '"%s" is not a valid %s' % (value, self.choice_type))
        if self.choice_type == str:
            value = value.upper()
            self.valid_choices = [s.upper() for s in self.valid_choices]
        if value not in self.valid_choices:
            raise argparse.ArgumentTypeError(
                '%s is not a valid option' % value)
        return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p',
                        '--port',
                        type=IntRange(1, 65535),
                        default=5666,
                        const='port',
                        nargs='?',
                        help='tcp port to publish data on: {0, 1, ..., 65535}')

    parser.add_argument('-b',
                        '--bus',
                        type=IntRange(0, 32),
                        default=1,
                        const='bus',
                        nargs='?',
                        help='I2C bus to read data from')

    parser.add_argument('-d',
                        '--device',
                        type=str,
                        default=None,
                        const='device',
                        nargs='?',
                        help='device bus to read data from')

    parser.add_argument('-r',
                        '--raw',
                        type=bool,
                        default=False,
                        const='raw',
                        nargs='?',
                        help='Return raw or physical data')

    parser.add_argument('--baudrate',
                        type=ValidChoice(BAUDRATES, int),
                        default=9600,
                        const='baudrate',
                        nargs='?',
                        help='Baudrate to use for serial sensors')

    parser.add_argument('-s',
                        '--sensors',
                        type=ValidChoice(SENSORS.keys()),
                        metavar='sensors',
                        nargs='+',
                        help='Names of sensors to read data from.'
                             'Supported sensors include:\n %s'
                             % ', '.join(SENSORS.keys()))

    parser.add_argument('--sample-rate',
                        type=int,
                        metavar='sample_rate',
                        nargs='?',
                        help='Sample rate of sensors in Hertz')

    parser.add_argument('-g',
                        type=float,
                        metavar='gain',
                        nargs='?',
                        help='Gain value for gyroscope bias')

    args = parser.parse_args()
    worker = SensorWorker(bus=args.bus,
                          device=args.device,
                          sensors=args.sensors,
                          raw=args.raw,
                          port=args.port,
                          baudrate=args.baudrate)

    worker.start()


if __name__ == "__main__":
    main()
