#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import signal
import threading
import zmq
import msgpack
import argparse
from ahrs_sensors import ADXL345, HMC5883L, ITG3200
from time import time, sleep

SENSORS = {'ADXL345': ADXL345,
           'HMC5883L': HMC5883L,
           'ITG-3200': ITG3200}


class SensorWorker(threading.Thread):

    def __init__(self, bus=1, sensors=None, raw=True, port=5666):
        threading.Thread.__init__(self)
        self.context = zmq.Context()
        self.pub = self.context.socket(zmq.PUB)
        self.pub.bind("tcp://*:%s" % port)
        self.pub.setsockopt(zmq.LINGER, 40)
        self.pub.setsockopt(zmq.SNDHWM, 1)
        signal.signal(signal.SIGINT, self.clean_up)
        self.raw = raw
        self.sensors = self.Sensor_Info(bus, sensors)

    class Sensor_Info(object):

        def __init__(self, bus=1, sensors=None):
            self.bus = bus
            self.sensors = [] if sensors is None else sensors
            for i, sensor in enumerate(self.sensors):
                self.sensors[i] = SENSORS[sensor](bus=bus)

    def run(self):
        read_time = time()
        try:
            while True:
                time_since_last_reading = time() - read_time
                if 0 < time_since_last_reading < 0.006:
                    sleep(0.006 - time_since_last_reading)
                read_time = time()
                sensors_data = dict([(sensor.__class__.__name__,
                                      sensor.poll())
                                     for sensor in self.sensors.sensors])
                sensors_data['read_time'] = time() - read_time
                sensors_data['time'] = time()
                sensors_data['id'] = 0x01
                packed_data = msgpack.dumps(sensors_data)
                self.pub.send(packed_data)
        except KeyboardInterrupt:
            pass
        finally:
            print('Finishing')
            self.clean_up()

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


class ValidSensor(object):

    def __init__(self):
        self.valid_sensors = list(SENSORS.keys())

    def __call__(self, value):
        value = value.upper()
        if value not in self.valid_sensors:
            raise argparse.ArgumentTypeError(
                '%s is not a valid sensor' % value)
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

    parser.add_argument('-r',
                        '--raw',
                        type=bool,
                        default=True,
                        const='raw',
                        nargs='?',
                        help='Return raw or physical data')

    parser.add_argument('-s',
                        '--sensors',
                        type=ValidSensor(),
                        metavar='sensors',
                        nargs='+',
                        help='Names of sensors to read data from.'
                             'Supported sensors include:\n %s'
                             % ', '.join(SENSORS.keys()))

    args = parser.parse_args()
    worker = SensorWorker(bus=args.bus,
                          sensors=args.sensors,
                          raw=args.raw,
                          port=args.port)

    worker.start()

if __name__ == "__main__":
    main()
