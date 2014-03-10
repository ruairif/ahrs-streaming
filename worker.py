#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import signal
import threading
import zmq
import msgpack
import argparse
import yaml
import os
from sensor import SensorTypes as ST
from time import time, sleep
from madgwick import Madgwick


DEFAULTS = {'port': 5666,
            'bus':  1,
            'device': None,
            'baudrate': None,
            'sensors': ['ADXL345', 'ITG3200'],
            'sample_rate': 100,
            'gain': 0.5,
            'id': 'IMU'}

SCRIPT_PATH = os.path.dirname(os.path.realpath(__file__))
DEFAULT_CONFIG_FILE = SCRIPT_PATH + '/default.yaml'

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

SENSORS = {}
SENSORS_MAP = {}

MODULES = {}

class SensorWorker(threading.Thread):

    def __init__(self,
                 bus=1,
                 device=None,
                 sensors=None,
                 port=5666,
                 baudrate=9600,
                 gain=1.0,
                 sample_rate=100,
                 _id='sensors'):
        threading.Thread.__init__(self)
        self.context = zmq.Context()
        self.pub = self.context.socket(zmq.PUB)
        self.pub.bind("tcp://*:%s" % port)
        self.pub.setsockopt(zmq.LINGER, 40)
        self.pub.setsockopt(zmq.SNDHWM, 1)
        signal.signal(signal.SIGINT, self.clean_up)
        self.sample_rate = sample_rate
        self.gain = gain
        self._id = _id
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
                    key_name = SENSORS_MAP[key]
                    if key_name in ['ahrs', 'imu']:
                        sensors_data_renamed = sensors_data[key]
                        break
                    else:
                        sensors_data_renamed[key_name] = sensors_data[key]

                sensors_data = sensors_data_renamed
                sensors_data['quaternion'] = self.update_quaternion(
                    quaternion,
                    sensors_data)
                sensors_data['euler'] = quaternion.to_euler()
                sensors_data['read_time'] = time() - read_time
                sensors_data['time'] = time()
                sensors_data['id'] = self._id
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
        _ = ST.name
        accel = sensors.get(_(ST.ACCELEROMETER), None)
        gyro = sensors.get(_(ST.GYROSCOPE), None)
        mag = sensors.get(_(ST.MAGNETOMETER), None)
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
        if value is None or value == 'None':
            return
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
        if value is None or value == 'None':
            return

        if not self.valid_choices:
            return value
        try:
            value = self.choice_type(value)
        except ValueError:
            raise argparse.ArgumentTypeError(
                '"%s" is not a valid %s' % (value, self.choice_type))
        if self.choice_type == str:
            _value = value.upper()
            self.valid_choices = [s.upper() for s in self.valid_choices]
        if _value not in self.valid_choices:
            raise argparse.ArgumentTypeError(
                '%s is not a valid option.\n'
                'Valid options are: %s.' % (value,
                                            str(self.valid_choices)[1:-1]))
        return value


def create_options_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p',
                        '--port',
                        type=IntRange(1, 65535),
                        default=None,
                        const='port',
                        nargs='?',
                        help='tcp port to publish data on: {0, 1, ..., 65535}')

    parser.add_argument('-b',
                        '--bus',
                        type=IntRange(0, 32),
                        default=None,
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

    parser.add_argument('--baudrate',
                        type=ValidChoice(BAUDRATES, int),
                        default=None,
                        const='baudrate',
                        nargs='?',
                        help='Baudrate to use for serial sensors')

    parser.add_argument('-s',
                        '--sensors',
                        type=ValidChoice(SENSORS.keys()),
                        metavar='sensors',
                        nargs='+',
                        help='Names of sensors to read data from.')

    parser.add_argument('--sample-rate',
                        type=int,
                        metavar='sample_rate',
                        nargs='?',
                        help='Sample rate of sensors in Hertz')

    parser.add_argument('-g',
                        '--gain',
                        type=float,
                        metavar='gain',
                        nargs='?',
                        help='Gain value for gyroscope bias')

    parser.add_argument('--id',
                        type=str,
                        metavar='id',
                        nargs='?',
                        help='identification string for a particular sensor'
                             ' or group of sensors')


    parser.add_argument('-f',
                        '--config-file',
                        type=str,
                        default=None,
                        const='config_file',
                        nargs='?',
                        help='YAML formatted config file to read config'
                             ' options from')
    return parser


def main():

    configs_parser = create_options_parser()

    args = configs_parser.parse_args()

    with open(args.config_file or DEFAULT_CONFIG_FILE) as f:
        config_data = yaml.load(f)


    options = config_data.get('options', None)
    conf_args = []

    for key in DEFAULTS.keys():
        default_value = DEFAULTS[key]
        config_value = options.get(key)
        arg_key = '--' + key.replace('_', '-')

        if isinstance(config_value, list) or isinstance(default_value, list):
            conf_args += [arg_key] + list(config_value) or default_value

        elif config_value or default_value:
            config_value = str(config_value) if config_value else None
            conf_args += [arg_key, config_value or str(default_value)]

    configs = configs_parser.parse_args(conf_args)

    for module, classes in config_data.get('sensors', {}).items():
        try:
            MODULES[module] = __import__(module, fromlist=classes)
            _ = MODULES[module]
            for sensor in classes:
                try:
                    sensor_obj = vars(_)[sensor]
                    SENSORS[sensor.upper()] = sensor_obj
                    SENSORS_MAP[sensor] = ST.name(sensor_obj._type)
                except KeyError:
                    raise ImportError('Cannot import name "%s"' % sensor)
                    sys.exit(0)
        except ImportError as e:
            raise e
            sys.exit(0)

    options_parser = create_options_parser()
    args = options_parser.parse_args()

    sensors = list(set((args.sensors or []) + configs.sensors))

    worker = SensorWorker(bus=args.bus or configs.bus,
                          device=args.device or configs.device,
                          sensors=sensors,
                          port=args.port or configs.port,
                          baudrate=args.baudrate or configs.baudrate,
                          _id=args.id or configs.id)

    worker.start()


if __name__ == "__main__":
    main()
