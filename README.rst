AHRS Data Streaming Utilities
=============================

This program is designed to stream data from supported sensors over TCP.
Supported sensrs include ITG-3200, ADXL345, HMC5883L, Nav440.

Usage
=====

worker.py [-h] [-p [PORT]] [-b [BUS]] [-d [DEVICE]] [-r [RAW]]
          [--baudrate [BAUDRATE]] [-s sensors [sensors ...]]
          [--sample-rate [sample_rate]] [-g [gain]] [--id [id]]

optional arguments:
  -h, --help            show this help message and exit
  -p [PORT], --port [PORT]
                        tcp port to publish data on: {0, 1, ..., 65535}
  -b [BUS], --bus [BUS]
                        I2C bus to read data from
  -d [DEVICE], --device [DEVICE]
                        device bus to read data from
  -r [RAW], --raw [RAW]
                        Return raw or physical data
  --baudrate [BAUDRATE]
                        Baudrate to use for serial sensors
  -s sensors [sensors ...], --sensors sensors [sensors ...]
                        Names of sensors to read data from.Supported sensors
                        include: HMC5883L, NAV440, ADXL345, ITG-3200
  --sample-rate [sample_rate]
                        Sample rate of sensors in Hertz
  -g [gain]             Gain value for gyroscope bias
  --id [id]             identification string for a particular sensor or group
                        of sensors

