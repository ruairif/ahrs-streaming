AHRS Data Streaming Utilities
=============================

This program is designed to stream data from supported sensors over TCP.
Any sensor that implements the sensor protocol in the required ahrs_ library
can be used. To add a new sensor include it in the sensors section of the
config file that is used when the program is run.


Usage
-----
To use the worker daemon from the commanline::

    usage: worker.py [-h] [-p [PORT]] [-b [BUS]] [-d [DEVICE]]
                     [--baudrate [BAUDRATE]] [-s sensors [sensors ...]]
                     [--sample-rate [sample_rate]] [-g [gain]] [--id [id]]
                     [-f [CONFIG_FILE]]

    optional arguments:
        -h, --help            show this help message and exit
        -p [PORT], --port [PORT]
                            tcp port to publish data on: {0, 1, ..., 65535}
        -b [BUS], --bus [BUS]
                            I2C bus to read data from
        -d [DEVICE], --device [DEVICE]
                            serial device bus to read data from
        --baudrate [BAUDRATE]
                            Baudrate to use for serial sensors
        -s sensors [sensors ...], --sensors sensors [sensors ...]
                            Names of sensors to read data from.
        --sample-rate [sample_rate]
                            Sample rate of sensors in Hertz
        -g [gain], --gain [gain]
                            Gain value for gyroscope bias
        --id [id]             identification string for a particular sensor or
                            group of sensors
        -f [CONFIG_FILE], --config-file [CONFIG_FILE]
                            YAML formatted config file to read config options
                            from

Options can be added from the command line or through an optional config file.
The structure of a config file shown below::

    options:
        port: 5666
        bus: 1
        device: null
        baudrate: null
        sensors:
          - ADXL345
          - ITG3200

        sample_rate: 100
        gain: 0.5
        id: IMU

    sensors:
        ahrs_sensors:
           - ADXL345
           - HMC5883L
           - ITG3200
           - L3G4200D
           - Nav440

The options section mirrors the command line options and can allow settings
to be easily reused between runs.
The sensors section is used to dynamically load any required sensor classes.

If the sensor is contained within a submodule it can be imported using the
structure below::

    sensors:
        module.submodule:
          - SENSOR


.. _ahrs: https://github.com/ruairif/ahrs
