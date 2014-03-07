'''\
Implementation of the Madgwick algorithm for orientation implementation using
IMU (accelerometer and gyroscope) or AHRS (accelerometer, gyroscope and
magnetometer) data.\
'''

from math import sqrt, atan2, asin, pi


class Madgwick(object):

    '''\
    Madgwick algorithm implementation to correct rotation data read from
    sensors\
    '''

    class Quaternion(object):

        '''\
        4 dimensional vector describing rotation\
        '''
        offset = 0
        positions = 'wxyz'

        def __init__(self, w=1, x=0, y=0, z=0):
            self.data = [w, x, y, z]
            self.w = w
            self.x = x
            self.y = y
            self.z = z

        def __getitem__(self, key):
            if not key in list(range(0, 5)) + ['w', 'x', 'y', 'z']:
                return super(Madgwick, self).__getitem__(key)
            try:
                return self.data[key + self.offset]
            except TypeError:
                return vars(self)[key]

        def __setitem__(self, key, value):
            if not key in list(range(0, 5)) + ['w', 'x', 'y', 'z']:
                return super(Madgwick, self).__setitem__(key, value)
            try:
                self.data[key + self.offset] = value
                vars(self)[self.positions[key]] = value
            except TypeError:
                self.data[self.positions.index(key) + self.offset] = value
                vars(self)[key] = value
            return value

        def __delitem__(self, key):
            if not key in list(range(0, 5)) + ['w', 'x', 'y', 'z']:
                return super(Madgwick.Quaternion, self).__delitem__(key)

        def __iter__(self):
            for x in self.data[self.offset:]:
                yield x

        def __len__(self):
            return len(self.data)

        def __repr__(self):
            return str({'w': self.data[0],
                        'x': self.data[1],
                        'y': self.data[2],
                        'z': self.data[3]})

    class Sensor(Quaternion):

        '''\
        Class for manipulating x, y, z sensor values\
        '''

        offset = 1

        def __init__(self, x=0, y=0, z=0):
            super(Madgwick.Sensor, self).__init__(0, x, y, z)

        def __repr__(self):
            return str({'x': self.data[1],
                        'y': self.data[2],
                        'z': self.data[3]})

    def __init__(self, gain=1.0, sample_rate=100):
        self.quaternion = self.Quaternion(1.0, 0.0, 0.0, 0.0)
        self.gain = gain
        self.sample_period = 1.0 / sample_rate

    def update(self, accelerometer, gyroscope, magnetometer=None):
        '''\
        Update the rotation of the device using data from an accelerometer,
        gyroscope and optionally a magnetometer\
        '''
        if magnetometer is None:
            return self._madgwick_imu(accelerometer, gyroscope)
        else:
            return self._madgwick_ahrs(accelerometer, gyroscope, magnetometer)

    def _madgwick_ahrs(self, accelerometer, gyroscope, magnetometer):
        '''\
        Implementation of using accelerometer, gyroscope and magnetometer data\
        '''
        q = self.quaternion

        # Set up and normalise sensor data
        _ = accelerometer
        a = self.normalise(self.Sensor(_['x'], _['y'], _['z']))
        _ = magnetometer
        m = self.normalise(self.Sensor(_['x'], _['y'], _['z']))

        # Pre calculate repeated mathmathical operations

        _2q = self.scalar_multiply(q, 2)
        _qq = self.quaternion_multiply(q, q)

        _2qwm = self.scalar_multiply(m, _2q.w)
        _2qxm = self.Sensor(_2q.x * m.x, 0, 0)

        qwqx = q.w * q.x
        qwqy = q.w * q.y
        qwqz = q.w * q.z
        qxqy = q.x * q.y
        qxqz = q.x * q.z
        qyqz = q.y * q.z
        _2qyqz = 2.0 * qyqz
        _2qwqy = 2.0 * qwqy

        # Reference direction of Earth's magnetic field
        h = self.Sensor()
        h.x = (m.x * _qq.w - _2qwm.y * q.z + _2qwm.z * q.y +
               m.x * _qq.x + _2q.x * m.y * q.y + _2q.x * m.z * q.z -
               m.x * _qq.y - m.x * _qq.z)
        h.y = (_2qwm.x * q.z + m.y * _qq.w - _2qwm.z * q.x +
               _2qxm.x * q.y - m.y * _qq.x + m.y * _qq.y +
               _2q.y * m.z * q.z - m.y * _qq.z)

        _2b = self.Sensor()
        _2b.x = sqrt(h.x ** 2 + h.y ** 2) * 1.0
        _2b.z = (-_2qwm.x * q.y +
                 _2qwm.y * q.x +
                 m.z * _qq.w +
                 _2qxm.x * q.z -
                 m.z * _qq.x +
                 _2q.y * m.y * q.z -
                 m.z * _qq.y +
                 m.z * _qq.z)
        _4b = self.Sensor()
        _4b.x = 2.0 * _2b.x
        _4b.z = 2.0 * _2b.z

        # Gradient decent algorithm corrective step
        s = self.Quaternion()
        s.w = (-_2q.y * (2.0 * qxqz - _2qwqy - a.x) +
               _2q.x * (2.0 * qwqx + _2qyqz - a.y) -
               _2b.z * q.y * (_2b.x * (0.5 - _qq.y - _qq.z) +
                              _2b.z * (qxqz - qwqy) - m.x) +
               (-_2b.x * q.z + _2b.z * q.x) *
               (_2b.x * (qxqy - qwqz) + _2b.z * (qwqx + qyqz) - m.y) +
               _2b.x * q.y *
               (_2b.x * (qwqy + qxqz) + _2b.z * (0.5 - _qq.x - _qq.y) - m.z))

        s.x = (_2q.z * (2.0 * qxqz - _2qwqy - a.x) +
               _2q.w * (2.0 * qwqx + _2qyqz - a.y) -
               4.0 * q.x * (1 - 2.0 * _qq.x - 2.0 * _qq.y - a.z) +
               _2b.z * q.z *
               (_2b.x * (0.5 - _qq.y - _qq.z) + _2b.z * (qxqz - qwqy) - m.x) +
               (_2b.x * q.y + _2b.z * q.w) * (_2b.x * (qxqy - qwqz) +
                                              _2b.z * (qwqx + qyqz) - m.y) +
               (_2b.x * q.z - _4b.z * q.x) *
               (_2b.x * (qwqy + qxqz) + _2b.z * (0.5 - _qq.x - _qq.y) - m.z))

        s.y = (-_2q.w * (2.0 * qxqz - _2qwqy - a.x) +
               _2q.z * (2.0 * qwqx + _2qyqz - a.y) -
               4.0 * q.y * (1 - 2.0 * _qq.x - 2.0 * _qq.y - a.z) +
               (-_4b.x * q.y - _2b.z * q.w) *
               _2b.x * (0.5 - _qq.y - _qq.z) + _2b.z *
               ((qxqz - qwqy) - m.x) +
               (_2b.x * q.y + _2b.z * q.z) * (_2b.x * (qxqy - qwqz) +
                                              _2b.z * (qwqx + qyqz) - m.y) +
               (_2b.x * q.w - _4b.z * q.y) *
               (_2b.x * (qwqy + qxqz) + _2b.z * (0.5 - _qq.x - _qq.y) - m.z))

        s.z = (_2q.x * (2.0 * qwqz - _2qwqy - a.x) +
               _2q.y * (2.0 * qwqx + _2qyqz - a.y) +
               (-_4b.x * q.z + _2b.z * q.x) *
               (_2b.x * (0.5 - _qq.y - _qq.z) + _2b.z * (qxqz - qwqy) - m.x) +
               (-_2b.x * q.w + _2b.z * q.y) *
               (_2b.x * (qxqy - qwqz) + _2b.z * (qwqx + qyqz) - m.y) +
               _2b.x * q.x *
               (_2b.x * (qwqy + qxqz) + _2b.z * (0.5 - _qq.x - _qq.y) - m.z))

        s = Madgwick.normalise(s)

        return self._update_rotation(s, gyroscope)

    def _madgwick_imu(self, accelerometer, gyroscope):
        '''\
        Implementation of  using accelerometer and gyroscope data\
        '''
        q = self.quaternion

        # Set up and normalise sensor data
        _ = accelerometer
        a = Madgwick.normalise(self.Sensor(_['x'], _['y'], _['z']))

        _2q = self.scalar_multiply(q, 2)
        _4q = self.scalar_multiply(q, 4)
        _8q = self.scalar_multiply(q, 8)
        _qq = self.quaternion_multiply(q, q)

        s = self.Quaternion()
        s.w = _4q.w * _qq.y + _2q.y * a.x + _4q.w * _qq.x - _2q.x * a.y
        s.x = (_4q.x * _qq.z - _2q.z * a.x + 4.0 * _qq.w * q.x - _2q.w * a.y -
               _4q.x + _8q.x * _qq.x + _8q.x * _qq.y + _4q.x * a.z)
        s.y = (4.0 * _qq.w * q.y + _2q.w * a.x + _4q.y * _qq.z - _2q.z * a.y -
               _4q.y + _8q.y * _qq.x + _8q.y * _qq.y + _4q.y * a.z)
        s.z = 4.0 * _qq.x * q.z - _2q.x * a.x + 4.0 * _qq.y * q.z - _2q.y * a.y

        s = Madgwick.normalise(s)

        return self._update_rotation(s, gyroscope)

    def _update_rotation(self, s, gyroscope):
        '''\
        Update the rotation from the gyroscope and other sensor data\
        '''
        # Compute rate of change of quaternion
        _ = gyroscope
        g = self.Sensor(_['x'], _['y'], _['z'])
        q = self.quaternion
        gain = self.gain
        rate = self.Quaternion(
            0.5 * (-q.x ** 2 - q.y ** 2 - q.z ** 2) - gain * s.w,
            0.5 * (q.w * g.x + q.y * g.z - q.z * g.y) - gain * s.x,
            0.5 * (q.w * g.y - q.x * g.z + q.z * g.x) - gain * s.y,
            0.5 * (q.w * g.z + q.x * g.y - q.y * g.x) - gain * s.z)

        # Integrate to yield quaternion
        for i in range(len(rate)):
            q[i] += rate[i] * self.sample_period

        q = Madgwick.normalise(q)
        self.quaternion = q
        return q

    def as_dict(self):
        '''\
        Return direction quaternion as dict\
        '''
        return {'w': self.quaternion.w,
                'x': self.quaternion.x,
                'y': self.quaternion.y,
                'z': self.quaternion.z}

    def to_euler(self):
        '''\
        Convert a quaternion q to yaw pitch and roll angles\
        '''
        q = self.quaternion
        rotate_x0 = 2.0*(q.y*q.z + q.w*q.x)
        rotate_x1 = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z
        rotate_x = 0.0
        if rotate_x0 and rotate_x1:
            rotate_x = atan2(rotate_x0, rotate_x1)

        rotate_y0 = -2.0*(q.x*q.z - q.w*q.y)
        rotate_y = 0.0
        if rotate_y0 >= 1.0:
            rotate_0 = pi/2.0
        elif rotate_y0 <= -1.0:
            rotate_y = -pi/2.0
        else:
            rotate_y = asin(rotate_y0)

        rotate_z0 = 2.0*(q.x*q.y + q.w*q.z)
        rotate_z1 = q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z
        rotate_z = 0.0
        if rotate_z0 and rotate_z1:
            rotate_z = atan2(rotate_z0, rotate_z1)

        return {'x': rotate_x,
                'y': rotate_y,
                'z': rotate_x}

    @classmethod
    def normalise(cls, vector):
        '''\
        Normalise a vector\
        '''
        mod = sqrt(sum(x ** 2 for x in vector))
        for i, v in enumerate(vector):
            vector[i] = v / mod

        return vector

    @classmethod
    def scalar_multiply(cls, vector, scaler):
        '''\
        Multiply vector by a scalar\
        '''
        _ = [scaler * x for x in vector]
        if len(_) < 4:
            _ = [0] + _
        return cls.Quaternion(_[0], _[1], _[2], _[3])

    @classmethod
    def quaternion_multiply(cls, quat_1, quat_2):
        '''\
        Multiply 2 quaternions together element by element\
        '''
        _ = tuple(x * y for x, y in zip(quat_1, quat_2))
        if len(_) < 4:
            _ = [0] + _
        return cls.Quaternion(_[0], _[1], _[2], _[3])
