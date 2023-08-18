import unittest

import numpy as np
from pytransform3d import rotations as pr


class TestSum(unittest.TestCase):

    def test_euler_to_quaternion(self):
        euler_rpy = np.array([0.342564, 0.166510, 0.336120])
        quat = pr.quaternion_from_euler(euler_rpy, 0, 1, 2, True)
        quat_test = np.array([0.97049, 0.153756, 0.109199, 0.150277])
        self.assertTrue(np.allclose(quat, quat_test))


if __name__ == '__main__':
    unittest.main()
