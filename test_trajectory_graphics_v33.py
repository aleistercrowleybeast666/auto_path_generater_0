# -*- coding: utf-8 -*-
import unittest

from trajectory_graphics import classify_value, equal_bands


class TrajectoryGraphicsV33Test(unittest.TestCase):
    def test_speed_four_bands_and_over_limit(self):
        self.assertEqual(classify_value(0.0, 2.0, 4), 0)
        self.assertEqual(classify_value(0.5, 2.0, 4), 1)
        self.assertEqual(classify_value(1.0, 2.0, 4), 2)
        self.assertEqual(classify_value(1.5, 2.0, 4), 3)
        self.assertEqual(classify_value(2.0, 2.0, 4), 3)
        self.assertEqual(classify_value(2.02, 2.0, 4), 4)

    def test_accel_four_bands_and_over_limit(self):
        self.assertEqual(classify_value(0.0, 1.6, 4), 0)
        self.assertEqual(classify_value(0.4, 1.6, 4), 1)
        self.assertEqual(classify_value(0.8, 1.6, 4), 2)
        self.assertEqual(classify_value(1.2, 1.6, 4), 3)
        self.assertEqual(classify_value(1.6, 1.6, 4), 3)
        self.assertEqual(classify_value(1.62, 1.6, 4), 4)

    def test_angular_speed_four_bands_and_over_limit(self):
        self.assertEqual(classify_value(0.0, 6.0, 4), 0)
        self.assertEqual(classify_value(1.5, 6.0, 4), 1)
        self.assertEqual(classify_value(3.0, 6.0, 4), 2)
        self.assertEqual(classify_value(4.5, 6.0, 4), 3)
        self.assertEqual(classify_value(6.0, 6.0, 4), 3)
        self.assertEqual(classify_value(6.1, 6.0, 4), 4)

    def test_beta_four_bands_and_custom_maximum(self):
        self.assertEqual(equal_bands(3.0, 3), [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)])
        self.assertEqual(classify_value(-1.0, 4.0, 4), 1)
        self.assertEqual(classify_value(4.1, 4.0, 4), 4)


if __name__ == "__main__":
    unittest.main()
