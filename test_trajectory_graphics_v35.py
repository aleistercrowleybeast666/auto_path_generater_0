# -*- coding: utf-8 -*-
import unittest

from trajectory_graphics import classify_value, equal_bands, legend_entries


class TrajectoryGraphicsV35Test(unittest.TestCase):
    def test_equal_bands_and_classification(self):
        self.assertEqual(equal_bands(3.0, 3), [(0.0, 1.0), (1.0, 2.0), (2.0, 3.0)])
        self.assertEqual(classify_value(1.5, 3.0, 3), 1)
        self.assertEqual(classify_value(3.1, 3.0, 3), 3)

    def test_legend_entries(self):
        self.assertTrue(legend_entries("normal", 1, 1, 1, 1))


if __name__ == "__main__":
    unittest.main()
