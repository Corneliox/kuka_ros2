import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / 'kuka_ros2_demo' / 'pick_place_constants.py'


spec = importlib.util.spec_from_file_location('pick_place_constants', MODULE_PATH)
pick_place_constants = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pick_place_constants)


class PickHeightConstantsTest(unittest.TestCase):
    def test_object_specific_pick_heights(self):
        self.assertAlmostEqual(pick_place_constants.get_pick_z_for_object('cube'), 0.060)
        self.assertAlmostEqual(pick_place_constants.get_pick_z_for_object('red_cube'), 0.060)
        self.assertAlmostEqual(pick_place_constants.get_pick_z_for_object('clamp'), 0.07915)
        self.assertAlmostEqual(pick_place_constants.get_pick_z_for_object('washer'), 0.05631)


if __name__ == '__main__':
    unittest.main()
