"""Focused tests for DVL rangefinder distance selection."""

import importlib
import json
import math
import sys
import tempfile
import types
import unittest

# Keep this focused unit test runnable outside the extension container.
logger = types.SimpleNamespace(
    debug=lambda *_args, **_kwargs: None,
    info=lambda *_args, **_kwargs: None,
    warning=lambda *_args, **_kwargs: None,
)
sys.modules.setdefault("loguru", types.SimpleNamespace(logger=logger))
sys.modules.setdefault("nmap3", types.SimpleNamespace(Nmap=object))
sys.modules.setdefault("requests", types.SimpleNamespace())

dvl = importlib.import_module("dvl")


class RangefinderDistanceTest(unittest.TestCase):
    """Verify the source selection used before sending DISTANCE_SENSOR."""

    def test_median_beams_is_the_default_source(self):
        self.assertEqual(
            dvl.DvlDriver().rangefinder_distance_source,
            dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN,
        )

    def test_reported_distance_is_returned_unchanged(self):
        data = {"altitude": 1.23, "transducers": []}

        self.assertEqual(
            dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_REPORTED),
            1.23,
        )

    def test_median_uses_four_valid_beams_and_projects_to_dvl_z(self):
        cosine = math.cos(math.radians(22.5))
        data = {
            "altitude": 99.0,
            "transducers": [
                {"distance": 1.0 / cosine, "beam_valid": True},
                {"distance": 2.0 / cosine, "beam_valid": True},
                {"distance": 2.0 / cosine, "beam_valid": True},
                {"distance": 2.0 / cosine, "beam_valid": True},
            ],
        }

        self.assertAlmostEqual(dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN), 2.0)

    def test_median_requires_all_four_valid_positive_beams(self):
        data = {
            "altitude": 1.23,
            "transducers": [
                {"distance": 1.0, "beam_valid": True},
                {"distance": 1.0, "beam_valid": True},
                {"distance": 1.0, "beam_valid": True},
                {"distance": -1.0, "beam_valid": True},
            ],
        }

        self.assertIsNone(dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))

    def test_median_rejects_excessive_projected_beam_spread(self):
        cosine = math.cos(math.radians(22.5))
        data = {
            "altitude": 1.23,
            "transducers": [
                {"distance": 1.0 / cosine, "beam_valid": True},
                {"distance": 1.0 / cosine, "beam_valid": True},
                {"distance": 1.0 / cosine, "beam_valid": True},
                {"distance": 3.0 / cosine, "beam_valid": True},
            ],
        }

        self.assertIsNone(dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))

    def test_beam_median_filter_requires_three_samples_and_resets_after_gap(self):
        driver = dvl.DvlDriver()

        self.assertIsNone(driver.filtered_rangefinder_distance(1.0, now=0.0))
        self.assertIsNone(driver.filtered_rangefinder_distance(3.0, now=0.2))
        self.assertEqual(driver.filtered_rangefinder_distance(2.0, now=0.4), 2.0)
        self.assertIsNone(driver.filtered_rangefinder_distance(2.0, now=1.5))

    def test_source_selection_is_validated_and_persisted(self):
        driver = dvl.DvlDriver()
        with tempfile.TemporaryDirectory() as directory:
            driver.settings_path = directory + "/settings.json"

            self.assertTrue(driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))
            self.assertEqual(driver.rangefinder_distance_source, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN)
            with open(driver.settings_path, encoding="utf-8") as settings:
                self.assertEqual(
                    json.load(settings)["rangefinder_distance_source"],
                    dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN,
                )

            driver.rangefinder_distance_history.extend([1.0, 2.0, 3.0])
            self.assertTrue(driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_REPORTED))
            self.assertTrue(driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))
            self.assertFalse(driver.rangefinder_distance_history)
            self.assertFalse(driver.set_rangefinder_distance_source("not-a-source"))
