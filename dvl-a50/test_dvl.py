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

    def test_lower_median_uses_two_or_more_valid_beams_and_projects_to_dvl_z(self):
        cosine = math.cos(math.radians(22.5))
        data = {
            "altitude": 99.0,
            "transducers": [
                {"distance": 1.0 / cosine, "beam_valid": True},
                {"distance": 3.0 / cosine, "beam_valid": True},
                {"distance": 2.0 / cosine, "beam_valid": True},
                {"distance": 2.0 / cosine, "beam_valid": True},
            ],
        }

        self.assertAlmostEqual(dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN), 2.0)

    def test_lower_median_requires_two_valid_positive_beams(self):
        data = {
            "altitude": 1.23,
            "transducers": [
                {"distance": -1.0, "beam_valid": True},
                {"distance": 1.0, "beam_valid": False},
            ],
        }

        self.assertIsNone(dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))

    def test_lower_median_tolerates_a_deep_beam(self):
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

        self.assertAlmostEqual(
            dvl.DvlDriver.rangefinder_distance(data, dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN),
            1.0,
        )

    def test_per_beam_filter_requires_three_samples_and_resets_after_gap(self):
        driver = dvl.DvlDriver()
        data = {
            "transducers": [
                {"id": 0, "distance": 1.0, "beam_valid": True},
                {"id": 1, "distance": 1.0, "beam_valid": True},
                {"id": 2, "distance": 1.0, "beam_valid": True},
                {"id": 3, "distance": 10.0, "beam_valid": True},
            ]
        }
        cosine = math.cos(math.radians(22.5))

        self.assertIsNone(driver.filtered_rangefinder_distance(data, timestamp_s=0.0))
        self.assertIsNone(driver.filtered_rangefinder_distance(data, timestamp_s=0.2))
        self.assertAlmostEqual(driver.filtered_rangefinder_distance(data, timestamp_s=0.4), cosine)
        self.assertIsNone(driver.filtered_rangefinder_distance(data, timestamp_s=1.5))

    def test_per_beam_filter_prevents_one_frame_beam_jump_from_changing_output(self):
        driver = dvl.DvlDriver()

        def frame(beam_one_distance):
            return {
                "transducers": [
                    {"id": 0, "distance": 1.0, "beam_valid": True},
                    {"id": 1, "distance": beam_one_distance, "beam_valid": True},
                    {"id": 2, "distance": 3.0, "beam_valid": True},
                    {"id": 3, "distance": 4.0, "beam_valid": True},
                ]
            }

        cosine = math.cos(math.radians(22.5))
        self.assertIsNone(driver.filtered_rangefinder_distance(frame(2.0), timestamp_s=0.0))
        self.assertIsNone(driver.filtered_rangefinder_distance(frame(2.0), timestamp_s=0.2))
        self.assertAlmostEqual(driver.filtered_rangefinder_distance(frame(2.0), timestamp_s=0.4), 2.0 * cosine)
        self.assertAlmostEqual(driver.filtered_rangefinder_distance(frame(20.0), timestamp_s=0.6), 2.0 * cosine)
        self.assertAlmostEqual(driver.filtered_rangefinder_distance(frame(20.0), timestamp_s=0.8), 3.0 * cosine)

    def test_rangefinder_rejects_stale_or_out_of_order_frames(self):
        driver = dvl.DvlDriver()

        self.assertTrue(driver.rangefinder_frame_is_fresh({"time_of_transmission": 1_000_000}, wall_time_s=1.1))
        self.assertFalse(driver.rangefinder_frame_is_fresh({"time_of_transmission": 1_000_000}, wall_time_s=1.1))
        self.assertFalse(driver.rangefinder_frame_is_fresh({"time_of_transmission": 2_000_000}, wall_time_s=4.0))

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

            driver.rangefinder_beam_histories[0] = dvl.deque([1.0, 2.0, 3.0])
            self.assertTrue(driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_REPORTED))
            self.assertTrue(driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN))
            self.assertFalse(driver.rangefinder_beam_histories)
            self.assertFalse(driver.set_rangefinder_distance_source("not-a-source"))
