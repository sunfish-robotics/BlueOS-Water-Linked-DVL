"""Deterministic persistence and driver lifecycle regression tests."""

import math
import unittest
from unittest.mock import patch

from range_filter import RangePersistenceFilter
from test_dvl import dvl


def frame(values):
    return {
        "transducers": [
            {"id": i, "distance": value / math.cos(math.radians(22.5)), "beam_valid": True}
            for i, value in enumerate(values)
        ]
    }


class PersistenceTest(unittest.TestCase):
    def test_noise_is_symmetric_without_downward_drift(self):
        filt = RangePersistenceFilter()
        filt.update(2, 0)
        outputs = [filt.update(2 + (0.02 if i % 2 else -0.02), i / 10) for i in range(1, 1001)]
        self.assertAlmostEqual(sum(outputs[-200:]) / 200, 2, places=8)
        self.assertIsNone(filt.candidate)
        self.assertLess(max(outputs) - min(outputs), 0.02)

    def test_brief_and_alternating_steps(self):
        for values in ([2.3] * 4 + [2], [2.3, 2] * 100):
            filt = RangePersistenceFilter()
            filt.update(2, 0)
            for i, value in enumerate(values, 1):
                self.assertEqual(filt.update(value, i / 10), 2)
            self.assertIsNone(filt.candidate)

    def test_sustained_step_rates_and_no_repeated_delay(self):
        for rate in (5, 10, 20, 50):
            filt = RangePersistenceFilter()
            filt.update(2, 0)
            first = None
            previous = 2
            for i in range(1, rate * 5):
                output = filt.update(2.3, i / rate)
                if output > 2 and first is None:
                    first = i / rate
                    self.assertGreaterEqual(first - 1 / rate, 0.5 - 1e-9)
                    self.assertLess(first - 1 / rate, 0.5 + 1 / rate + 1e-9)
                if first is not None:
                    self.assertAlmostEqual(2.3 - output, (2.3 - previous) * math.exp(-1 / rate))
                previous = output
            self.assertGreater(output, 2.29)

    def test_wandering_fixed_anchor_and_new_confirmed_candidate(self):
        filt = RangePersistenceFilter()
        filt.update(2, 0)
        for i in range(1, 21):
            self.assertEqual(filt.update(2.3 + i * 0.02, i / 10), 2)
        for i in range(21, 28):
            filt.update(2.8, i / 10)
        self.assertTrue(filt.confirmed)
        held = filt.accepted
        self.assertEqual(filt.update(3.1, 2.8), held)
        self.assertFalse(filt.confirmed)
        self.assertEqual(filt.update(1.7, 2.9), 1.7)

    def test_invalid_gaps_rewind_and_duplicate(self):
        for bad in (None, float("nan"), float("inf"), -1, 0):
            filt = RangePersistenceFilter()
            filt.update(2, 0)
            filt.update(2.3, 0.1)
            self.assertIsNone(filt.update(bad, 0.2))
            self.assertIsNone(filt.candidate)
            self.assertEqual(filt.update(2.4, 0.3), 2.4)
        for timestamp in (0.1, 0.0, float("nan")):
            filt = RangePersistenceFilter()
            filt.update(2, 0.1)
            self.assertIsNone(filt.update(2.3, timestamp))
        filt = RangePersistenceFilter()
        filt.update(2, 0)
        filt.update(2.3, 0.1)
        self.assertEqual(filt.update(2.4, 1), 2.4)
        self.assertFalse(filt.confirmed)

    def test_irregular_sampling_uses_elapsed_time(self):
        filt = RangePersistenceFilter()
        filt.update(2, 0)
        for timestamp in (0.05, 0.12, 0.31, 0.49):
            self.assertEqual(filt.update(2.3, timestamp), 2)
        self.assertAlmostEqual(filt.update(2.3, 0.56), 2 + 0.3 * (1 - math.exp(-0.07)))


class DriverPersistenceTest(unittest.TestCase):
    def test_selection_deep_excursion_and_shallower_median_latency(self):
        driver = dvl.DvlDriver()
        for i in range(30):
            result = driver.filtered_rangefinder_distance(frame([2, 2, 2, 8.7]), i / 10)
        self.assertEqual(result, 2)
        self.assertEqual(driver.filtered_rangefinder_distance(frame([1.7] * 4), 3), 2)
        self.assertEqual(driver.filtered_rangefinder_distance(frame([1.7] * 4), 3.1), 1.7)

    def test_invalid_beams_recovery_and_duplicates(self):
        driver = dvl.DvlDriver()
        for i in range(3):
            driver.filtered_rangefinder_distance(frame([2] * 4), i / 10)
        self.assertEqual(driver.filtered_rangefinder_distance(frame([2, 2]), 0.3), 2)
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([2, float("inf"), float("nan")]), 0.4))
        for timestamp in (0.5, 0.6):
            self.assertIsNone(driver.filtered_rangefinder_distance(frame([2, 2]), timestamp))
        self.assertEqual(driver.filtered_rangefinder_distance(frame([2, 2]), 0.7), 2)
        duplicate = frame([2])
        duplicate["transducers"] *= 4
        self.assertIsNone(driver.filtered_rangefinder_distance(duplicate, 0.8))

    def test_gap_and_clock_rewind_require_warmup(self):
        driver = dvl.DvlDriver()
        for i in range(3):
            driver.filtered_rangefinder_distance(frame([2] * 4), i / 10)
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([3] * 4), 0.7))
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([3] * 4), 0.8))
        self.assertEqual(driver.filtered_rangefinder_distance(frame([3] * 4), 0.9), 3)
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([3] * 4), 0.1))
        for timestamp in (float("nan"), float("inf"), -1):
            self.assertFalse(driver.rangefinder_frame_is_fresh({"time_of_transmission": timestamp}, 1))
        self.assertTrue(driver.rangefinder_frame_is_fresh({"time_of_transmission": 2e6}, 2))
        self.assertFalse(driver.rangefinder_frame_is_fresh({"time_of_transmission": 1e6}, 1))
        self.assertTrue(driver.rangefinder_frame_is_fresh({"time_of_transmission": 1.1e6}, 1.1))

    def test_handle_velocity_never_republishes_on_invalid_or_duplicate(self):
        driver = dvl.DvlDriver()
        sample = dict(frame([2] * 4), vx=0, vy=0, vz=0, velocity_valid=True, fom=0, time=100)
        with patch.object(driver.mav, "send_dvl_beams"), patch.object(driver.mav, "send_odometry"), patch.object(
            driver.mav, "send_rangefinder"
        ) as send, patch("dvl.time.time", return_value=10), patch("dvl.time.monotonic", return_value=10):
            for timestamp in (9.8e6, 9.9e6, 10e6):
                sample["time_of_transmission"] = timestamp
                driver.handle_velocity(sample)
            self.assertEqual(send.call_count, 1)
            driver.last_rangefinder_send_time = 0
            driver.handle_velocity(sample)
            sample["velocity_valid"] = False
            driver.handle_velocity(sample)
            self.assertEqual(send.call_count, 1)


class LifecycleTest(unittest.TestCase):
    def test_source_orientation_and_connection_reset(self):
        driver = dvl.DvlDriver()
        with patch.object(driver, "save_settings"):
            for action in (
                lambda: driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_REPORTED),
                lambda: driver.set_rangefinder_distance_source(dvl.RANGEFINDER_DISTANCE_BEAM_MEDIAN),
                lambda: driver.set_orientation(dvl.DVL_DOWN_REVERSED),
                lambda: driver.setup_connections(timeout=0),
            ):
                driver.rangefinder_filter.update(2, 0)
                driver.rangefinder_filter.update(2.3, 0.1)
                action()
                self.assertIsNone(driver.rangefinder_filter.accepted)
                self.assertIsNone(driver.rangefinder_filter.candidate)
                self.assertFalse(driver.rangefinder_beam_histories)

    def test_invalid_velocity_cancels_pending_confirmation(self):
        driver = dvl.DvlDriver()
        driver.rangefinder_filter.update(2, 0)
        driver.rangefinder_filter.update(2.3, 0.1)
        with patch.object(driver.mav, "send_dvl_beams"):
            driver.handle_velocity(dict(frame([2] * 4), vx=0, vy=0, vz=0, velocity_valid=False, fom=1, time=100))
        self.assertIsNone(driver.rangefinder_filter.candidate)
        self.assertIsNone(driver.rangefinder_filter.accepted)


class AdditionalCoverageTest(unittest.TestCase):
    def test_candidate_band_allows_noise_without_reanchoring(self):
        filt = RangePersistenceFilter()
        filt.update(2, 0)
        for i in range(1, 7):
            output = filt.update(2.3 + (0.01 if i % 2 else -0.01), i / 10)
        self.assertTrue(filt.confirmed)
        self.assertAlmostEqual(filt.candidate, 2.31)
        self.assertGreater(output, 2)

    def test_mixed_depth_beam_switch_at_multiple_rates(self):
        for rate in (5, 10, 20, 50):
            driver = dvl.DvlDriver()
            for i in range(rate):
                driver.filtered_rangefinder_distance(frame([1.8, 2, 2.3, 2.4]), i / rate)
            # Only the second beam changes; other beams see different depths.
            for i in range(rate, rate * 3):
                result = driver.filtered_rangefinder_distance(frame([1.8, 2.3, 2.3, 2.4]), i / rate)
                if i / rate < 1 + 1 / rate + 0.5 - 1e-9:
                    self.assertAlmostEqual(result, 2)
            self.assertGreater(result, 2.2)
            self.assertLess(result, 2.3)

    def test_gap_does_not_confirm_old_candidate(self):
        driver = dvl.DvlDriver()
        for i in range(3):
            driver.filtered_rangefinder_distance(frame([2] * 4), i / 10)
        for timestamp in (0.3, 0.4, 0.5):
            driver.filtered_rangefinder_distance(frame([2.3] * 4), timestamp)
        self.assertIsNotNone(driver.rangefinder_filter.candidate)
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([2.3] * 4), 1.0))
        self.assertIsNone(driver.rangefinder_filter.candidate)
        self.assertIsNone(driver.filtered_rangefinder_distance(frame([2.3] * 4), 1.1))
        self.assertEqual(driver.filtered_rangefinder_distance(frame([2.3] * 4), 1.2), 2.3)
