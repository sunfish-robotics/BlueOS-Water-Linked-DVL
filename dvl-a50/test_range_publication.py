"""Exercise range publication timing without starting threads or sending MAVLink."""

import unittest
from unittest.mock import Mock, patch

from test_dvl import dvl
from test_range_filter import frame


class RangePublicationTest(unittest.TestCase):
    def setUp(self):
        self.driver = dvl.DvlDriver()
        self.driver.mav = Mock()

    def publish(self, arrival, valid=True, values=None, source_time=None):
        sample = dict(
            frame([2] * 4 if values is None else values),
            vx=0,
            vy=0,
            vz=0,
            velocity_valid=valid,
            fom=0,
            time=100,
            time_of_transmission=(arrival if source_time is None else source_time) * 1e6,
        )
        with patch("dvl.time.time", return_value=arrival), patch("dvl.time.monotonic", return_value=arrival):
            self.driver.handle_velocity(sample)

    def test_normal_rates_and_jitter_publish_every_mature_sample(self):
        for rate in (5, 9, 10, 15):
            with self.subTest(rate=rate):
                self.setUp()
                for i in range(40):
                    self.publish(100 + i / rate + (0.003 if i % 2 else 0))
                self.assertEqual(self.driver.mav.send_rangefinder.call_count, 38)

    def test_fast_input_remains_rate_limited(self):
        sent_at = []
        for i in range(200):
            arrival = 100 + i / 1000
            before = self.driver.mav.send_rangefinder.call_count
            self.publish(arrival)
            if self.driver.mav.send_rangefinder.call_count > before:
                sent_at.append(arrival)
        self.assertGreater(len(sent_at), 1)
        self.assertTrue(all(b - a >= 0.05 for a, b in zip(sent_at, sent_at[1:])))

    def test_invalid_input_still_suppresses_range_and_requires_recovery(self):
        for bad in (
            {"valid": False},
            {"values": [2]},
            {"values": [float("nan")] * 4},
            {"source_time": 100.2},  # Duplicate last accepted sample.
            {"source_time": 99},  # Stale.
            {"source_time": 102},  # Future.
        ):
            with self.subTest(bad=bad):
                self.setUp()
                for i in range(3):
                    self.publish(100 + i / 10)
                self.assertEqual(self.driver.mav.send_rangefinder.call_count, 1)
                self.publish(100.3, **bad)
                self.assertEqual(self.driver.mav.send_rangefinder.call_count, 1)
                for arrival in (100.4, 100.5):
                    self.publish(arrival)
                    self.assertEqual(self.driver.mav.send_rangefinder.call_count, 1)
                self.publish(100.6)
                self.assertEqual(self.driver.mav.send_rangefinder.call_count, 2)

    def test_gap_and_disabled_output_do_not_publish_held_range(self):
        for i in range(3):
            self.publish(100 + i / 10)
        self.publish(101)
        self.publish(101.1)
        self.assertEqual(self.driver.mav.send_rangefinder.call_count, 1)
        self.publish(101.2)
        self.assertEqual(self.driver.mav.send_rangefinder.call_count, 2)
        self.driver.rangefinder = False
        self.publish(101.3)
        self.assertEqual(self.driver.mav.send_rangefinder.call_count, 2)
