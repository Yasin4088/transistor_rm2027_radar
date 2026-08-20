import unittest

from frame_rate import RecentFrameRate


class RecentFrameRateTest(unittest.TestCase):
    def test_reports_rate_from_distinct_delivered_frames(self):
        rate = RecentFrameRate(window_seconds=2.0)

        self.assertIsNone(rate.snapshot(timestamp=0.0))
        rate.mark(timestamp=0.0)
        self.assertIsNone(rate.snapshot(timestamp=0.0))
        rate.mark(timestamp=0.5)
        rate.mark(timestamp=1.0)

        self.assertAlmostEqual(rate.snapshot(timestamp=1.0), 2.0)

    def test_reports_zero_after_a_previously_active_stream_stalls(self):
        rate = RecentFrameRate(window_seconds=2.0)
        rate.mark(timestamp=0.0)
        rate.mark(timestamp=0.5)

        self.assertEqual(rate.snapshot(timestamp=3.0), 0.0)


if __name__ == "__main__":
    unittest.main()
