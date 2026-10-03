"""Behavioral checks; no claim of vehicle-level safety certification."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
import numpy as np
from acc import inputs, simulate, controller, advance, metrics


class ACCBehavior(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.p, cls.scenarios = inputs()

    def test_suite_no_collisions_and_bounded_actuation(self):
        for h in [1., 1.5, 2.]:
            for s in self.scenarios:
                with self.subTest(headway=h, scenario=s["name"]):
                    p = dict(self.p, headway=h)
                    x = simulate(s, p)
                    self.assertTrue(np.isfinite(x[:, :10]).all())
                    self.assertGreaterEqual(x[:, 1:3].min(), 0)
                    self.assertLessEqual(x[:, 5:7].max(), 2+1e-9)
                    self.assertGreaterEqual(x[:, 5:7].min(), -4-1e-9)
                    self.assertFalse(metrics(x, s, p)["collision"])

    def test_free_cruise_converges(self):
        x = simulate(self.scenarios[0], self.p)
        self.assertLess(abs(x[-1, 1]-self.scenarios[0]["set_speed"]), .15)
        self.assertTrue((x[:, 7] == 0).all())
        self.assertLessEqual(np.max(abs(np.diff(x[:, 6])/self.p["dt"])), 2.5+1e-8)

    def test_baseline_tracking_and_emergency_selectivity(self):
        for s in self.scenarios:
            x = simulate(s, self.p)
            with self.subTest(scenario=s["name"]):
                self.assertLess(abs(x[-1, 1]-s["final_speed"]), .15)
                self.assertIsNotNone(metrics(x, s, self.p)["speed_settling_s"])
                if s["name"] != "hard_brake":
                    self.assertNotIn(2, x[:, 7])

    def test_emergency_hold_and_release(self):
        state = np.array([2., 0., 0., 0.])
        u, held, *_ = controller(0, 0, 8, True, 25, 0, state, self.p)
        self.assertEqual((u, held[0]), (-4, 2))
        _, cleared, *_ = controller(0, 0, 8, False, 25, 0, held, self.p)
        self.assertEqual(cleared[0], 0)
        _, moving_lead, *_ = controller(0, 3, 15, True, 25, 0, held, self.p)
        self.assertEqual(moving_lead[0], 1)

    def test_slowdown_recovers_headway(self):
        x = simulate(self.scenarios[2], self.p)
        self.assertLess(abs(x[-1, 3]-x[-1, 4]), .5)
        self.assertLess(abs(x[-1, 1]-x[-1, 2]), .15)

    def test_braking_stops_and_triggers_emergency(self):
        x = simulate(self.scenarios[3], self.p)
        self.assertIn(2, x[:, 7])
        self.assertLess(x[-1, 1], .1)
        self.assertGreater(x[:, 3].min(), 0)
        self.assertTrue((x[x[:, 7] == 2, 5] == -4).all())

    def test_road_clears_resumes_cruise(self):
        x = simulate(self.scenarios[5], self.p)
        self.assertTrue((x[x[:, 0] >= 25, 7] == 0).all())
        self.assertLess(abs(x[-1, 1]-self.scenarios[5]["set_speed"]), .15)

    def test_infeasible_gap_is_reported_not_clamped(self):
        s = dict(self.scenarios[3], ego_speed=25, lead_speed=0, initial_gap=6,
                 duration=8, events=[[0, 0]], settle_after=0)
        x = simulate(s, self.p)
        self.assertEqual(x[0, 7], 2)
        self.assertLess(x[:, 3].min(), 0)
        self.assertTrue(metrics(x, s, self.p)["collision"])

    def test_stopping_kinematics(self):
        v, distance = advance(1, -4, 1)
        self.assertEqual(v, 0)
        self.assertAlmostEqual(distance, .125)

    def test_antiwindup_under_long_saturation(self):
        state = np.array([0., 0., 100., 0.])
        for _ in range(2000):
            u, state, *_ = controller(0, 0, 100, False, 100, 0, state, self.p)
        self.assertEqual(state[1], 0)
        self.assertLessEqual(u, 2)

    def test_follow_hysteresis(self):
        _, cruise, *_ = controller(20, 20, 60, True, 25, 0,
                                    np.array([0., 0., 5., 0.]), self.p)
        _, follow, *_ = controller(20, 20, 60, True, 25, 0,
                                    np.array([1., 0., 0., 0.]), self.p)
        self.assertEqual(cruise[0], 0)
        self.assertEqual(follow[0], 1)

    def test_step_size_convergence(self):
        for s in self.scenarios:
            a = simulate(s, self.p)
            b = simulate(s, dict(self.p, dt=.01))[::2]
            with self.subTest(scenario=s["name"]):
                self.assertLess(np.max(abs(a[:, 1]-b[:, 1])), .3)
                self.assertLess(np.max(abs(a[:, 3]-b[:, 3])), 1.)


if __name__ == "__main__":
    unittest.main()
