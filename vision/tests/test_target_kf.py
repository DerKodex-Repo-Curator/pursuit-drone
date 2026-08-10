"""Tests for the target Kalman filter.

The interesting properties are not "does it filter" but the three things it
exists to fix: it should smooth without lagging badly, it should keep a useful
estimate through a gap in detections and say how stale that estimate is, and
its uncertainty should grow while coasting so a caller can tell an estimate
from a guess.
"""
from __future__ import annotations

import numpy as np
import pytest

from pursuit.target_kf import TargetKF


DT = 1.0 / 30.0


def feed(kf: TargetKF, points, dt=DT):
    for cx, cy, h in points:
        kf.predict(dt)
        kf.update(cx, cy, h)


def test_an_unseeded_filter_offers_nothing():
    kf = TargetKF()
    assert not kf.valid
    kf.predict(DT)            # must not explode before it has ever seen data
    assert not kf.valid


def test_first_measurement_seeds_position_exactly():
    """No velocity is knowable from one point, so position should be taken at
    face value rather than blended against a fabricated prior."""
    kf = TargetKF()
    kf.update(320.0, 240.0, 180.0)
    assert kf.valid
    assert kf.centre == pytest.approx((320.0, 240.0))
    assert kf.box_h == pytest.approx(180.0)
    assert kf.velocity == pytest.approx((0.0, 0.0))


def test_it_learns_a_constant_velocity():
    kf = TargetKF()
    v = 120.0                                  # px/s
    feed(kf, [(100.0 + v * i * DT, 240.0, 180.0) for i in range(40)])
    vx, vy = kf.velocity
    assert vx == pytest.approx(v, rel=0.10)
    assert vy == pytest.approx(0.0, abs=8.0)


def test_velocity_is_per_second_not_per_frame():
    """The old lead computation worked in px/frame and divided by dt at the
    point of use, so a dropped frame changed the number's MEANING. Feeding the
    same physical motion at two frame rates must give the same velocity."""
    a, b = TargetKF(), TargetKF()
    v = 150.0
    feed(a, [(100.0 + v * i * (1 / 30), 240.0, 180.0) for i in range(60)], 1 / 30)
    feed(b, [(100.0 + v * i * (1 / 15), 240.0, 180.0) for i in range(30)], 1 / 15)
    assert a.velocity[0] == pytest.approx(b.velocity[0], rel=0.12)


def test_it_smooths_measurement_noise():
    """The whole point of filtering: the estimate should sit much closer to the
    truth than the individual measurements do."""
    rng = np.random.default_rng(0)
    kf = TargetKF()
    truth = 320.0
    noisy = [truth + rng.normal(0, 12.0) for _ in range(60)]
    feed(kf, [(n, 240.0, 180.0) for n in noisy])

    meas_err = float(np.sqrt(np.mean((np.array(noisy) - truth) ** 2)))
    filt_err = abs(kf.centre[0] - truth)
    assert filt_err < meas_err * 0.6


def test_it_coasts_through_a_gap_in_detections():
    """The face stage runs on about a quarter of frames and detections drop
    out behind furniture. A stale aim point with no notion of staleness is
    what this replaces."""
    kf = TargetKF()
    v = 200.0
    feed(kf, [(100.0 + v * i * DT, 240.0, 180.0) for i in range(30)])
    x_at_gap = kf.centre[0]

    for _ in range(9):                         # 300 ms with nothing
        kf.predict(DT)

    assert kf.valid                            # still inside max_coast_s
    assert kf.centre[0] > x_at_gap             # it kept moving
    assert kf.centre[0] == pytest.approx(x_at_gap + v * 9 * DT, rel=0.15)


def test_coasting_too_long_stops_being_offered():
    """Coasting is for occlusion, not for inventing a target that has left."""
    kf = TargetKF(max_coast_s=0.5)
    feed(kf, [(320.0, 240.0, 180.0) for _ in range(10)])
    assert kf.valid
    for _ in range(20):                        # 660 ms
        kf.predict(DT)
    assert not kf.valid


def test_uncertainty_grows_while_coasting():
    """This is what lets a caller distinguish a confident estimate from an
    extrapolation, which the old zero-velocity signal could not."""
    kf = TargetKF()
    feed(kf, [(320.0, 240.0, 180.0) for _ in range(30)])
    settled = kf.pos_std_est
    for _ in range(10):
        kf.predict(DT)
    assert kf.pos_std_est > settled * 1.5


def test_a_measurement_after_coasting_pulls_it_back():
    kf = TargetKF()
    feed(kf, [(320.0, 240.0, 180.0) for _ in range(30)])
    for _ in range(8):
        kf.predict(DT)
    coasted = kf.pos_std_est
    kf.update(320.0, 240.0, 180.0)
    assert kf.pos_std_est < coasted


def test_predict_ahead_is_the_lead_the_controller_wants():
    kf = TargetKF()
    v = 240.0
    feed(kf, [(100.0 + v * i * DT, 240.0, 180.0) for i in range(40)])
    cx, _ = kf.centre
    ahead_x, _ = kf.predict_ahead(0.1)
    assert ahead_x - cx == pytest.approx(v * 0.1, rel=0.12)


def test_height_is_filtered_too_because_it_is_the_range_cue():
    rng = np.random.default_rng(1)
    kf = TargetKF()
    truth = 200.0
    feed(kf, [(320.0, 240.0, truth + rng.normal(0, 18.0)) for _ in range(50)])
    assert abs(kf.box_h - truth) < 12.0


def test_non_finite_measurements_are_refused_not_absorbed():
    """A NaN reaching the state makes every subsequent output NaN, and it
    would propagate silently into an attitude setpoint."""
    kf = TargetKF()
    feed(kf, [(320.0, 240.0, 180.0) for _ in range(10)])
    before = kf.centre
    kf.update(float("nan"), 240.0, 180.0)
    kf.update(320.0, float("inf"), 180.0)
    assert kf.centre == pytest.approx(before)
    assert np.all(np.isfinite(kf.x))


def test_covariance_stays_symmetric_over_a_long_run():
    """Joseph form is used precisely so this holds for a whole flight; the
    textbook (I-KH)P loses symmetry to rounding and eventually goes
    indefinite."""
    rng = np.random.default_rng(2)
    kf = TargetKF()
    for i in range(4000):
        kf.predict(DT)
        kf.update(320 + rng.normal(0, 10), 240 + rng.normal(0, 10),
                  180 + rng.normal(0, 15))
    assert np.allclose(kf.P, kf.P.T, atol=1e-8)
    assert np.all(np.linalg.eigvalsh(kf.P) > -1e-9)


def test_reset_clears_everything():
    kf = TargetKF()
    feed(kf, [(320.0, 240.0, 180.0) for _ in range(20)])
    kf.reset()
    assert not kf.valid
    assert kf.updates == 0
