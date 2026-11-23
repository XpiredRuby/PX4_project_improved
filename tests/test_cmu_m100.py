import numpy as np

from fidelity import cmu_m100 as cm


def _flight(convention, wind=(3.0, -2.0), n=600):
    t = np.arange(n) * 0.2
    heading = np.where(t < 60, 0.3, 2.4)  # two legs with different courses
    v = 6.0 * np.column_stack([np.sin(heading), np.cos(heading)])
    rel = np.array(wind) - v  # relative airflow seen on board
    frame, sense, sign = convention
    yaw = np.pi / 2 - heading
    bearing = np.arctan2(sign * rel[:, 0], sign * rel[:, 1])
    if frame == "body":
        bearing = bearing - (np.pi / 2 - yaw)
    angle = bearing * (1 if sense == "cw" else -1)
    return {
        "t": t,
        "v": v,
        "alt": np.full(n, 30.0),
        "yaw": yaw,
        "air_speed": np.linalg.norm(rel, axis=1),
        "air_angle": angle,
        "east": np.cumsum(v[:, 0]) * 0.2,
        "north": np.cumsum(v[:, 1]) * 0.2,
    }


def test_rotation_identity_and_yaw():
    r = cm.rotation(np.array([[0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]]))[0]
    assert np.allclose(r @ np.array([1.0, 0, 0]), [0, 1, 0], atol=1e-9)


def test_wind_is_recovered_with_the_true_convention():
    for conv in [("world", "cw", -1), ("body", "ccw", 1)]:
        d = _flight(conv)
        assert np.allclose(cm.wind(d, conv), [3.0, -2.0], atol=1e-6)


def test_best_convention_is_the_one_with_steady_wind():
    conv = ("world", "cw", -1)
    flights = [_flight(conv, wind=w) for w in ((3.0, -2.0), (-1.0, 4.0))]
    assert cm.best_convention(flights) == conv
