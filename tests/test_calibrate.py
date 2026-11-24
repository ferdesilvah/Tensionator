"""Tests for tools/calibrate_plates.py (no hardware needed).

    python tests/test_calibrate.py      (or: pytest tests/)
"""
from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from calibrate_plates import Point, fit, main  # noqa: E402


def test_fit_recovers_scale_and_flags_hysteresis():
    off, s = 1000.0, -2150.0
    pts = [Point("zero", 0, 0, 0, off, 50, 100, True, 0)]
    for i, kg in enumerate((10, 20, 30), 1):
        F = kg * 9.80665
        pts.append(Point("load", i, kg, F, off + s * F, 50, 100, True, 0))
    F = 20 * 9.80665
    pts.append(Point("unload", 2, 20, F, off + s * (F + 0.5), 50, 100, True, 0))   # reads 0.5 N high
    r = fit(pts, off)
    assert abs(r["scale"] - s) / abs(s) < 0.002
    expected = 100 * 0.5 / (30 * 9.80665)
    assert abs(r["hysteresis_pct_fs"] - expected) / expected < 0.01


def test_sim_calibration_stores_fitted_scale():
    with tempfile.TemporaryDirectory() as tmp:
        rc = main(["--sim", "--plates", "10,20", "--hanger-kg", "1", "--no-unload", "--out", tmp])
        assert rc == 0
        js = json.loads(next(Path(tmp).glob("*_platesim.json")).read_text())
        assert abs(js["applied"]["diff_pct"]) < 0.2
        assert abs(js["applied"]["stored_scale"] - (-2143.7)) / 2143.7 < 0.01   # sim's true scale
        assert (Path(tmp) / "cal_history.csv").exists()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok ", name)
    print("all tests passed")
