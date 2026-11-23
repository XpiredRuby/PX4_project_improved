"""Prediction-error metrics and an empirical error envelope with coverage checking."""

from __future__ import annotations

import numpy as np


def error_norm(predicted: np.ndarray, truth: np.ndarray) -> np.ndarray:
    return np.linalg.norm(predicted - truth, axis=1)


def summarize(errors: np.ndarray) -> dict:
    e = np.asarray(errors, float)
    return {
        "rmse_m": float(np.sqrt(np.mean(e**2))),
        "p50_m": float(np.percentile(e, 50)),
        "p95_m": float(np.percentile(e, 95)),
        "max_m": float(np.max(e)),
        "samples": int(e.size),
    }


def envelope(train_errors: np.ndarray, level: float = 0.95) -> float:
    """Error bound set from training data only; claimed to hold at `level` on unseen flights."""
    return float(np.quantile(np.asarray(train_errors, float), level))


def coverage(test_errors: np.ndarray, bound: float) -> float:
    return float(np.mean(np.asarray(test_errors, float) <= bound))
