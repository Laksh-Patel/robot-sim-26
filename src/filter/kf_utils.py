from __future__ import annotations
import math
import numpy as np

# ============================================================
# Angle utilities
# ============================================================

def wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def angle_mean(values: np.ndarray, weights: np.ndarray) -> float:
    """Circular mean of angles in radians."""
    s = float(np.sum(weights * np.sin(values)))
    c = float(np.sum(weights * np.cos(values)))
    return math.atan2(s, c)

def ensure_spd(P: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    P = np.asarray(P, dtype=float)
    P = 0.5 * (P + P.T)

    # Fast path: Cholesky succeeds only for positive-definite matrices, and is
    # ~3x cheaper than a full eigendecomposition for small matrices (5x5 here).
    try:
        np.linalg.cholesky(P)
        return P
    except np.linalg.LinAlgError:
        pass

    # Fallback: fix eigenvalues to enforce positive-definiteness.
    eigvals, eigvecs = np.linalg.eigh(P)
    eigvals = np.maximum(eigvals, eps)
    P = eigvecs @ np.diag(eigvals) @ eigvecs.T
    return 0.5 * (P + P.T)

def extract_wall_markers(markers) -> list:
    """Filter SR camera markers down to wall markers only."""
    return [m for m in markers if m.id < 20]

def count_wall_markers(markers) -> int:
    """Convenience: number of wall markers currently visible."""
    return len(extract_wall_markers(markers))
