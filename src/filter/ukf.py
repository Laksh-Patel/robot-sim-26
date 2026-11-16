from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, List, Tuple
import numpy as np

from src.filter.kf_utils import *

# ============================================================
# UKF Core
# ============================================================

class UKF:
    """
    Unscented Kalman Filter core utilities.
    """

    def __init__(self, alpha: float = 1e-2, beta: float = 2.0, kappa: float = 0.0):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.kappa = float(kappa)

    def sigma_points(self, x: np.ndarray, P: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns:
            X: (2n+1, n) sigma points
            Wm: (2n+1,) weights for mean
            Wc: (2n+1,) weights for covariance
        """
        x = np.asarray(x, dtype=float).reshape(-1)
        n = x.shape[0]

        lam = self.alpha**2 * (n + self.kappa) - n
        c = n + lam

        # Just symmetrise; try Cholesky directly to avoid a redundant ensure_spd
        # eigendecomposition in the normal (already-SPD) case.
        P_sym = 0.5 * (np.asarray(P, dtype=float) + np.asarray(P, dtype=float).T)
        try:
            S = np.linalg.cholesky(c * P_sym)
        except np.linalg.LinAlgError:
            S = np.linalg.cholesky(ensure_spd(c * P_sym, eps=1e-6))

        X = np.zeros((2 * n + 1, n), dtype=float)
        X[0] = x
        X[1:n + 1] = x + S.T       # vectorised: one slice per sigma-point block
        X[n + 1:2 * n + 1] = x - S.T

        Wm = np.full(2 * n + 1, 1.0 / (2.0 * c), dtype=float)
        Wc = np.full(2 * n + 1, 1.0 / (2.0 * c), dtype=float)
        Wm[0] = lam / c
        Wc[0] = lam / c + (1.0 - self.alpha**2 + self.beta)

        return X, Wm, Wc

    def unscented_mean_cov(
        self,
        X: np.ndarray,
        Wm: np.ndarray,
        Wc: np.ndarray,
        angle_indices: List[int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute mean and covariance of sigma points, with circular mean on angle dims.
        """
        X = np.asarray(X, dtype=float)
        Wm = np.asarray(Wm, dtype=float).reshape(-1)
        Wc = np.asarray(Wc, dtype=float).reshape(-1)
        n = X.shape[1]

        x_mean = np.zeros(n, dtype=float)
        ang = set(angle_indices)
        lin_idx = [i for i in range(n) if i not in ang]

        if lin_idx:
            x_mean[lin_idx] = np.sum(X[:, lin_idx] * Wm[:, None], axis=0)

        for j in angle_indices:
            x_mean[j] = angle_mean(X[:, j], Wm)

        # Vectorised covariance: build all residuals at once, then use einsum
        # instead of accumulating n outer products in a Python loop.
        dX = X - x_mean                        # (2n+1, n)
        for j in angle_indices:
            dX[:, j] = (dX[:, j] + np.pi) % (2.0 * np.pi) - np.pi
        P = np.einsum('i,ij,ik->jk', Wc, dX, dX)

        return x_mean, ensure_spd(P)

    def update(
        self,
        x: np.ndarray,
        P: np.ndarray,
        z: np.ndarray,
        R: np.ndarray,
        h: Callable[[np.ndarray], np.ndarray],
        x_angle_indices: List[int],
        z_angle_indices: List[int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Generic UKF measurement update.
        """
        x = np.asarray(x, dtype=float).reshape(-1)
        P = np.asarray(P, dtype=float)
        z = np.asarray(z, dtype=float).reshape(-1)
        R = np.asarray(R, dtype=float)

        X, Wm, Wc = self.sigma_points(x, P)

        Zsig = np.array([h(Xi) for Xi in X], dtype=float)  # (2n+1, m)
        m = Zsig.shape[1]

        if R.shape != (m, m):
            raise ValueError(f"R must be shape ({m},{m}), got {R.shape}")

        # predicted measurement mean
        z_pred = np.zeros(m, dtype=float)
        angz = set(z_angle_indices)
        linz = [i for i in range(m) if i not in angz]
        if linz:
            z_pred[linz] = np.sum(Zsig[:, linz] * Wm[:, None], axis=0)
        for j in z_angle_indices:
            z_pred[j] = angle_mean(Zsig[:, j], Wm)

        # Vectorised innovation covariance S and cross covariance Pxz.
        # Build all residuals as 2-D arrays, wrap angles in one broadcast op,
        # then use einsum instead of accumulating outer products in a Python loop.
        dZ = Zsig - z_pred                       # (2n+1, m)
        for j in z_angle_indices:
            dZ[:, j] = (dZ[:, j] + np.pi) % (2.0 * np.pi) - np.pi

        dX = X - x                               # (2n+1, n_x)
        for j in x_angle_indices:
            dX[:, j] = (dX[:, j] + np.pi) % (2.0 * np.pi) - np.pi

        S = ensure_spd(np.einsum('i,ij,ik->jk', Wc, dZ, dZ) + R)
        Pxz = np.einsum('i,ij,ik->jk', Wc, dX, dZ)
        K = Pxz @ np.linalg.inv(S)

        innovation = z - z_pred
        for j in z_angle_indices:
            innovation[j] = wrap_angle(float(innovation[j]))
        x_new = x + K @ innovation
        for j in x_angle_indices:
            x_new[j] = wrap_angle(float(x_new[j]))
        P_new = ensure_spd(P - K @ S @ K.T)

        return x_new, P_new

# ============================================================
# Localiser configuration / observations
# ============================================================

@dataclass
class LocaliserConfig:
    # UKF parameters
    # alpha=1.0 keeps lambda=0, giving Wm[0]=0 and all-positive weights.
    # alpha=0.3 gave lambda=-4.55 and Wm[0]=-10.11, which causes the mean to
    # oscillate wildly when P is large (the negative central weight fights the
    # peripheral ones, producing large position swings every predict step).
    alpha: float = 1.0
    beta: float = 2.0
    kappa: float = 0.0

    # Process noise standard deviations (per tick).
    # sigma_v and sigma_omega were 0.50 and 1.20 — far too large.
    # After 14 ticks without a measurement, velocity std dev reached ~1.9 m/s,
    # causing sigma points to span ±0.76m/tick in position, which combined with
    # the formerly-negative central weight produced the oscillating multi-metre drift.
    sigma_x: float = 0.08     # m
    sigma_y: float = 0.08     # m
    sigma_theta: float = 0.10 # rad
    sigma_v: float = 0.05     # m/s — tightened from 0.50; motor model is reasonably accurate
    sigma_omega: float = 0.30 # rad/s — tightened from 1.20; same rationale

    # First-order smoothing of commanded velocities into v/omega
    # Closer to 1 tracks the motor power velocities more closely
    v_cmd_smoothing: float = 0.95
    om_cmd_smoothing: float = 0.70

    # Measurement noise “floors” for pose sensors (triangulation & wall-pose)
    # You can override per measurement based on quality/marker count.
    meas_sigma_x: float = 0.12      # m
    meas_sigma_y: float = 0.12      # m
    meas_sigma_theta: float = 0.18  # rad

    # If you want to trust triangulation more than wall-pose, you can scale its R down:
    triangulation_R_scale: float = 0.3   # <1 means "trust more"
    wallpose_R_scale: float = 0.85       # ~1 means baseline

@dataclass
class PoseMeasurement:
    """
    A direct pose measurement z = [x, y, theta] in arena frame.
    """
    z: np.ndarray   # shape (3,)
    R: np.ndarray   # shape (3,3)
    source: str = "pose"

