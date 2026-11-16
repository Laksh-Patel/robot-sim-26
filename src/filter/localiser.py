from __future__ import annotations
from typing import Optional, Tuple
import math
import numpy as np

from src.filter.kf_utils import *
from src.filter.ukf import *
from src.constants import ARENA

# ============================================================
# UKF Localiser (NO SLAM)
# ============================================================

class UKFLocaliser:
    """
    Lightweight UKF localisation.

    State order:
        0 px
        1 py
        2 theta
        3 v
        4 omega
    """
    IX, IY, ITH, IV, IOM = 0, 1, 2, 3, 4

    def __init__(self, config: Optional[LocaliserConfig] = None):
        self.cfg = config or LocaliserConfig()
        self.ukf = UKF(alpha=self.cfg.alpha, beta=self.cfg.beta, kappa=self.cfg.kappa)

        # Initial mean and covariance (tune to your start conditions)
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=float)
        self.P = np.diag([0.20, 0.20, 0.35, 0.50, 1.00])**2  # initial uncertainty
        self.P = ensure_spd(self.P)

        # Process noise covariance Q (fixed; could be dt-scaled if you want)
        self.Q = np.diag([
            self.cfg.sigma_x,
            self.cfg.sigma_y,
            self.cfg.sigma_theta,
            self.cfg.sigma_v,
            self.cfg.sigma_omega,
        ])**2

        self.x[self.ITH] = wrap_angle(float(self.x[self.ITH]))
        self.x_angle_indices = [self.ITH]

        # Last state that was anchored by a real pose measurement.
        # None until the first successful update_pose call.
        self._x_trusted: Optional[np.ndarray] = None
        self._P_trusted: Optional[np.ndarray] = None

    # ------------------------
    # Motor model + motion model
    # ------------------------

    def motor_to_cmd(self, power_left: float, power_right: float) -> tuple[float, float]:
        """
        Convert motor powers to linear and angular velocity commands using the calibration curves.
        
        Calibration data (from linear regression):
        Linear velocity  (both motors equal):  v = 1.38915 * p - 0.11375   (m/s)
        Angular velocity (spin on spot, ±p):   ω = 7.18647 * p - 0.648571  (rad/s)
        
        Wheel separation: 2d = 0.32 m  →  d = 0.16 m
        """

        # --- Calibration constants ---
        V_SLOPE     =  1.526   # m/s per unit power
        V_INTERCEPT = -0.1363   # m/s offset

        W_SLOPE     =  7.43113   # rad/s per unit power
        W_INTERCEPT = -0.723939  # rad/s offset

        def signed_linear(p: float) -> float:
            multiplier = 1 if p >= 0 else -1
            abs_p = abs(p)
            speed = max(0, (V_SLOPE * abs_p + V_INTERCEPT)) # Keeps speed non-negative to account for deadzone
            return speed * multiplier

        def signed_angular(p: float) -> float:
            multiplier = 1 if p >= 0 else -1
            abs_p = abs(p)
            speed = max(0, (W_SLOPE * abs_p + W_INTERCEPT)) # Keeps speed non-negative to account for deadzone
            return speed * multiplier

        # Split power into average and differential components
        p_avg  = (power_left + power_right) / 2.0   # drives forward/back
        p_diff = (power_right - power_left) / 2.0   # drives rotation (right > left → positive omega → CW/right turn)

        linear_velocity  = signed_linear(p_avg)
        angular_velocity = signed_angular(p_diff)

        return (linear_velocity, angular_velocity)

    def predict(self, uL: float, uR: float, dt: float) -> None:
        """
        UKF prediction step.
        """
        Xsig, Wm, Wc = self.ukf.sigma_points(self.x, self.P)
        v_cmd, om_cmd = self.motor_to_cmd(uL, uR)

        # Vectorised motion model: apply to all sigma points at once instead of
        # calling motion_model() 11 times in a Python loop.
        Xp = Xsig.copy()                          # (2n+1, 5)

        a = self.cfg.v_cmd_smoothing
        b = self.cfg.om_cmd_smoothing
        v  = (1.0 - a) * Xp[:, self.IV]  + a * v_cmd
        om = (1.0 - b) * Xp[:, self.IOM] + b * om_cmd

        v  = np.clip(v,  -2.0,  2.0)
        om = np.clip(om, -15.0, 15.0)

        th = (Xp[:, self.ITH] + om * dt + np.pi) % (2.0 * np.pi) - np.pi
        Xp[:, self.IX]  = Xp[:, self.IX]  + v * np.cos(th) * dt
        Xp[:, self.IY]  = Xp[:, self.IY]  + v * np.sin(th) * dt
        Xp[:, self.ITH] = th
        Xp[:, self.IV]  = v
        Xp[:, self.IOM] = om

        x_pred, P_pred = self.ukf.unscented_mean_cov(Xp, Wm, Wc, self.x_angle_indices)

        self.x = x_pred
        self.P = ensure_spd(P_pred + self.Q)
        self._clamp_to_arena()

    # ------------------------
    # Pose measurement update
    # ------------------------

    def update_pose(self, meas: PoseMeasurement) -> None:
        """
        Update with z=[x,y,theta].
        Works for both triangulation and your wall-pose solver.
        """
        z = np.asarray(meas.z, dtype=float).reshape(3)
        R = np.asarray(meas.R, dtype=float).reshape(3, 3)

        def h(x: np.ndarray) -> np.ndarray:
            return np.array([x[self.IX], x[self.IY], x[self.ITH]], dtype=float)

        self.x, self.P = self.ukf.update(
            self.x, self.P, z, R,
            h=h,
            x_angle_indices=self.x_angle_indices,
            z_angle_indices=[2],
        )
        self._clamp_to_arena()

        # Checkpoint: this is now a trustworthy anchored state.
        self._x_trusted = self.x.copy()
        self._P_trusted = self.P.copy()

    def _gate_measurement(
        self,
        z: np.ndarray,
        wall_marker_missing_iterations: int,
        position_threshold_m: float = 1.5, # TODO check this value
    ) -> Tuple[bool, float]:
        """
        Check whether an incoming pose measurement z=[x, y, theta] is plausible
        given the current filter state, and decide how much to trust it.

        The jump is measured against the *trusted checkpoint* (last successfully
        anchored state), not the current (possibly drifted) filter state. This
        means the gate never locks out genuine corrections just because the filter
        has wandered.

        Three cases:
          - Jump ≤ threshold from trusted state: accept normally (r_scale=1.0).
          - Jump > threshold AND no trusted checkpoint exists yet, OR
            missing_iterations >= 5 (filter has been flying blind):
              restore the trusted checkpoint (inflating P to reflect accumulated
              uncertainty), then accept the measurement at normal R. The filter
              will snap directly to the camera's answer rather than dragging
              toward it from a bad starting point.
          - Jump > threshold AND missing_iterations < 5 (recently anchored):
              this is almost certainly a sensor glitch — hard-reject.

        Returns:
            (accepted: bool, r_scale: float)
        """
        # Use the trusted checkpoint position for the jump calculation if available,
        # otherwise fall back to the current filter state.
        if self._x_trusted is not None:
            ref_x = float(self._x_trusted[self.IX])
            ref_y = float(self._x_trusted[self.IY])
        else:
            ref_x, ref_y, _ = self.pose()

        jump = math.sqrt((z[0] - ref_x) ** 2 + (z[1] - ref_y) ** 2)

        if jump <= position_threshold_m:
            return True, 1.0

        # Large jump relative to last trusted position.
        if self._x_trusted is None or wall_marker_missing_iterations >= 5:
            # Filter has drifted far from its last known-good state. Restore the
            # trusted checkpoint and inflate P to reflect dead-reckoning uncertainty
            # accumulated since then, then let the measurement correct from there.
            if self._x_trusted is not None:
                self.x = self._x_trusted.copy()
                # Inflate position and heading uncertainty; keep velocity states as-is.
                P_restored = self._P_trusted.copy()
                drift_inflation = np.diag([
                    (ARENA.ARENA_HALF_EDGE ** 2) / 3.0,   # px: uniform-arena uncertainty
                    (ARENA.ARENA_HALF_EDGE ** 2) / 3.0,   # py
                    (math.pi ** 2) / 3.0,                 # theta: uniform over [-pi, pi]
                    0.0, 0.0,                             # velocities: keep trusted values
                ])
                self.P = ensure_spd(P_restored + drift_inflation)
                print(f"GATE: filter drifted {jump:.2f} m — restored trusted checkpoint, accepting measurement")
            # else:
            #     print(f"GATE: no trusted checkpoint yet, accepting measurement unconditionally")
            return True, 1.0

        # Recently anchored but huge jump — sensor glitch, reject.
        print(f"GATE: rejected measurement jump of {jump:.2f} m (missing_iters={wall_marker_missing_iterations})")
        return False, 1.0

    def update_zero_velocity(self) -> None:
        """
        Inject a zero-velocity pseudo-measurement: z = [v=0, omega=0].

        Call this when both motor powers are 0. It pins the velocity states
        to zero so they cannot drift during dead-reckoning ticks without a
        wall-marker correction. The measurement noise R is tight (0.01 m/s,
        0.05 rad/s std dev) because commanded-zero is highly reliable.
        """
        z = np.array([0.0, 0.0], dtype=float)
        R = np.diag([0.01**2, 0.05**2])

        def h(x: np.ndarray) -> np.ndarray:
            return np.array([x[self.IV], x[self.IOM]], dtype=float)

        theta_before = float(self.x[self.ITH])
        self.x, self.P = self.ukf.update(
            self.x, self.P, z, R,
            h=h,
            x_angle_indices=self.x_angle_indices,
            z_angle_indices=[],
        )
        self.x[self.ITH] = theta_before  # prevent zero-velocity cross-covariance from corrupting heading

    # ------------------------
    # Convenience: build R from "quality"
    # ------------------------

    def make_pose_R(self, sigma_x: float, sigma_y: float, sigma_theta: float, scale: float = 1.0) -> np.ndarray:
        return np.diag([sigma_x, sigma_y, sigma_theta])**2 * float(scale)

    def make_R_from_wall_marker_count(self, n_markers: int) -> np.ndarray:
        """
        Cheap heuristic: more markers => more confident.
        Adjust numbers after testing.
        """
        # baseline sigmas
        sx, sy, sth = self.cfg.meas_sigma_x, self.cfg.meas_sigma_y, self.cfg.meas_sigma_theta

        if n_markers <= 0:
            return self.make_pose_R(999.0, 999.0, 999.0)  # effectively "ignore"
        if n_markers == 1:
            # one marker pose is weak: very noisy heading in particular
            return self.make_pose_R(sx * 2.2, sy * 2.2, sth * 2.8, scale=self.cfg.wallpose_R_scale)
        if n_markers == 2:
            return self.make_pose_R(sx * 1.4, sy * 1.4, sth * 1.6, scale=self.cfg.wallpose_R_scale)
        # 3+ markers
        return self.make_pose_R(sx * 0.9, sy * 0.9, sth * 0.9, scale=self.cfg.wallpose_R_scale)

    def make_R_triangulation_default(self) -> np.ndarray:
        """
        Default triangulation covariance. Often you can trust this more than 1-marker pose.
        """
        return self.make_pose_R(
            self.cfg.meas_sigma_x,
            self.cfg.meas_sigma_y,
            self.cfg.meas_sigma_theta,
            scale=self.cfg.triangulation_R_scale,
        )

    # ------------------------
    # Arena constraint
    # ------------------------

    def _clamp_to_arena(self) -> None:
        """
        Hard-constrain the position estimate to the physical arena bounds.

        When a dimension is clamped the filter state is also corrected:
          - The clamped axis's covariance row/column is collapsed to a small
            diagonal value (we now have high confidence about position on that axis).
          - Linear velocity is zeroed (the robot hit a boundary and stopped).

        Call this after every step that writes to self.x (predict & update_pose).
        """
        for axis in (self.IX, self.IY):
            val = float(self.x[axis])
            clamped = float(np.clip(val, -ARENA.ARENA_HALF_EDGE, ARENA.ARENA_HALF_EDGE))
            if clamped != val:
                self.x[axis] = clamped
                self.x[self.IV] = 0.0       # linear velocity → 0, robot is against the wall
                # Inflate position uncertainty: clamping means the filter has diverged,
                # not that we know where we are. Use variance = (half_arena)^2 / 3,
                # the variance of a uniform distribution across the full arena length,
                # to express "we have essentially no idea where in the arena we are".
                self.P[axis, axis] = (ARENA.ARENA_HALF_EDGE ** 2) / 3.0

    # ------------------------
    # Outputs
    # ------------------------

    def pose(self) -> Tuple[float, float, float]:
        return float(self.x[self.IX]), float(self.x[self.IY]), wrap_angle(float(self.x[self.ITH]))

    def velocities(self) -> Tuple[float, float]:
        return float(self.x[self.IV]), float(self.x[self.IOM])

    def state(self) -> np.ndarray:
        return self.x.copy()

    def covariance(self) -> np.ndarray:
        return self.P.copy()

def localisation_tick(
    localiser: UKFLocaliser,
    *,
    uL: float,
    uR: float,
    dt: float,
    triangulation_pose: Optional[Tuple[float, float, float]] = None,
    wall_pose: Optional[Tuple[float, float, float]] = None,
    wall_marker_count: int = 0,
    wall_marker_missing_iterations: int = 0,
) -> None:
    """
    Per-tick sequence:
        1) predict with motor powers
        2) zero-velocity update if motors are off
        3) triangulation update (gated)
        4) wall-pose update (gated)
    """
    localiser.predict(uL, uR, dt)

    # 0) Zero-velocity update: if motors are commanded off, pin v and omega to 0.
    if uL == 0.0 and uR == 0.0:
        localiser.update_zero_velocity()

    # 1) Triangulation update (if available, valid, and passes jump gate)
    if triangulation_pose is not None and not any(
        v is None or (isinstance(v, float) and np.isnan(v)) for v in triangulation_pose
    ):
        z = np.array(triangulation_pose, dtype=float)
        accepted, r_scale = localiser._gate_measurement(z, wall_marker_missing_iterations)
        if accepted:
            R = localiser.make_R_triangulation_default()
            R = R * r_scale
            localiser.update_pose(PoseMeasurement(z=z, R=R, source="triangulation"))

    # 2) Wall-pose update (if available, valid, and passes jump gate)
    if wall_pose is not None and not any(
        v is None or (isinstance(v, float) and np.isnan(v)) for v in wall_pose
    ):
        z = np.array(wall_pose, dtype=float)
        accepted, r_scale = localiser._gate_measurement(z, wall_marker_missing_iterations)
        if accepted:
            R = localiser.make_R_from_wall_marker_count(wall_marker_count)
            R = R * r_scale
            localiser.update_pose(PoseMeasurement(z=z, R=R, source="wall_pose"))
