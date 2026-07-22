#!/usr/bin/env python3
"""
Bias calibration tool — measure the bike's static roll imbalance.
No motors, no LQR. Just IMU + physics.

Steps:
  1. Verify IMU zero — place bike vertically, check roll reading
  2. Verify steer center — align front wheel, check steer reading
  3. Drop test — hold upright, release, measure initial roll acceleration

The roll acceleration in the first 0.3s of free fall gives the bias torque:
    bias_deg = roll_acc_dps2 / a4
    where a4 = m*g*h/I = 93.4 s⁻²  (from your bike's measured parameters)

Usage on Pi:
  ros2 run bike_controller calibrate_bias
"""

from __future__ import annotations

import math
import sys
import time
import threading
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState


# ── Bike physical constants ──────────────────────────────────────────────
MASS = 2.218          # kg
COM_HEIGHT = 0.105    # m
ROLL_INERTIA = 0.02445  # kg·m²
GRAVITY = 9.81        # m/s²

# a4 = m*g*h / I  (gravitational roll stiffness, units: s⁻²)
A4 = MASS * GRAVITY * COM_HEIGHT / ROLL_INERTIA  # ≈ 93.4

# Conversion: bias_deg = roll_acc_dps2 / A4
# (roll_acc in deg/s², A4 converts rad/s² → s⁻², so deg/s² / A4 = bias in deg/rad * rad = deg)


def roll_acc_to_bias_deg(roll_acc_dps2: float) -> float:
    """Convert roll angular acceleration (deg/s²) to bias angle (deg)."""
    roll_acc_radps2 = math.radians(roll_acc_dps2)
    bias_rad = roll_acc_radps2 / A4
    return math.degrees(bias_rad)


class BiasCalibrator(Node):
    # Servo positions (rad) — same as balance_executor defaults
    SERVO_DOWN_1 = 2.82
    SERVO_DOWN_2 = 0.72
    SERVO_SWING = 0.7854  # 45° full retraction
    SERVO_HALF_SWING = 0.30  # ~17°, enough to clear the ground

    def __init__(self):
        super().__init__('bias_calibrator')

        self.imu_sub = self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.state_sub = self.create_subscription(JointState, '/motor/state',
                                                   self._on_motor_state, 10)
        self.servo_pub = self.create_publisher(JointState, '/servo/command', 10)

        # Latest readings
        self._lock = threading.Lock()
        self.latest_roll_deg = 0.0
        self.latest_roll_rate_dps = 0.0
        self.latest_steer_deg = 0.0
        self.latest_imu_time = 0.0

        # Recorded data
        self.recording: list[tuple[float, float, float]] = []  # (t, roll_deg, roll_rate_dps)

        # Don't touch kickstands on startup — keep whatever state they're in
        self.get_logger().info("Bias Calibrator ready.")

    def _set_kickstands(self, lift_ratio: float):
        """lift_ratio: 0.0 = fully down, 1.0 = fully up, 0.38 = half-up"""
        ratio = max(0.0, min(1.0, lift_ratio))
        t1 = self.SERVO_DOWN_1 - ratio * self.SERVO_SWING
        t2 = self.SERVO_DOWN_2 + ratio * self.SERVO_SWING
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['servo_1', 'servo_2']
        msg.position = [float(t1), float(t2)]

        # Publish several times to ensure delivery
        for i in range(10):
            self.servo_pub.publish(msg)
            time.sleep(0.05)

        state = "UP" if ratio > 0.5 else ("half-up" if ratio > 0.1 else "DOWN")
        self.get_logger().error(
            f"*** KICKSTAND CMD: {state} (ratio={ratio:.0%}) "
            f"servo_1→{t1:.3f} servo_2→{t2:.3f} "
            f"published 10× ***"
        )

    def _on_imu(self, msg: Imu):
        from scipy.spatial.transform import Rotation
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        try:
            r = Rotation.from_quat(q)
            roll, pitch, yaw = r.as_euler("ZYX", degrees=True)
        except Exception:
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self.latest_roll_deg = float(roll)
            self.latest_roll_rate_dps = float(msg.angular_velocity.x)  # x-axis after rotation
            self.latest_imu_time = t

    def _on_motor_state(self, msg: JointState):
        try:
            idx = msg.name.index('steer_motor')
            self.latest_steer_deg = math.degrees(float(msg.position[idx]))
        except (ValueError, IndexError):
            pass

    def read_current(self) -> tuple[float, float, float]:
        """Return (roll_deg, roll_rate_dps, steer_deg)."""
        with self._lock:
            return (self.latest_roll_deg, self.latest_roll_rate_dps, self.latest_steer_deg)

    # ── Step 1: IMU zero check ──────────────────────────────────────────
    def step_imu_zero(self) -> bool:
        print("\n" + "=" * 55)
        print("STEP 1: IMU ZERO CHECK")
        print("=" * 55)
        print("  1. Put bike on a level surface (use a spirit level).")
        print("  2. Hold it perfectly vertical (both wheels touching ground).")
        print("  3. Make sure the front wheel is STRAIGHT.")
        print()
        print("  Watch the live roll angle below. It should be ~0°.")
        print("  If it's off by more than 1°, the IMU calibration")
        print("  (sensor_to_bike_rpy_deg) needs correction.")
        print()
        print("  Live readings (updates every 0.5s):")
        print("    Roll    Steer")
        print("    ------  ------")

        for _ in range(12):  # 6 seconds of monitoring
            roll, _, steer = self.read_current()
            bar = "█" * max(0, min(20, int(abs(roll) * 4)))
            side = "← left" if roll < -1 else (">> right" if roll > 1 else "  center")
            print(f"\r    {roll:+6.2f}°  {steer:+6.2f}°  {bar} {side}   ", end="")
            sys.stdout.flush()
            time.sleep(0.5)

        print("\n")
        ans = input("  Does IMU read ~0° when bike is vertical? [y/n]: ").strip().lower()
        if ans != 'y':
            print("\n  ⚠  IMU calibration needed!")
            print("     Adjust sensor_to_bike_rpy_deg in your launch file.")
            print("     Current roll offset ≈ (whatever you see above).")
            print("     Then re-run this tool.")
            return False
        return True

    # ── Step 2: Steer center check ──────────────────────────────────────
    def step_steer_center(self) -> bool:
        print("\n" + "=" * 55)
        print("STEP 2: STEER CENTER CHECK")
        print("=" * 55)
        print("  1. Align the front wheel visually with the bike frame.")
        print("  2. The steer reading should match steer_center_deg (usually 0°).")
        print()
        print("  Live steer angle:")
        print("    Steer")
        print("    ------")

        for _ in range(8):
            _, _, steer = self.read_current()
            bar = "█" * max(0, min(20, int(abs(steer) * 2)))
            side = "← left" if steer < -1 else ("→ right" if steer > 1 else "center")
            print(f"\r    {steer:+6.2f}°  {bar} {side}   ", end="")
            sys.stdout.flush()
            time.sleep(0.5)

        print("\n")
        ans = input("  Is front wheel physically straight? [y/n]: ").strip().lower()
        if ans != 'y':
            print("\n  Adjust the wheel and try again.")
            return self.step_steer_center()

        _, _, steer_now = self.read_current()
        print(f"\n  Steer reads {steer_now:+.2f}° when wheel is physically straight.")
        if abs(steer_now) > 2.0:
            print(f"  ⚠  Offset > 2°. Set steer_center_deg = {steer_now:+.1f} in your launch file.")
        else:
            print(f"  ✓ Steer center OK (offset = {steer_now:+.2f}°).")

        self.steer_offset = steer_now  # record for later
        return True

    # ── Step 3: Drop test ───────────────────────────────────────────────
    def step_drop_test(self):
        print("\n" + "=" * 55)
        print("STEP 3: DROP TEST")
        print("=" * 55)
        print("  1. Hold the bike upright (IMU roll ≈ 0°).")
        print("  2. Front wheel MUST be aligned straight.")
        print("  3. Keep the bike as still as possible.")
        print("  4. Press ENTER, then release the bike.")
        print("     Do NOT push — just let go.")
        print()

        input("  Press ENTER when ready...")

        # Retract kickstands so they don't touch ground during fall
        print("  Retracting kickstands half-up...")
        self._set_kickstands(1.0)  # full 45° retraction, completely clear ground
        time.sleep(0.5)

        # Countdown
        for i in [3, 2, 1]:
            print(f"\r  Recording in {i}...", end="")
            sys.stdout.flush()
            time.sleep(1)
        print("\r  RECORDING — RELEASE THE BIKE NOW!            ")

        # Record 3 seconds at ~50Hz
        self.recording = []
        start = time.time()
        while time.time() - start < 3.0:
            roll, roll_rate, _ = self.read_current()
            self.recording.append((time.time() - start, roll, roll_rate))
            time.sleep(0.01)  # ~100Hz sampling

        # Deploy kickstands back down
        self._set_kickstands(0.0)
        print(f"  Recorded {len(self.recording)} samples in 3s.")

        if len(self.recording) < 20:
            print("  ⚠  Not enough IMU data. Check IMU connection.")
            return None

        # Parse
        t_arr = np.array([r[0] for r in self.recording])
        roll_arr = np.array([r[1] for r in self.recording])
        rate_arr = np.array([r[2] for r in self.recording])

        # Detect release: find where |roll_rate| exceeds 3 deg/s
        release_idx = 0
        for i in range(1, len(t_arr)):
            if abs(rate_arr[i]) > 3.0 and abs(rate_arr[i-1]) < 3.0:
                release_idx = i
                break

        if release_idx == 0:
            # Fallback: use max rate change
            drate = np.diff(rate_arr)
            release_idx = int(np.argmax(np.abs(drate))) + 1
            print(f"  (auto-detected release at t={t_arr[release_idx]:.2f}s)")

        release_t = t_arr[release_idx]
        print(f"  Release detected at t = {release_t:.2f}s")

        # Fit roll_rate vs time in the first 0.3s after release
        fit_end_t = release_t + 0.3
        fit_mask = (t_arr >= release_t) & (t_arr <= fit_end_t)
        fit_t = t_arr[fit_mask]
        fit_rate = rate_arr[fit_mask]

        if len(fit_t) < 5:
            print("  ⚠  Not enough data after release. Trying longer window...")
            fit_end_t = release_t + 0.5
            fit_mask = (t_arr >= release_t) & (t_arr <= fit_end_t)
            fit_t = t_arr[fit_mask]
            fit_rate = rate_arr[fit_mask]

        if len(fit_t) < 5:
            print("  ⚠  Cannot compute bias — too few samples.")
            return None

        # Linear fit: roll_rate = a + slope * (t - release_t)
        fit_t_rel = fit_t - release_t
        slope, intercept = np.polyfit(fit_t_rel, fit_rate, 1)
        roll_acc_dps2 = slope  # deg/s²

        bias_deg = roll_acc_to_bias_deg(roll_acc_dps2)
        direction = "RIGHT" if bias_deg > 0 else "LEFT"

        # Also compute from quadratic fit on roll for cross-check
        fit_roll = roll_arr[fit_mask]
        try:
            quad = np.polyfit(fit_t_rel, fit_roll, 2)
            roll_acc2_dps2 = 2.0 * quad[0]  # 2nd derivative * 0.5 = coeff
            bias2_deg = roll_acc_to_bias_deg(roll_acc2_dps2)
        except Exception:
            bias2_deg = None

        # Print results
        print()
        print("  " + "=" * 49)
        print(f"  Roll acceleration: {roll_acc_dps2:+.1f} deg/s²  ({direction})")
        if bias2_deg is not None:
            print(f"  Roll acc (quad fit):{roll_acc2_dps2:+.1f} deg/s²")
        print(f"  a4 (grav stiffness): {A4:.1f} s⁻²")
        print(f"  R² of linear fit:    {np.corrcoef(fit_t_rel, fit_rate)[0,1]**2:.3f}")
        print("  " + "-" * 49)
        print(f"  >>> BIAS = {abs(bias_deg):.2f}° to the {direction} <<<")
        if bias2_deg is not None:
            print(f"  >>> BIAS (quad fit) = {abs(bias2_deg):.2f}° to the {direction} <<<")
        print("  " + "=" * 49)
        print()
        print("  Use this in sim_tune:")
        print(f"    python sim_tune.py --compare --bias_roll {abs(bias_deg):.1f}")
        print()
        print("  (If the bike fell the other way, the number will be negative.")
        print("   sim_tune uses positive = right-side heavy.)")

        # Plot
        try:
            self._plot_drop(t_arr, roll_arr, rate_arr, release_idx, fit_t, fit_t_rel,
                           fit_rate, slope, intercept, bias_deg)
        except Exception as e:
            print(f"  (Plot skipped: {e})")

        return bias_deg

    def _plot_drop(self, t_arr, roll_arr, rate_arr, release_idx,
                   fit_t, fit_t_rel, fit_rate, slope, intercept, bias_deg):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt

        release_t = t_arr[release_idx]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        fig.suptitle(f"Drop Test — Bias = {abs(bias_deg):.2f}°", fontsize=13)

        ax1.plot(t_arr, roll_arr, lw=1.2, color='#1f77b4', label='roll')
        ax1.axvline(release_t, color='red', ls='--', lw=1, label='release')
        ax1.axvspan(release_t, release_t + 0.3, alpha=0.1, color='green', label='fit window')
        ax1.set_ylabel("Roll [deg]")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3)

        ax2.plot(t_arr, rate_arr, lw=1.2, color='#ff7f0e', label='roll rate')
        ax2.axvline(release_t, color='red', ls='--', lw=1, label='release')
        ax2.axvspan(release_t, release_t + 0.3, alpha=0.1, color='green')

        # Fitted line
        fit_line = intercept + slope * fit_t_rel
        ax2.plot(fit_t, fit_line, color='green', lw=2, ls='-',
                 label=f'fit: {slope:.0f} deg/s²')
        ax2.set_ylabel("Roll rate [deg/s]")
        ax2.set_xlabel("Time [s]")
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3)

        fig.tight_layout()
        plt.show()


def main():
    rclpy.init()
    node = BiasCalibrator()

    # Run steps in a background thread so the spinner keeps going
    def run_steps():
        try:
            if not node.step_imu_zero():
                print("\n✗ IMU zero check failed. Fix calibration and re-run.")
                return

            if not node.step_steer_center():
                print("\n✗ Steer center check failed.")
                return

            bias = node.step_drop_test()
            if bias is not None:
                print(f"\n✓ Done! Use --bias_roll {abs(bias):.1f} in sim_tune.py")
            else:
                print("\n✗ Drop test failed. Try again.")
        except KeyboardInterrupt:
            pass
        finally:
            print("\nExiting. Remember to use the bias value in sim_tune!")

    thread = threading.Thread(target=run_steps, daemon=True)
    thread.start()

    try:
        while thread.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
