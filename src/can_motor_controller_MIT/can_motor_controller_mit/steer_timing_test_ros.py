#!/usr/bin/env python3
"""Measure steering arrival time through the ROS motor controller."""

import csv
import math
import statistics
import time
from datetime import datetime
from pathlib import Path
from typing import List, NamedTuple, Optional, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState


# Change one MIT gain at a time between runs.
MIT_KP = 10.0
MIT_KD = 0.2

STEER_CENTER_DEG = 0.0
MAX_DELTA_DEG = 15.0  # First hardware test; raise gradually after validation.
MEASURED_CYCLES = 5
POS_TOLERANCE_DEG = 1.0
SETTLED_SAMPLES = 3
MOVE_TIMEOUT_SEC = 3.0
COMMAND_PERIOD_SEC = 0.02
STATE_TIMEOUT_SEC = 3.0
MOTOR_CONTROLLER_NODE = "/motor_controller_node"
RESULT_DIR = Path.home() / "steer_test_results"


class TracePoint(NamedTuple):
    run_time_s: float
    move_time_s: float
    phase: str
    target_deg: float
    position_deg: float
    error_deg: float
    speed_deg_s: float
    torque_nm: float


class MoveResult(NamedTuple):
    label: str
    reached: bool
    arrival_time_s: Optional[float]
    final_error_deg: float
    peak_torque_nm: float
    samples: int


def wait_for_parameter_services(client, timeout_sec: float) -> bool:
    """Support both current and older rclpy AsyncParameterClient APIs."""
    wait_method = getattr(client, "wait_for_services", None)
    if wait_method is None:
        wait_method = getattr(client, "wait_for_service", None)
    if wait_method is None:
        raise RuntimeError("AsyncParameterClient has no parameter-service wait method")
    return bool(wait_method(timeout_sec=timeout_sec))


def extract_parameter_results(response):
    """Return SetParametersResult entries from Jazzy or older rclpy responses."""
    if response is None:
        return []
    return list(response.results) if hasattr(response, "results") else list(response)


def stable_arrival_time(
    times: Sequence[float],
    positions: Sequence[float],
    target: float,
    tolerance: float,
    required: int,
) -> Optional[float]:
    streak_start = 0
    streak = 0
    for index, position in enumerate(positions):
        if abs(target - position) <= tolerance:
            if streak == 0:
                streak_start = index
            streak += 1
            if streak >= required:
                return times[streak_start]
        else:
            streak = 0
    return None


class SteerTimingNode(Node):
    def __init__(self) -> None:
        super().__init__("steer_timing_test")
        self.command_pub = self.create_publisher(JointState, "/motor/command", 10)
        self.create_subscription(JointState, "/motor/state", self._state_callback, 20)
        self.param_client = AsyncParameterClient(self, MOTOR_CONTROLLER_NODE)

        self.position_deg: Optional[float] = None
        self.speed_deg_s = 0.0
        self.torque_nm = 0.0
        self.state_sequence = 0
        self.trace: List[TracePoint] = []
        self.results: List[MoveResult] = []
        self.run_start = time.monotonic()
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    def _state_callback(self, message: JointState) -> None:
        try:
            index = message.name.index("steer_motor")
            self.position_deg = math.degrees(float(message.position[index]))
            if len(message.velocity) > index:
                self.speed_deg_s = math.degrees(float(message.velocity[index]))
            if len(message.effort) > index:
                self.torque_nm = float(message.effort[index])
            self.state_sequence += 1
        except (ValueError, IndexError, TypeError):
            return

    def wait_for_state(self) -> None:
        deadline = time.monotonic() + STATE_TIMEOUT_SEC
        while rclpy.ok() and self.position_deg is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position_deg is None:
            raise RuntimeError("No steer_motor feedback received on /motor/state")
        print(f"Initial steering angle: {self.position_deg:.2f} deg")

    def configure_motor_gains(self) -> None:
        print(f"Setting MIT gains: KP={MIT_KP:g}, KD={MIT_KD:g}")
        if not wait_for_parameter_services(self.param_client, 5.0):
            raise RuntimeError(
                f"Parameter service for {MOTOR_CONTROLLER_NODE} is unavailable. "
                "Use the modified motor_controller_node.py."
            )
        future = self.param_client.set_parameters(
            [
                Parameter("steer_mit_kp", Parameter.Type.DOUBLE, float(MIT_KP)),
                Parameter("steer_mit_kd", Parameter.Type.DOUBLE, float(MIT_KD)),
                Parameter(
                    "steer_zero_epoch",
                    Parameter.Type.INTEGER,
                    int(time.time_ns() & 0x7FFFFFFF),
                ),
            ]
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        responses = extract_parameter_results(future.result())
        if not responses or not all(response.successful for response in responses):
            reasons = [response.reason for response in responses or []]
            raise RuntimeError(f"Controller rejected gain update: {reasons}")

        # MIT gains are encoded directly into every ID2 control frame.
        deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        print("MIT gains accepted; current steering position recorded as 0 deg.")

    def _publish_target(self, target_deg: float) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["steer_motor"]
        message.position = [math.radians(target_deg)]
        self.command_pub.publish(message)

    def move_and_time(self, target_deg: float, label: str, measured: bool) -> MoveResult:
        times: List[float] = []
        positions: List[float] = []
        peak_torque = 0.0
        last_sequence = self.state_sequence
        move_start = time.monotonic()
        next_command_time = move_start
        print(f"  {label}: target={target_deg:.1f} deg")

        arrival = None
        while rclpy.ok() and time.monotonic() - move_start <= MOVE_TIMEOUT_SEC:
            now = time.monotonic()
            if now >= next_command_time:
                self._publish_target(target_deg)
                next_command_time = now + COMMAND_PERIOD_SEC

            rclpy.spin_once(self, timeout_sec=0.01)
            if self.state_sequence == last_sequence or self.position_deg is None:
                continue
            last_sequence = self.state_sequence

            elapsed = time.monotonic() - move_start
            position = self.position_deg
            error = target_deg - position
            times.append(elapsed)
            positions.append(position)
            peak_torque = max(peak_torque, abs(self.torque_nm))
            self.trace.append(
                TracePoint(
                    time.monotonic() - self.run_start,
                    elapsed,
                    label,
                    target_deg,
                    position,
                    error,
                    self.speed_deg_s,
                    self.torque_nm,
                )
            )
            arrival = stable_arrival_time(
                times,
                positions,
                target_deg,
                POS_TOLERANCE_DEG,
                SETTLED_SAMPLES,
            )
            if arrival is not None:
                break

        final_error = abs(target_deg - positions[-1]) if positions else math.inf
        result = MoveResult(
            label,
            arrival is not None,
            arrival,
            final_error,
            peak_torque,
            len(positions),
        )
        state = f"arrival={arrival:.3f} s" if arrival is not None else "TIMEOUT"
        print(
            f"    {state}, final error={final_error:.2f} deg, "
            f"peak torque={peak_torque:.2f} Nm, samples={len(positions)}"
        )
        if measured:
            self.results.append(result)
        return result

    def require_position(self, target_deg: float, label: str) -> None:
        if not self.move_and_time(target_deg, label, False).reached:
            raise RuntimeError(f"Steering did not settle at {target_deg:.1f} deg")

    @staticmethod
    def print_summary(name: str, results: Sequence[MoveResult]) -> None:
        values = [r.arrival_time_s for r in results if r.arrival_time_s is not None]
        print(f"\n{name}:")
        if not values:
            print("  No valid moves")
            return
        print(f"  Valid moves: {len(values)}/{len(results)}")
        print(f"  Median arrival time: {statistics.median(values):.3f} s")
        print(f"  Range: {min(values):.3f} to {max(values):.3f} s")

    def run_test(self) -> None:
        self.wait_for_state()
        self.configure_motor_gains()
        high = STEER_CENTER_DEG + MAX_DELTA_DEG
        low = STEER_CENTER_DEG - MAX_DELTA_DEG

        print("Centering and warming up (excluded from statistics)...")
        self.require_position(STEER_CENTER_DEG, "Center")
        self.require_position(high, "Warm-up positive")
        self.require_position(low, "Warm-up negative")

        positive: List[MoveResult] = []
        negative: List[MoveResult] = []
        print("\nStarting measured moves...")
        for cycle in range(1, MEASURED_CYCLES + 1):
            positive.append(self.move_and_time(high, f"Cycle {cycle} positive", True))
            negative.append(self.move_and_time(low, f"Cycle {cycle} negative", True))

        print("\n" + "=" * 60)
        self.print_summary("POSITIVE DIRECTION", positive)
        self.print_summary("NEGATIVE DIRECTION", negative)
        self.print_summary("OVERALL", positive + negative)
        print("=" * 60)

    def save_results(self) -> None:
        if not self.trace:
            return
        RESULT_DIR.mkdir(parents=True, exist_ok=True)
        stem = f"steer_MIT_KP_{MIT_KP:g}_KD_{MIT_KD:g}_{self.timestamp}"
        csv_path = RESULT_DIR / f"{stem}.csv"
        png_path = RESULT_DIR / f"{stem}.png"

        with csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(TracePoint._fields)
            writer.writerows(self.trace)

        times = [point.run_time_s for point in self.trace]
        figure, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
        axes[0].step(times, [p.target_deg for p in self.trace], where="post", label="Target")
        axes[0].plot(times, [p.position_deg for p in self.trace], label="Actual")
        axes[0].set_ylabel("Angle (deg)")
        axes[0].legend()
        axes[1].plot(times, [p.error_deg for p in self.trace], color="tab:red")
        axes[1].axhline(POS_TOLERANCE_DEG, color="gray", linestyle="--")
        axes[1].axhline(-POS_TOLERANCE_DEG, color="gray", linestyle="--")
        axes[1].set_ylabel("Error (deg)")
        axes[2].plot(times, [p.speed_deg_s for p in self.trace], color="tab:green")
        axes[2].set_ylabel("Speed (deg/s)")
        axes[3].plot(times, [p.torque_nm for p in self.trace], color="tab:orange")
        axes[3].set_ylabel("MIT torque (Nm)")
        axes[3].set_xlabel("Run time (s)")
        for axis in axes:
            axis.grid(True)
        figure.suptitle(f"MIT steering timing: KP={MIT_KP:g}, KD={MIT_KD:g}")
        figure.tight_layout()
        figure.savefig(png_path, dpi=160)
        plt.close(figure)
        print(f"CSV saved: {csv_path}")
        print(f"Plot saved: {png_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SteerTimingNode()
    try:
        node.run_test()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as error:
        print(f"Error: {error}")
    finally:
        node.save_results()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
