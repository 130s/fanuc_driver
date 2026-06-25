#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0
"""Watch the controllers for STMO-inactive aborts and drive the recovery service.

When the scaled joint-trajectory controller aborts with

    Aborted: STMO is inactive (!motion_possible)

the known workaround is to switch the GPIO control state back on:

    ros2 service call /fanuc_gpio_controller/switch_control_state \\
        fanuc_msgs/srv/SwitchControlState "status: 1"

This node detects that abort on ``/rosout``, issues the workaround **once** per
episode, then confirms that ``motion_possible`` turns back ``true`` by watching
``/fanuc_gpio_controller/robot_status``. If the workaround does not restore
motion within ``FANUC_BENCHMARK_STMO_TIMEOUT`` seconds the whole benchmark is
aborted: the node exits with a non-zero code and the launch file's
``on_exit="shutdown"`` tears everything else down. The number of successful
recoveries is reported when the node stops.
"""
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.msg import Log
    from fanuc_msgs.msg import RobotStatus
except ModuleNotFoundError:  # pragma: no cover - exercised in minimal environments
    rclpy = None
    Node = object
    Log = object
    RobotStatus = object

LOG_PATH = Path(os.environ.get("FANUC_BENCHMARK_STMO_LOG", "/tmp/fanuc_benchmark_stmo_recovery.log"))

# How long to wait for motion_possible to turn true after issuing the workaround.
RECOVERY_TIMEOUT = float(os.environ.get("FANUC_BENCHMARK_STMO_TIMEOUT", "10.0"))
# How often to re-check motion_possible while waiting.
POLL_INTERVAL = float(os.environ.get("FANUC_BENCHMARK_STMO_POLL", "0.25"))
# Ignore further STMO warnings for this long after an attempt, so a burst of
# identical warnings only triggers a single workaround call.
COOLDOWN = float(os.environ.get("FANUC_BENCHMARK_STMO_COOLDOWN", "3.0"))

ROBOT_STATUS_TOPIC = "/fanuc_gpio_controller/robot_status"
SERVICE_CMD = [
    "ros2",
    "service",
    "call",
    "/fanuc_gpio_controller/switch_control_state",
    "fanuc_msgs/srv/SwitchControlState",
    "status: 1",
]

RE_PATTERN = re.compile(r"STMO is inactive|!motion_possible", re.IGNORECASE)


class StmoRecoveryNode(Node):
    def __init__(
        self,
        recovery_timeout: float = RECOVERY_TIMEOUT,
        poll_interval: float = POLL_INTERVAL,
        cooldown: float = COOLDOWN,
    ) -> None:
        super().__init__("fanuc_benchmark_stmo_recovery")
        self._recovery_timeout = recovery_timeout
        self._poll_interval = poll_interval
        self._cooldown = cooldown

        self._count = 0
        self._motion_possible = None  # tri-state: None=unknown, True, False
        self._abort = False
        self._lock = threading.Lock()
        self._recovering = False
        self._last_attempt = 0.0

        self._subscription = self.create_subscription(
            Log,
            "/rosout",
            self._rosout_callback,
            10,
        )
        self._status_subscription = self.create_subscription(
            RobotStatus,
            ROBOT_STATUS_TOPIC,
            self._status_callback,
            10,
        )
        self.get_logger().info(
            "STMO recovery watcher active "
            f"(timeout={recovery_timeout}s, cooldown={cooldown}s)."
        )

    @property
    def recovery_count(self) -> int:
        return self._count

    @property
    def aborted(self) -> bool:
        return self._abort

    def _status_callback(self, msg: "RobotStatus") -> None:
        self._motion_possible = bool(msg.motion_possible)

    def _rosout_callback(self, msg: Log) -> None:
        if not should_trigger_recovery(msg.msg):
            return
        self._maybe_start_recovery()

    def _maybe_start_recovery(self) -> None:
        """Start a single recovery attempt, debounced against warning bursts."""
        with self._lock:
            if self._recovering:
                return
            # The tail of a warning burst can arrive after motion is already
            # restored; don't issue a redundant workaround in that case.
            if self._motion_possible is True:
                return
            now = time.monotonic()
            if now - self._last_attempt < self._cooldown:
                return
            self._recovering = True
            self._last_attempt = now
        threading.Thread(target=self._run_recovery, daemon=True).start()

    def _run_recovery(self) -> None:
        timestamp = datetime.now(timezone.utc).isoformat()
        self.get_logger().warn(
            f"STMO inactive detected at {timestamp}; issuing recovery workaround once."
        )
        issue_recovery_command()

        if self._wait_for_motion_possible():
            with self._lock:
                self._count += 1
                count = self._count
                self._recovering = False
            append_log_entry(timestamp, count)
            self.get_logger().info(
                f"Recovery succeeded: motion_possible restored (recovery #{count})."
            )
            return

        # Workaround did not restore motion -> abort the whole benchmark.
        with self._lock:
            count = self._count  # successful recoveries before this failure
        self.get_logger().error(
            f"Recovery FAILED: motion_possible did not turn true within "
            f"{self._recovery_timeout:.1f}s after the workaround. Aborting benchmark. "
            f"{format_final_report(count)}"
        )
        self._abort = True
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()

    def _wait_for_motion_possible(self) -> bool:
        deadline = time.monotonic() + self._recovery_timeout
        while time.monotonic() < deadline:
            if self._motion_possible is True:
                return True
            time.sleep(self._poll_interval)
        return self._motion_possible is True


def should_trigger_recovery(message: str) -> bool:
    return bool(RE_PATTERN.search(message))


def format_log_entry(timestamp: str, count: int) -> str:
    return f"{timestamp} recovery_cmd_issued_count={count}\n"


def format_final_report(count: int) -> str:
    return f"STMO recoveries performed during this run: {count}"


def append_log_entry(timestamp: str, count: int) -> None:
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open("a", encoding="utf-8") as handle:
        handle.write(format_log_entry(timestamp, count))


def issue_recovery_command() -> None:
    subprocess.run(SERVICE_CMD, check=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def main() -> None:
    if rclpy is None:
        print("rclpy is not available; STMO recovery watcher cannot start.", file=sys.stderr)
        return

    rclpy.init(args=sys.argv)
    node = StmoRecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        count = node.recovery_count
        aborted = node.aborted
        # Report how many recoveries happened over the whole run.
        node.get_logger().info(format_final_report(count))
        print(format_final_report(count))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if aborted:
            sys.exit(1)


if __name__ == "__main__":
    main()
