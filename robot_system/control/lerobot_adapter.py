"""
Step 5 — LeRobot adapter with dry-run support.

Wraps SO101Follower so the rest of the pipeline works in radians regardless
of whether the hardware is connected.

Real robot facts (discovered from inspection):
  - Port:          COM4
  - Calibration:   my_awesome_follower_arm
  - Obs keys:      shoulder_pan.pos, shoulder_lift.pos, elbow_flex.pos,
                   wrist_flex.pos, wrist_roll.pos, gripper.pos
  - Units:         DEGREES (use_degrees=True by default)
  - Send action:   robot.send_action({'shoulder_pan.pos': deg, ...})

The adapter converts everything to/from radians internally.

Usage:
    # Dry-run (no hardware)
    adapter = LeRobotAdapter(dry_run=True)

    # Live
    from control.lerobot_adapter import make_live_adapter
    adapter = make_live_adapter()   # connects automatically
"""

from __future__ import annotations

import math
import time
from typing import Any


# Canonical short names used throughout the pipeline (radians)
JOINT_NAMES: tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# Map short name -> LeRobot observation key
_OBS_KEY: dict[str, str] = {j: f"{j}.pos" for j in JOINT_NAMES}

# Map short name -> LeRobot action key (same pattern for SO-101)
_ACT_KEY: dict[str, str] = {j: f"{j}.pos" for j in JOINT_NAMES}


class LeRobotAdapter:
    """
    Thin adapter around SO101Follower.

    All angles in/out of this class are in RADIANS.
    Internally converts to/from degrees for the LeRobot API.

    Parameters
    ----------
    robot:    connected SO101Follower instance (None if dry_run=True)
    dry_run:  print commands instead of sending to hardware
    """

    JOINT_NAMES = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']

    def __init__(self, robot: Any = None, dry_run: bool = True) -> None:
        if not dry_run and robot is None:
            raise ValueError("robot must be provided when dry_run=False")
        self.robot   = robot
        self.dry_run = dry_run
        self._last_joints: dict[str, float] = {j: 0.0 for j in JOINT_NAMES}

    # ------------------------------------------------------------------
    # Observation — returns radians
    # ------------------------------------------------------------------

    def observe(self) -> dict[str, float]:
        """Return current joint positions as {name: radians}."""
        if self.dry_run:
            return dict(self._last_joints)

        obs = self.robot.get_observation()
        result: dict[str, float] = {}
        for name in JOINT_NAMES:
            key = _OBS_KEY[name]
            if key in obs:
                result[name] = math.radians(float(obs[key]))
        return result

    # ------------------------------------------------------------------
    # Movement — accepts radians
    # ------------------------------------------------------------------

    def move_joints(self, joints: dict[str, float], blocking: bool = True) -> None:
        """
        Command joint positions given in radians.

        Only the joints in `joints` are updated; others keep their last value.
        """
        full = dict(self._last_joints)
        full.update(joints)
        self._last_joints = full

        if self.dry_run:
            deg_str = ", ".join(
                f"{k}={math.degrees(v):.1f}deg" for k, v in full.items()
            )
            print(f"[DRY-RUN] move_joints  {deg_str}")
            return

        # Convert radians -> degrees for LeRobot
        action = {_ACT_KEY[name]: math.degrees(val)
                  for name, val in full.items()
                  if name in _ACT_KEY}
        self.robot.send_action(action)
        if blocking:
            time.sleep(0.05)

    def move_to_home(self) -> None:
        """Send all joints to zero (home) position."""
        self.move_joints({j: 0.0 for j in JOINT_NAMES})

    # ------------------------------------------------------------------
    # Gripper helpers
    # ------------------------------------------------------------------

    def open_gripper(self) -> None:
        self.move_joints({"gripper": 0.0})

    def close_gripper(self) -> None:
        self.move_joints({"gripper": math.radians(45)})

    # ------------------------------------------------------------------
    # Context-manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "LeRobotAdapter":
        return self

    def __exit__(self, *_: Any) -> None:
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass


# ------------------------------------------------------------------
# Factory for live use
# ------------------------------------------------------------------

def make_live_adapter(
    port: str = "COM4",
    robot_id: str = "my_awesome_follower_arm",
) -> LeRobotAdapter:
    """
    Connect to the real SO-101 and return a ready-to-use adapter.

    Usage:
        adapter = make_live_adapter()
        print(adapter.observe())
    """
    from lerobot.robots.so101_follower.so101_follower import SO101Follower
    from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

    cfg   = SO101FollowerConfig(port=port, cameras={}, id=robot_id)
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    return LeRobotAdapter(robot=robot, dry_run=False)
