from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _require_pybullet():
    try:
        import pybullet as p  # noqa: N812
    except ImportError as exc:
        raise ImportError("Install PyBullet with: python -m pip install pybullet") from exc
    return p


@dataclass
class JointCalibration:
    logical: str
    urdf_joint_name: str
    tick_offset: float
    sign: float
    tick_min: int
    tick_max: int
    ticks_per_rev: int = 4096
    use_slider: bool = True


class PyBulletArmSim:
    """PyBullet mirror of the real arm plus debug sliders for sim jog."""

    def __init__(self, calibration_path: Path):
        self.calibration_path = Path(calibration_path).resolve()
        self.joints: list[JointCalibration] = []
        self.urdf_path = Path()
        self._load_calibration()

        self._p = None
        self._client: int | None = None
        self.robot_id: int | None = None
        self.joint_indices: list[int] = []
        self.slider_ids: list[int] = []

    def _load_calibration(self) -> None:
        data: dict[str, Any] = json.loads(self.calibration_path.read_text(encoding="utf-8"))
        urdf = (data.get("urdf_path") or os.environ.get("ROBOT_URDF") or "").strip()
        if not urdf:
            raise ValueError("Missing urdf_path in simulation calibration.")

        urdf_path = Path(urdf).expanduser()
        if not urdf_path.is_absolute():
            urdf_path = self.calibration_path.parent / urdf_path
        self.urdf_path = urdf_path.resolve()
        if not self.urdf_path.is_file():
            raise FileNotFoundError(f"URDF not found: {self.urdf_path}")

        self.joints = [JointCalibration(**item) for item in data["joints"]]

    @staticmethod
    def tick_to_rad(cfg: JointCalibration, tick: float) -> float:
        return cfg.sign * (tick - cfg.tick_offset) * (2 * math.pi / cfg.ticks_per_rev)

    @staticmethod
    def rad_to_tick(cfg: JointCalibration, rad: float) -> int:
        tick = rad / cfg.sign / (2 * math.pi / cfg.ticks_per_rev) + cfg.tick_offset
        return int(round(tick))

    def connect(self) -> None:
        p = _require_pybullet()
        self._p = p
        self._client = p.connect(p.GUI)
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)
        p.setAdditionalSearchPath(str(self.urdf_path.parent))

        flags = getattr(p, "URDF_USE_INERTIA_FROM_FILE", 0)
        self.robot_id = p.loadURDF(str(self.urdf_path), basePosition=[0, 0, 0], useFixedBase=True, flags=flags)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.35,
            cameraYaw=45,
            cameraPitch=-18,
            cameraTargetPosition=[0.22, 0, 0.58],
        )

        name_to_index = {}
        for index in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, index)
            if info[2] == p.JOINT_REVOLUTE:
                name_to_index[info[1].decode("utf-8")] = index

        missing = [cfg.urdf_joint_name for cfg in self.joints if cfg.urdf_joint_name not in name_to_index]
        if missing:
            raise KeyError(f"URDF joints not found: {missing}. Available: {sorted(name_to_index)}")
        self.joint_indices = [name_to_index[cfg.urdf_joint_name] for cfg in self.joints]

    def disconnect(self) -> None:
        if self._p is not None and self._client is not None:
            self.clear_sliders()
            self._p.disconnect(self._client)
        self._client = None
        self.robot_id = None

    def step_gui(self) -> None:
        if self._p is not None and self._client is not None:
            self._p.stepSimulation()

    def clear_sliders(self) -> None:
        if self._p is None:
            return
        for slider_id in self.slider_ids:
            try:
                self._p.removeUserDebugItem(slider_id)
            except Exception:
                pass
        self.slider_ids = []

    def set_visual_from_ticks(self, ticks: dict[str, int]) -> None:
        if self._p is None or self.robot_id is None:
            return
        for cfg, joint_index in zip(self.joints, self.joint_indices):
            tick = float(ticks.get(cfg.logical, cfg.tick_offset))
            self._p.resetJointState(self.robot_id, joint_index, self.tick_to_rad(cfg, tick))

    def recreate_sliders_from_ticks(self, ticks: dict[str, int]) -> None:
        if self._p is None or self.robot_id is None:
            return
        self.clear_sliders()
        for cfg, joint_index in zip(self.joints, self.joint_indices):
            if not cfg.use_slider:
                continue
            info = self._p.getJointInfo(self.robot_id, joint_index)
            lower, upper = float(info[8]), float(info[9])
            if lower >= upper:
                lower, upper = -2 * math.pi, 2 * math.pi
            rad = self.tick_to_rad(cfg, float(ticks.get(cfg.logical, cfg.tick_offset)))
            rad = max(lower, min(upper, rad))
            label = f"{cfg.logical} ({cfg.urdf_joint_name})"
            self.slider_ids.append(self._p.addUserDebugParameter(label, lower, upper, rad))
        self.set_visual_from_ticks(ticks)

    def read_sliders_as_ticks(self) -> dict[str, int]:
        if self._p is None or self.robot_id is None:
            return {}
        slider_cfgs = [cfg for cfg in self.joints if cfg.use_slider]
        if len(slider_cfgs) != len(self.slider_ids):
            return {}
        out = {}
        for slider_id, cfg in zip(self.slider_ids, slider_cfgs):
            rad = float(self._p.readUserDebugParameter(slider_id))
            tick = self.rad_to_tick(cfg, rad)
            out[cfg.logical] = int(max(cfg.tick_min, min(cfg.tick_max, tick)))
        return out

