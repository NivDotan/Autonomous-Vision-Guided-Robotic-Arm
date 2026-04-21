from __future__ import annotations

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
ASSETS_DIR = PACKAGE_DIR / "assets"


PORT = "COM4"
MOTOR_NAMES = ("base", "shoulder", "elbow", "palm", "wrist", "gripper")
MOTOR_IDS = (1, 2, 3, 4, 5, 6)

DEFAULT_TICKS = {
    "base": 2048,
    "shoulder": 2048,
    "elbow": 2048,
    "palm": 2048,
    "wrist": 3200,
    "gripper": 3000,
}

GRIPPER_OPEN = 3000
GRIPPER_CLOSE = 1500
GRIPPER_ROT_90_POS = 3750
GRIP_LOAD_THRESHOLD = 150

SPEED_LIMIT = 25
SHOULDER_DIR = -1
ELBOW_DIR = 1
PALM_DIR = 1

SH_MIN, SH_MAX = 1000, 3000
EL_MIN, EL_MAX = 1000, 3000
PALM_MIN, PALM_MAX = 1000, 3000

DEADBAND_X = 0.10
DEADBAND_Y = 0.10

APPROACH_THRESHOLD = 95000
SHOULDER_COMPENSATION_RATIO = 0.4
K_BASE = 140
K_SHOULDER = 140
K_ELBOW = 22
CENTERED_X = 0.12
CENTERED_Y = 0.12
AIM_X = 1.8
AIM_Y = 1.5

SAM2_CHECKPOINT = r"E:/sam2.1_hiera_tiny.pt"
SAM2_MODEL_CFG = "configs/sam2.1/sam2.1_hiera_t.yaml"
SEG_EVERY_N_FRAMES = 2

RFDETR_MODEL_SIZE = "nano"
RFDETR_CONFIDENCE = 0.45
DEFAULT_TARGET_CLASS = "cup"
AUTO_APPROACH_AFTER_RFDETR = False

SIM_INSTANT_WHEN_JOG = True
SIM_CALIBRATION_PATH = ASSETS_DIR / "joint_sim_calibration.json"
HOME_POSITION_PATH = ASSETS_DIR / "StartHelloPos.json"

CAM_BLOCK_MEAN_MAX = 25
CAM_BLOCK_VAR_MAX = 40
HOME_TOL = 25

