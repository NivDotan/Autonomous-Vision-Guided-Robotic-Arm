from __future__ import annotations

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent.parent
ASSETS_DIR = PACKAGE_DIR / "assets"


PORT = "COM4"
CAMERA_INDEX = 1     # 0 = first camera, 1 = second camera, etc.
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
WRIST_CARRY_POS = 2048       # Wrist position after catching — rotate before going home
GRIP_LOAD_THRESHOLD = 100

SPEED_LIMIT = 25
SHOULDER_DIR = -1
ELBOW_DIR = -1
PALM_DIR = -1

SH_MIN, SH_MAX = 1000, 3000
EL_MIN, EL_MAX = 400, 3000
PALM_MIN, PALM_MAX = 1000, 3500

DEADBAND_X = 0.10
DEADBAND_Y = 0.10

APPROACH_THRESHOLD = 95000
SHOULDER_COMPENSATION_RATIO = 0.4
K_BASE = 140
K_SHOULDER = 450
K_ELBOW = 65
ELBOW_CENTERING_GATE = 0.3
CENTERED_X = 0.12
CENTERED_Y = 0.12
AIM_X = 1.8
AIM_Y = 1.5

# V2 — bottom-center cell of 3x3 grid (cell 8): X=center, Y=5/6 down
AIM_CELL_X = 0.80      # 0.5 = horizontal center
AIM_CELL_Y = 5.0/6.0   # 5/6 = center of bottom row

# Approach target: 4x4 grid cells 6+10 region (second col, middle two rows)
APPROACH_AIM_X = 3.2 / 4#2.2 / 4
APPROACH_AIM_Y = 2 / 4#1.2 / 4

# Grip retry — fallback sequence of (aim_x, aim_y) tried after each miss
RETRY_AIM_SEQUENCE = [
    (2.5/4, 2.7/4),   # attempt 1 — primary
    (2.5/4, 1.8/4),   # attempt 2 — higher
    (2.5/4, 3.2/4),   # attempt 3 — lower
    (1.8/4, 2.7/4),   # attempt 4 — left
    (3.2/4, 2.7/4),   # attempt 5 — right
]
GRIP_CHECK_FRAMES    = 25  # frames after close before declaring miss (~1 sec at 25fps)
GRIP_LOAD_MIN_FRAMES = 10  # min frames before checking load (avoids motor-torque false positive)
RETREAT_TOLERANCE    = 30  # ticks — close enough to pre-approach position
GRIP_UP_SHOULDER_DELTA = -60   # Shoulder ticks per "up" retry (negative = lifts arm)
GRIP_CLOSER_ELBOW_DELTA = -40  # Elbow ticks for "closer" retry (negative = extends)
GRIP_UP_MAX_TRIES = 3          # Up attempts before switching to "closer"

SAM2_CHECKPOINT = r"E:/sam2.1_hiera_tiny.pt"
SAM2_MODEL_CFG = "configs/sam2.1/sam2.1_hiera_t.yaml"
SEG_EVERY_N_FRAMES = 2

RFDETR_MODEL_SIZE = "nano"
RFDETR_CONFIDENCE = 0.45
DEFAULT_TARGET_CLASS = "cup"
AUTO_APPROACH_AFTER_RFDETR = False

SIM_INSTANT_WHEN_JOG = True
#SIM_CALIBRATION_PATH = ASSETS_DIR / "joint_sim_calibration.json"
#HOME_POSITION_PATH = ASSETS_DIR / "StartHelloPos.json"
SIM_CALIBRATION_PATH = PROJECT_ROOT / "joint_sim_calibration.json"
HOME_POSITION_PATH = PROJECT_ROOT / "StartHelloPos.json"
HOME_POSITION_PATH = PROJECT_ROOT / "StartHelloPos_handoff.json"  # changed this


CAM_BLOCK_MEAN_MAX = 25
CAM_BLOCK_VAR_MAX = 40
HOME_TOL = 25

# ── Motor daemon (Tier 1) ─────────────────────────────────────────────────────
USE_MOTOR_DAEMON = True            # Set True to route commands through C++ daemon
DAEMON_ENDPOINT  = "tcp://localhost:5555"

# ── RealSense depth camera (Tier 1) ──────────────────────────────────────────
MOCK_REALSENSE   = False            # Set True to use MockRealSenseDepth for testing
REALSENSE_ENABLED = True            # Set False to skip RealSense init entirely
HAND_EYE_CALIB_PATH: str | None = None  # Path to camera→base calibration JSON

# ── VL53L1X distance sensor via ESP32 ────────────────────────────────────────
VL53_ENABLED          = True   # Set False to skip sensor init
VL53_PORT             = "COM3" # ESP32
VL53_BAUD             = 115200
VL53_LOCK_DIST_MM        = 130  # Below this: freeze base/shoulder/elbow, only palm moves
VL53_GRIP_DIST_MM        = 110   # Trigger pre-grasp when avg of last 3 readings ≤ this (mm)
VL53_STABLE_WINDOW_MM    = 15   # Max spread across last 3 readings to count as "stable"
VL53_MAX_JUMP_MM         = 30   # Max change between consecutive readings (rejects noise/occlusion)
VL53_PREGRASP_PALM_DELTA = -50  # Ticks to move palm before closing gripper
VL53_MAX_APPROACH_MM     = 400  # Distance at which elbow drive is at full power (err_area=1.0)
VL53_SHOULDER_RATIO      = 0.3  # How much shoulder moves relative to elbow in locked mode (0=off, 1=equal)

# ── Dashboard ────────────────────────────────────────────────────────────────
DASHBOARD_ENABLED = False           # Set True to start FastAPI state broadcaster
DASHBOARD_PORT    = 8000

