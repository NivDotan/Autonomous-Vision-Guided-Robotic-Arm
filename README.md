# SAM2 RF-DETR Robot Arm Control

This project is a computer-vision control app for a small Feetech-servo robot
arm. It combines hand tracking, object segmentation, object detection, and a
PyBullet simulator so the arm can be controlled from a webcam feed.

The app was refactored from a single large experiment script into a cleaner
Python package. The goal is to make the code easier to read, demo, maintain,
and upload as a standalone GitHub repository.

## What It Does

- Uses **MediaPipe Hands** for hand-based teleoperation.
- Uses **SAM2** to segment an object selected by mouse click.
- Optionally uses **RF-DETR** to find an object automatically by class name,
  for example `cup`, then passes its bounding box to SAM2.
- Uses OpenCV CSRT tracking after SAM2 creates the initial object box.
- Moves the robot arm toward the tracked object in object mode.
- Closes the gripper on command.
- Reads gripper load from the Feetech motor bus and returns home after a grab.
- Mirrors the arm in a **PyBullet simulator** with debug sliders for sim jog.

## Typical Flow

1. Start the app.
2. Press `M` to switch from hand mode to object mode.
3. Select an object:
   - click it in the camera window, or
   - press `U` to search for the default target `cup`, or
   - press `T` and type a target class.
4. Press `S` to enable motion updates.
5. Press `A` to approach the tracked object.
6. Press `Space` to close/open the gripper.

## Controls

| Key | Action |
| --- | --- |
| `S` | Enable/disable motor writes and internal motion updates |
| `M` | Switch between hand tracking and object tracking |
| `A` | Toggle approach mode for the tracked object |
| `Space` | Close/open the gripper |
| `U` | Ask RF-DETR to find the default target, currently `cup` |
| `T` | Type a target class for RF-DETR |
| `J` | Toggle PyBullet sim jog mode |
| `R` | Recreate/sync PyBullet sliders |
| `Z` / `X` | Manual palm adjustment |
| `C` | Return palm control to automatic mode |
| `Q` | Quit |

## Project Structure

```text
robot_sam2_app/
  README.md
  requirements.txt
  .gitignore
  robot_sam2_app/
    app.py                  main camera, UI, and orchestration loop
    config.py               ports, model paths, gains, limits, constants
    control.py              hand/object vision-to-joint target logic
    hardware.py             Feetech / LeRobot hardware wrapper
    simulation.py           PyBullet simulator wrapper
    state.py                RobotState dataclass
    tracking.py             SAM2 initialization + OpenCV CSRT tracking
    utils.py                small shared helpers
    vision/
      sam2_segmenter.py     SAM2 point/box prompted segmentation
      rfdetr_selector.py    RF-DETR target selection
    assets/
      joint_sim_calibration.json
      so101_simple_sim.urdf
      StartHelloPos.json
```

## Requirements

Install the normal dependencies:

```powershell
python -m pip install -r requirements.txt
```

Install SAM2 from the official repository:

```powershell
git clone https://github.com/facebookresearch/sam2.git
cd sam2
python -m pip install -e .
```

RF-DETR is optional. Install it only if you want the `U` and `T` autonomous
target selection controls:

```powershell
python -m pip install rfdetr
```

## Model Paths

SAM2 settings are in:

```text
robot_sam2_app/config.py
```

Current default:

```python
SAM2_CHECKPOINT = r"E:/sam2.1_hiera_tiny.pt"
SAM2_MODEL_CFG = "configs/sam2.1/sam2.1_hiera_t.yaml"
```

Download the matching SAM2 checkpoint and update the path if your file is in a
different location.

## Run

From this repository folder:

```powershell
python -m robot_sam2_app.main
```

The app opens:

- an OpenCV camera/control window named `Robot Brain`
- a PyBullet simulator window if PyBullet can load successfully

The real robot can remain disconnected. The app will still run camera, SAM2,
object tracking, and simulator logic. Motor writes only happen when hardware is
connected and `S` has enabled motors.

## Simulator

The simulator files are included in `robot_sam2_app/assets`:

- `so101_simple_sim.urdf`
- `joint_sim_calibration.json`
- `StartHelloPos.json`

The simulator is not meant to be a perfect CAD model. It is a readable visual
mirror with matching joint names, tick offsets, and PyBullet debug sliders.

## Hardware Notes

The hardware layer expects a Feetech/LeRobot motor bus on:

```python
PORT = "COM4"
```

Change this in `config.py` if your robot uses a different port.

The motor map is:

```text
1 base
2 shoulder
3 elbow
4 palm
5 wrist
6 gripper
```

## Status

This is an experimental robotics control app. Test first with motors disabled
or with the simulator before enabling hardware movement.
