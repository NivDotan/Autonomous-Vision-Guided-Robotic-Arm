import sys, math
sys.path.insert(0, ".")
from lerobot.common.robot_devices.robots.factory import make_robot
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError
from control.lerobot_adapter import LeRobotAdapter
from kinematics.forward_kinematics import forward_kinematics

robot = make_robot("so101")

try:
    try:
        robot.connect()
    except RobotDeviceAlreadyConnectedError:
        print("Robot was already connected; continuing.")

    adapter = LeRobotAdapter(robot=robot, dry_run=False)
    obs = adapter.observe()

    print("Joint angles ^(deg^):")
    for k, v in obs.items():
        print(f"  {k}: {math.degrees^(v^):.1f}")

    fk = forward_kinematics(
        obs.get("shoulder_pan", 0.0),
        obs.get("shoulder_lift", 0.0),
        obs.get("elbow_flex", 0.0),
        obs.get("wrist_flex", 0.0),
        0.0,
    )

    print(f"End-effector: x={fk['x']:.3f} y={fk['y']:.3f} z={fk['z']:.3f} m")

finally:
    try:
        robot.disconnect()
    except Exception as e:
        print(f"Disconnect skipped/failed: {e}")
