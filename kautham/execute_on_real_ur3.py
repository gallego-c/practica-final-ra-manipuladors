#!/usr/bin/env python3
"""
execute_on_real_ur3.py — Replays a Kautham taskfile on a REAL UR3 + Robotiq 85.

This is "Level 4": it closes the sim-to-real gap. It does NOT re-plan anything.
It reads the joint trajectories that Kautham already computed (RRT-Connect) and
stored as <Conf> waypoints in kautham/taskfile_rubik_ur3.xml, then streams them to
the physical arm through the Universal Robots ROS 2 driver, opening/closing the
Robotiq gripper at the right moments.

Why this works without any [0,1]->radians conversion:
  Each <Conf> in the taskfile is [SE3 base pose (7)] + [2 base/coupling] +
  [6 UR3 joints IN RADIANS] + [gripper mimic joints]. We pull columns 9..14.

Gripper logic (derived purely from the taskfile structure):
  - start OPEN
  - Transit  -> Transfer  : CLOSE  (grab the cube)
  - Transfer -> Transit   : OPEN   (release the cube)

----------------------------------------------------------------------------
PREREQUISITES (run these first, in separate terminals):

  1. UR3 driver (replace with your robot's IP):
       ros2 launch ur_robot_driver ur_control.launch.py \
            ur_type:=ur3 robot_ip:=192.168.1.102 \
            launch_rviz:=false
     On the UR teach pendant, load + run the External Control URCap program.

  2. Robotiq gripper driver (depends on your hardware connection; example):
       ros2 launch robotiq_driver robotiq_control.launch.py

  3. IMPORTANT — calibrate once so the driver kinematics match YOUR arm:
       ros2 launch ur_calibration calibration_correction.launch.py \
            robot_ip:=192.168.1.102 \
            target_filename:="$(pwd)/my_ur3_calibration.yaml"
----------------------------------------------------------------------------

USAGE:
  python3 execute_on_real_ur3.py --taskfile ../kautham/taskfile_rubik_ur3.xml
  python3 execute_on_real_ur3.py --taskfile ... --dry-run        # no robot, just prints
  python3 execute_on_real_ur3.py --taskfile ... --time-per-segment 6.0
"""

import argparse
import sys
import time
import xml.etree.ElementTree as ET

# ── UR3 joints live at columns 9..14 of every <Conf> (verified against home pose
#    [0, -pi/2, 0, -pi/2, 0, 0]). Make this configurable in case your URDF differs.
UR_JOINT_COLS = [9, 10, 11, 12, 13, 14]

# Joint names MUST match the controller's expected order (UR ROS 2 default order).
UR_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Safety: max joint speed used to auto-time waypoints (rad/s). Keep LOW for first runs.
MAX_JOINT_SPEED = 0.4
# Hard sanity bound: refuse any waypoint outside +/- this many radians.
JOINT_LIMIT = 7.0  # ~ +/- 2*pi with margin


# ════════════════════════════════════════════════════════════════════════════
#  TASKFILE PARSING  (no ROS needed for this part — works in --dry-run too)
# ════════════════════════════════════════════════════════════════════════════
def parse_taskfile(path):
    """Returns a list of segments: [{'type': 'Transit'|'Transfer',
                                      'waypoints': [[j1..j6], ...]}, ...]."""
    tree = ET.parse(path)
    root = tree.getroot()
    segments = []
    for child in root:
        if child.tag not in ("Transit", "Transfer"):
            continue
        waypoints = []
        for conf in child.findall("Conf"):
            vals = conf.text.split()
            if len(vals) <= max(UR_JOINT_COLS):
                raise ValueError(
                    f"Conf has only {len(vals)} values; cannot read cols "
                    f"{UR_JOINT_COLS}. Check UR_JOINT_COLS."
                )
            joints = [float(vals[c]) for c in UR_JOINT_COLS]
            for j in joints:
                if abs(j) > JOINT_LIMIT:
                    raise ValueError(f"Joint value {j} exceeds safety limit "
                                     f"+/-{JOINT_LIMIT} rad. Aborting.")
            waypoints.append(joints)
        if waypoints:
            segments.append({"type": child.tag, "waypoints": waypoints})
    return segments


def gripper_events(segments):
    """Returns dict {segment_index: 'close'|'open'} for transitions.
    The event happens BEFORE executing segment i (i.e. at its start pose)."""
    events = {}
    prev = None
    for i, seg in enumerate(segments):
        t = seg["type"]
        if prev == "Transit" and t == "Transfer":
            events[i] = "close"
        elif prev == "Transfer" and t == "Transit":
            events[i] = "open"
        prev = t
    return events


def downsample(waypoints, keep_every=1):
    """Kautham paths are very dense. Optionally thin them, always keeping endpoints."""
    if keep_every <= 1 or len(waypoints) <= 2:
        return waypoints
    kept = waypoints[::keep_every]
    if kept[-1] != waypoints[-1]:
        kept.append(waypoints[-1])
    return kept


def auto_durations(waypoints, max_speed=MAX_JOINT_SPEED, min_dt=0.05):
    """Time-parameterize: dt between waypoints from the largest joint delta."""
    times = [0.0]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        max_delta = max(abs(x - y) for x, y in zip(a, b))
        dt = max(max_delta / max_speed, min_dt)
        times.append(times[-1] + dt)
    return times


# ════════════════════════════════════════════════════════════════════════════
#  ROS 2 EXECUTION
# ════════════════════════════════════════════════════════════════════════════
def run_on_robot(segments, events, time_per_segment=None, keep_every=4):
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from builtins import Exception as _Exc
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    rclpy.init()
    node = Node("ur3_rubik_taskfile_executor")
    traj_client = ActionClient(
        node, FollowJointTrajectory,
        "/scaled_joint_trajectory_controller/follow_joint_trajectory",
    )

    node.get_logger().info("Waiting for the UR3 trajectory action server...")
    if not traj_client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(
            "scaled_joint_trajectory_controller action server not found. "
            "Is ur_robot_driver running and the External Control program playing?")
        rclpy.shutdown()
        sys.exit(1)

    gripper = make_gripper(node)

    def send_segment(waypoints):
        waypoints = downsample(waypoints, keep_every)
        if time_per_segment is not None:
            # spread waypoints evenly across a fixed segment duration
            n = len(waypoints)
            times = [time_per_segment * k / max(n - 1, 1) for k in range(n)]
        else:
            times = auto_durations(waypoints)

        traj = JointTrajectory()
        traj.joint_names = list(UR_JOINT_NAMES)
        for wp, t in zip(waypoints, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in wp]
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        gh = future.result()
        if not gh.accepted:
            raise _Exc("Trajectory goal REJECTED by controller.")
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)
        return res_future.result()

    try:
        for i, seg in enumerate(segments):
            if events.get(i) == "close":
                node.get_logger().info(f"[seg {i}] CLOSE gripper (grab cube)")
                gripper("close"); time.sleep(1.0)
            elif events.get(i) == "open":
                node.get_logger().info(f"[seg {i}] OPEN gripper (release cube)")
                gripper("open"); time.sleep(1.0)

            node.get_logger().info(
                f"[seg {i}] Executing {seg['type']} "
                f"({len(seg['waypoints'])} waypoints)")
            send_segment(seg["waypoints"])
        node.get_logger().info("✓ Full Rubik solve trajectory executed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def make_gripper(node):
    """Returns a callable gripper('open'|'close').

    Robotiq drivers vary. The cleanest modern path is the GripperCommand action
    exposed by robotiq_driver / ur_robotiq setups. Adjust the action name and the
    open/close positions to YOUR gripper (0.0 = open, ~0.8 = closed is typical
    for the Robotiq 2F-85 in meters of finger gap-equivalent).
    """
    from rclpy.action import ActionClient
    from control_msgs.action import GripperCommand
    import rclpy

    client = ActionClient(node, GripperCommand,
                          "/robotiq_gripper_controller/gripper_cmd")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().warn(
            "Gripper action server not found — gripper commands will be SKIPPED. "
            "Wire make_gripper() to your actual Robotiq driver.")
        return lambda _cmd: None

    OPEN_POS, CLOSE_POS, EFFORT = 0.0, 0.8, 40.0

    def cmd(action):
        goal = GripperCommand.Goal()
        goal.command.position = OPEN_POS if action == "open" else CLOSE_POS
        goal.command.max_effort = EFFORT
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut)
        gh = fut.result()
        if gh.accepted:
            rclpy.spin_until_future_complete(node, gh.get_result_async())
    return cmd


# ════════════════════════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Replay Kautham taskfile on a real UR3.")
    ap.add_argument("--taskfile", required=True, help="Path to taskfile_rubik_ur3.xml")
    ap.add_argument("--dry-run", action="store_true",
                    help="Parse + print the plan; do NOT move the robot.")
    ap.add_argument("--time-per-segment", type=float, default=None,
                    help="Fixed seconds per segment (default: auto from joint speed).")
    ap.add_argument("--keep-every", type=int, default=4,
                    help="Downsample dense paths: keep 1 of every N waypoints.")
    args = ap.parse_args()

    segments = parse_taskfile(args.taskfile)
    events = gripper_events(segments)

    print(f"Parsed {len(segments)} segments from {args.taskfile}")
    for i, seg in enumerate(segments):
        ev = events.get(i, "")
        ev = f"  <-- {ev.upper()} GRIPPER" if ev else ""
        print(f"  seg {i:2d}: {seg['type']:8s} "
              f"{len(seg['waypoints']):4d} waypoints{ev}")
        if i == 0:
            print(f"           start joints: "
                  f"{['%.3f' % j for j in seg['waypoints'][0]]}")

    if args.dry_run:
        print("\n[--dry-run] Not moving the robot. Verify the start joints above "
              "match your robot's /joint_states at home before a real run.")
        return

    print("\n⚠  Make sure the workspace is clear and you can reach the e-stop.")
    input("Press ENTER to execute on the REAL robot (Ctrl-C to abort)... ")
    run_on_robot(segments, events,
                 time_per_segment=args.time_per_segment,
                 keep_every=args.keep_every)


if __name__ == "__main__":
    main()
