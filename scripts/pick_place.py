#!/usr/bin/env python3

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node, robot_shutdown, robot_startup
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import math, numpy as np, time
from geometry_msgs.msg import TransformStamped
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
import tf_transformations as tft

# ------------------- CONFIG -------------------
ROBOT_MODEL = 'wx250s'
ROBOT_NAME  = ROBOT_MODEL
REF_FRAME   = 'camera_color_optical_frame'
ARM_TAG_FRAME  = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'

# Camera-calibration waypoint: puts the tag under/in front of camera
SCAN_DISTANCE_M = 0.3      # forward from camera to park the tag
SCAN_Z_BIAS_M   = -0.02      # small up/down tweak in base frame

# Pick/place geometry
APPROACH_CLEARANCE_M = 0.10
PICK_CLEARANCE_M     = 0.07
PICK_TOUCHDOWN_M     = 0.00

# Simple, non-reactive tool pitch
PITCH_DEFAULT = 0.5         # radians

# Where to drop (right side of the robot: y < 0)
PLACE_POSE = (0.4, -0.3, 0.2)

# Safety: never pick anything close to the arm’s tag (avoids "picking itself")
EXCLUDE_ARMTAG_RADIUS_M = 0.15

# Polling cadence when nothing is visible
POLL_PERIOD_S = 0.25

# ------------------------------------------------

def quat_to_rotm(q):
    return tft.quaternion_matrix([q[0], q[1], q[2], q[3]])[:3, :3]

def camera_facing_waypoint(tf_buffer: Buffer, distance_m: float, z_bias_m: float):
    try:
        tf: TransformStamped = tf_buffer.lookup_transform(
            ARM_BASE_FRAME, REF_FRAME, Time(), timeout=Duration(seconds=2.0)
        )
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        print(f"[WARN] TF lookup {ARM_BASE_FRAME} <- {REF_FRAME} failed: {e}")
        return None, None
    tx, ty, tz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
    qx, qy, qz, qw = tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w
    R = quat_to_rotm((qx, qy, qz, qw))

    # Camera optical frame looks along +Z; move forward from it in base frame
    cam_forward_in_base = R @ np.array([0.0, 0.0, 1.0])
    p_base = np.array([tx, ty, tz]) + distance_m * cam_forward_in_base
    p_base[2] += z_bias_m
    return tuple(p_base.tolist()), cam_forward_in_base

def rpy_from_matrix(R):
    M = np.eye(4); M[:3, :3] = R
    roll, pitch, yaw = tft.euler_from_matrix(M, axes='sxyz')
    return float(roll), float(pitch), float(yaw)

def tool_rpy_facing_camera(forward_vec_base, up_hint=np.array([0.0, 0.0, 1.0])):
    f = np.asarray(forward_vec_base, dtype=float)
    if np.linalg.norm(f) < 1e-6: f = np.array([1.0, 0.0, 0.0])
    f = f / np.linalg.norm(f)
    z_tool = -f
    up = np.asarray(up_hint, dtype=float)
    if abs(np.dot(up, z_tool)) > 0.95: up = np.array([0.0, 1.0, 0.0])
    x_tool = np.cross(up, z_tool)
    if np.linalg.norm(x_tool) < 1e-6: x_tool = np.array([1.0, 0.0, 0.0])
    x_tool = x_tool / np.linalg.norm(x_tool)
    y_tool = np.cross(z_tool, x_tool)
    R = np.column_stack([x_tool, y_tool, z_tool])
    return rpy_from_matrix(R)

def try_find_armtag(armtag: InterbotixArmTagInterface) -> bool:
    try:
        res = armtag.find_ref_to_arm_base_transform()
        return True if (res is None or res is True) else bool(res)
    except Exception as e:
        print(f"[INFO] ARTag not found at this pose: {e}")
        return False

def armtag_position_in_base(tf_buffer: Buffer):
    try:
        tf: TransformStamped = tf_buffer.lookup_transform(
            ARM_BASE_FRAME, ARM_TAG_FRAME, Time(), timeout=Duration(seconds=0.5)
        )
        t = tf.transform.translation
        return (t.x, t.y, t.z)
    except Exception:
        return None

def get_clusters(pcl: InterbotixPointCloudInterface):
    success, clusters = pcl.get_cluster_positions(ref_frame=ARM_BASE_FRAME, sort_axis='x', reverse=True)
    return clusters if success and clusters else []

def filter_clusters_away_from_armtag(clusters, tf_buffer: Buffer, radius_m: float):
    tag_pos = armtag_position_in_base(tf_buffer)
    if tag_pos is None:  # if we can't localize the tag, be conservative and return as-is
        return [(c['position'][0], c['position'][1], c['position'][2]) for c in clusters]
    tx, ty, tz = tag_pos
    kept = []
    r2 = radius_m * radius_m
    for c in clusters:
        x, y, z = c['position']
        if (x - tx)**2 + (y - ty)**2 + (z - tz)**2 > r2:
            kept.append((x, y, z))
    return kept

def main():
    global_node = create_interbotix_global_node()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, global_node)

    bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
    pcl = InterbotixPointCloudInterface(node_inf=global_node)
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME, arm_tag_frame=ARM_TAG_FRAME, arm_base_frame=ARM_BASE_FRAME, node_inf=global_node
    )

    robot_startup(global_node)

    try:
        # --------- 1) Safe start posture
        bot.arm.go_to_sleep_pose()
        bot.gripper.release()

        # --------- 2) Calibration: park the tag under/in front of the camera
        waypoint, forward_vec = camera_facing_waypoint(tf_buffer, SCAN_DISTANCE_M, SCAN_Z_BIAS_M)
        if waypoint is None:
            print("[WARN] Using fallback calibration waypoint (TF not ready).")
            waypoint = (0.30, -0.20, 0.20); forward_vec = np.array([1.0, 0.0, 0.0])

        roll, pitch, yaw = tool_rpy_facing_camera(forward_vec, up_hint=np.array([0.0, 0.0, 1.0]))
        approach = (waypoint[0], waypoint[1], waypoint[2] + APPROACH_CLEARANCE_M)
        bot.arm.set_ee_pose_components(x=approach[0], y=approach[1], z=approach[2], roll=roll, pitch=pitch, yaw=yaw)
        bot.arm.set_ee_pose_components(x=waypoint[0], y=waypoint[1], z=waypoint[2], roll=roll, pitch=pitch, yaw=yaw)

        # Trigger/find the ARTag transform at this calibration pose
        _ = try_find_armtag(armtag)

        # Park in a neutral ready position (keeps tool clear of view)
        bot.arm.set_ee_pose_components(x=0.30, z=0.20, pitch=PITCH_DEFAULT)

        print("[INFO] Simple pick-place loop (non-reactive). CTRL+C to exit.")

        while True:
            # Poll until something appears
            clusters = get_clusters(pcl)
            targets = filter_clusters_away_from_armtag(clusters, tf_buffer, EXCLUDE_ARMTAG_RADIUS_M)

            if not targets:
                time.sleep(POLL_PERIOD_S)
                continue

            # Pick the nearest-in-x target from the filtered list (already sorted by x desc)
            tx, ty, tz = targets[0]
            print(f"[INFO] Target: x={tx:.3f}, y={ty:.3f}, z={tz:.3f}")

            # Straight pick (no mid-course retargeting)
            bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz + PICK_CLEARANCE_M, pitch=PITCH_DEFAULT)
            bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz + PICK_TOUCHDOWN_M, pitch=PITCH_DEFAULT)
            bot.gripper.grasp()
            bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz + PICK_CLEARANCE_M, pitch=PITCH_DEFAULT)

            # Place on the right side
            bot.arm.set_ee_pose_components(x=PLACE_POSE[0], y=PLACE_POSE[1], z=PLACE_POSE[2], pitch=PITCH_DEFAULT)
            bot.gripper.release()

            # Return to ready
            bot.arm.set_ee_pose_components(x=0.30, z=0.20, pitch=PITCH_DEFAULT)

            # Brief pause to let point cloud refresh
            time.sleep(POLL_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping loop.")
    finally:
        bot.arm.set_ee_pose_components(x=0.30, z=0.20, pitch=PITCH_DEFAULT)
        bot.arm.go_to_sleep_pose()
        robot_shutdown(global_node)

if __name__ == '__main__':
    main()

