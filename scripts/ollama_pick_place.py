#!/usr/bin/env python3

from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node, robot_shutdown, robot_startup
)
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

import math, numpy as np, time, json, base64, os, sys
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
import tf_transformations as tft

CV_OK = True
try:
    import cv2
    from cv_bridge import CvBridge
except Exception:
    CV_OK = False

# ------------------- CONFIG -------------------
ROBOT_MODEL = 'wx250s'
ROBOT_NAME  = ROBOT_MODEL
REF_FRAME   = 'camera_color_optical_frame'
ARM_TAG_FRAME  = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'

# Camera topics
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
IMAGE_TOPIC       = "/camera/camera/color/image_raw"

# Ready & pick/place geometry
APPROACH_CLEARANCE_M = 0.10
PICK_CLEARANCE_M     = 0.07
PICK_TOUCHDOWN_M     = 0.00
PITCH_DEFAULT        = 0.30           # IK-friendly
PLACE_POSE           = (0.40, -0.30, 0.20)  # right side (y < 0)
EXCLUDE_ARMTAG_RADIUS_M = 0.18
POLL_PERIOD_S        = 0.25

# Ollama (REST)
OLLAMA_URL   = os.getenv("OLLAMA_URL", "http://localhost:11434")
OLLAMA_MODEL = os.getenv("OLLAMA_MODEL", "moondream")  # fast vision model
HTTP_TIMEOUT_S = float(os.getenv("OLLAMA_HTTP_TIMEOUT", "45"))
GENERATE_ENDPOINT = f"{OLLAMA_URL.rstrip('/')}/api/generate"
# ------------------------------------------------

def quat_to_rotm(q): return tft.quaternion_matrix([q[0], q[1], q[2], q[3]])[:3, :3]

def rclpy_spin_once(node, timeout=0.0):
    try: rclpy.spin_once(node, timeout_sec=timeout)
    except Exception: time.sleep(timeout if timeout > 0 else 0.0)

# --- Camera model ---
class CamModel:
    def __init__(self):
        self.fx=self.fy=self.cx=self.cy=None; self.w=self.h=None; self.have_info=False
    def set_info(self, msg: CameraInfo):
        k = list(msg.k) if hasattr(msg, "k") else []; p = list(msg.p) if hasattr(msg, "p") else []
        if len(k)==9 and k[0]>0 and k[4]>0:
            self.fx,self.fy,self.cx,self.cy = float(k[0]),float(k[4]),float(k[2]),float(k[5])
        elif len(p)==12 and p[0]>0 and p[5]>0:
            self.fx,self.fy,self.cx,self.cy = float(p[0]),float(p[5]),float(p[2]),float(p[6])
        else:
            self.fx=self.fy=600.0; self.cx=float(msg.width)*0.5; self.cy=float(msg.height)*0.5
            print("[WARN] CameraInfo intrinsics missing/zero; using safe defaults.")
        self.w,self.h = int(msg.width), int(msg.height); self.have_info=True
        print(f"[INFO] CameraInfo set: w={self.w}, h={self.h}, fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")
    def project_cam_xyz_to_pixel(self, Xc, Yc, Zc):
        if not self.have_info or Zc<=0: return None
        return (self.fx*(Xc/Zc)+self.cx, self.fy*(Yc/Zc)+self.cy)

def base_to_camera(tf_buffer: Buffer):
    tf: TransformStamped = tf_buffer.lookup_transform(REF_FRAME, ARM_BASE_FRAME, Time(), timeout=Duration(seconds=0.5))
    t,q = tf.transform.translation, tf.transform.rotation
    return quat_to_rotm((q.x,q.y,q.z,q.w)), np.array([t.x,t.y,t.z])

def project_cluster_centers_to_uv01(clusters_xyz, tf_buffer: Buffer, cam: CamModel):
    try: R_bc, p_bc = base_to_camera(tf_buffer)
    except Exception as e:
        print(f"[WARN] TF base->camera unavailable: {e}"); return []
    out=[]
    for (xb,yb,zb) in clusters_xyz:
        pc = R_bc @ np.array([xb,yb,zb]) + p_bc
        uv = cam.project_cam_xyz_to_pixel(pc[0], pc[1], pc[2])
        if uv is None or not cam.w or not cam.h: continue
        out.append((uv[0]/cam.w, uv[1]/cam.h))
    return out

def try_set_pose_components(bot: InterbotixManipulatorXS, **kwargs):
    send = {k:v for k,v in kwargs.items() if v is not None}
    try:
        if send: bot.arm.set_ee_pose_components(**send)
        return True
    except Exception as e:
        print(f"[WARN] IK failed for pose {send}: {e}"); return False

def goto_pose_with_fallbacks(bot: InterbotixManipulatorXS, x=None,y=None,z=None, roll=None,pitch=None,yaw=None):
    if (roll is not None) or (pitch is not None) or (yaw is not None):
        if try_set_pose_components(bot, x=x,y=y,z=z, roll=roll,pitch=pitch,yaw=yaw): return True
    if pitch is not None:
        if try_set_pose_components(bot, x=x,y=y,z=z, pitch=pitch): return True
    return try_set_pose_components(bot, x=x,y=y,z=z)

def try_find_armtag(armtag: InterbotixArmTagInterface):
    try:
        res = armtag.find_ref_to_arm_base_transform()
        return True if (res is None or res is True) else bool(res)
    except Exception as e:
        print(f"[INFO] ARTag not found at this pose: {e}"); return False

def armtag_position_in_base(tf_buffer: Buffer):
    try:
        tf: TransformStamped = tf_buffer.lookup_transform(ARM_BASE_FRAME, ARM_TAG_FRAME, Time(), timeout=Duration(seconds=0.5))
        t = tf.transform.translation; return (t.x,t.y,t.z)
    except Exception: return None

def get_clusters(pcl: InterbotixPointCloudInterface):
    ok, clusters = pcl.get_cluster_positions(ref_frame=ARM_BASE_FRAME, sort_axis='x', reverse=True)
    return clusters if ok and clusters else []

def filter_clusters_away_from_armtag(clusters, tf_buffer: Buffer, radius_m: float):
    tag = armtag_position_in_base(tf_buffer)
    if tag is None:
        return [(c['position'][0],c['position'][1],c['position'][2]) for c in clusters]
    tx,ty,tz = tag; r2 = radius_m*radius_m; kept=[]
    for c in clusters:
        x,y,z = c['position']
        if (x-tx)**2 + (y-ty)**2 + (z-tz)**2 > r2: kept.append((x,y,z))
    return kept

# ---- Live image buffer for direct HTTP path ----
class ImageBuffer:
    def __init__(self):
        self.bridge = CvBridge() if CV_OK else None
        self.last_bgr=None; self.stamp_ns=0; self.size=None
    def cb(self, msg: Image):
        if not CV_OK: return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_bgr = img; self.size=(img.shape[1], img.shape[0])
            self.stamp_ns = int(msg.header.stamp.sec)*10**9 + int(msg.header.stamp.nanosec)
        except Exception as e:
            print(f"[WARN] cv_bridge conversion failed: {e}")

def encode_bgr_to_b64_jpg(bgr):
    if not CV_OK or bgr is None: return None
    ok, jpg = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    if not ok: return None
    return base64.b64encode(jpg.tobytes()).decode("ascii")

def call_generate_endpoint(img_b64, prompt, timeout_s):
    import urllib.request, json as _json
    payload = {"model": OLLAMA_MODEL, "prompt": prompt, "images": [img_b64], "stream": False, "format": "json"}
    req = urllib.request.Request(GENERATE_ENDPOINT, data=_json.dumps(payload).encode("utf-8"),
                                 headers={"Content-Type":"application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            data = _json.loads(resp.read().decode("utf-8"))
    except Exception as e:
        print(f"[WARN] /api/generate failed: {e}"); return "none"
    text = data.get("response","") or data.get("content","")
    if not text: return "none"
    try:
        parsed = json.loads(text); return parsed.get("bbox","none")
    except Exception:
        s=text.find('{'); e=text.rfind('}')
        if s>=0 and e>s:
            try: return json.loads(text[s:e+1]).get("bbox","none")
            except Exception: return "none"
        return "none"

def pick_cluster_closest_to_bbox_center(clusters_xyz, uv01_list, bbox):
    if not isinstance(bbox,(list,tuple)) or len(bbox)<2: return None
    xc,yc = float(bbox[0]), float(bbox[1])
    best_i,best_d2=None,float('inf')
    for i,(u,v) in enumerate(uv01_list):
        d2=(u-xc)**2 + (v-yc)**2
        if d2<best_d2: best_d2, best_i=d2, i
    return (best_i, clusters_xyz[best_i]) if best_i is not None else (None, None)

def wait_for_camera_frame(imgbuf, timeout_s=5.0):
    t0=time.time()
    while (time.time()-t0) < timeout_s:
        if imgbuf and imgbuf.last_bgr is not None:
            return True
        time.sleep(0.05)
    return False

def main():
    global_node = create_interbotix_global_node()
    tf_buffer = Buffer(); tf_listener = TransformListener(tf_buffer, global_node)

    bot = InterbotixManipulatorXS(robot_model=ROBOT_MODEL, robot_name=ROBOT_NAME, node=global_node)
    pcl = InterbotixPointCloudInterface(node_inf=global_node)
    armtag = InterbotixArmTagInterface(ref_frame=REF_FRAME, arm_tag_frame=ARM_TAG_FRAME,
                                       arm_base_frame=ARM_BASE_FRAME, node_inf=global_node)

    cam = CamModel()
    caminfo_got={"got":False}
    global_node.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
        lambda m: (not caminfo_got["got"]) and cam.set_info(m) or caminfo_got.update({"got":True}), 10)

    imgbuf = ImageBuffer() if CV_OK else None
    if CV_OK: global_node.create_subscription(Image, IMAGE_TOPIC, imgbuf.cb, 5)
    else: print("[WARN] OpenCV/cv_bridge not available; direct vision disabled.")

    robot_startup(global_node)

    try:
        # Safe start
        bot.arm.go_to_sleep_pose(); bot.gripper.release()
        goto_pose_with_fallbacks(bot, x=0.30, z=0.20, pitch=PITCH_DEFAULT)

        # Calibration hint (no motion required)
        print("[INFO] Calibration: place the arm AR tag under the camera. Waiting for TF...")
        t0=time.time()
        while armtag_position_in_base(tf_buffer) is None and (time.time()-t0)<10.0:
            _=try_find_armtag(armtag); rclpy_spin_once(global_node, 0.05)
        if armtag_position_in_base(tf_buffer) is None:
            print("[WARN] AR tag TF not found. Continuing anyway, but self-pick exclusion may be lax.")

        # Ensure CameraInfo arrived (non-fatal)
        t0=time.time()
        while not cam.have_info and (time.time()-t0)<5.0:
            rclpy_spin_once(global_node, 0.05)
        if not cam.have_info: print("[WARN] No CameraInfo; 2D matching will be approximate.")

        print("[INFO] Interactive mode. Type what to find (e.g., 'red ball'). Type 'q' to quit.")

        while True:
            # Ask the user what to find
            try:
                target = input("\nWhat should I find? (q to quit): ").strip()
            except EOFError:
                target = 'q'
            if not target:
                print("[HINT] Empty input; please enter an object description.")
                continue
            if target.lower() in ('q','quit','exit'):
                print("[INFO] Exiting by user request.")
                break

            # Make sure we have at least one recent frame
            if CV_OK and not wait_for_camera_frame(imgbuf, timeout_s=5.0):
                print("[WARN] No camera frame received; check topic and try again.")
                continue

            # Vision query
            prompt = f'Find the {target}. Return only JSON {{"bbox":[xc,yc,w,h]}} in 0..1 or {{"bbox":"none"}}.'
            b64 = encode_bgr_to_b64_jpg(imgbuf.last_bgr) if CV_OK else None
            bbox = call_generate_endpoint(b64, prompt, HTTP_TIMEOUT_S)
            print(f"[INFO] Vision bbox: {bbox}")

            # If not found by vision, report and reprompt
            if bbox == "none":
                print(f"[INFO] '{target}' not found.")
                continue

            # Get clusters
            clusters = get_clusters(pcl)
            clusters_xyz = filter_clusters_away_from_armtag(clusters, tf_buffer, EXCLUDE_ARMTAG_RADIUS_M)

            if not clusters_xyz:
                print(f"[INFO] '{target}' not found (no clusters).")
                continue

            # Need CameraInfo to correlate bbox to 3D cluster centers
            if not cam.have_info:
                print(f"[INFO] '{target}' not found (no CameraInfo for projection).")
                continue

            uv01 = project_cluster_centers_to_uv01(clusters_xyz, tf_buffer, cam)
            if not uv01:
                print(f"[INFO] '{target}' not found (projection failed).")
                continue

            idx, target_xyz = pick_cluster_closest_to_bbox_center(clusters_xyz, uv01, bbox)
            if idx is None or target_xyz is None:
                print(f"[INFO] '{target}' not found (no matching cluster).")
                continue

            tx,ty,tz = target_xyz
            print(f"[INFO] Target '{target}' → cluster[{idx}] at x={tx:.3f}, y={ty:.3f}, z={tz:.3f}")

            # Execute one pick→place cycle
            goto_pose_with_fallbacks(bot, x=tx, y=ty, z=tz + PICK_CLEARANCE_M, pitch=PITCH_DEFAULT)
            goto_pose_with_fallbacks(bot, x=tx, y=ty, z=tz + PICK_TOUCHDOWN_M,  pitch=PITCH_DEFAULT)
            bot.gripper.grasp()
            goto_pose_with_fallbacks(bot, x=tx, y=ty, z=tz + PICK_CLEARANCE_M, pitch=PITCH_DEFAULT)

            goto_pose_with_fallbacks(bot, x=PLACE_POSE[0], y=PLACE_POSE[1], z=PLACE_POSE[2], pitch=PITCH_DEFAULT)
            bot.gripper.release()

            # Return to ready and prompt again
            goto_pose_with_fallbacks(bot, x=0.30, z=0.20, pitch=PITCH_DEFAULT)
            time.sleep(POLL_PERIOD_S)

    except KeyboardInterrupt:
        print("\n[INFO] Stopping loop.")
    finally:
        try: goto_pose_with_fallbacks(bot, x=0.30, z=0.20, pitch=PITCH_DEFAULT)
        except Exception: pass
        try: bot.arm.go_to_sleep_pose()
        except Exception: pass
        try: robot_shutdown(global_node)
        except Exception: pass

if __name__ == '__main__':
    main()
