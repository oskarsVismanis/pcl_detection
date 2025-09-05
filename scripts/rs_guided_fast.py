# rs_guided_fast.py
import os, time, argparse
import numpy as np, cv2, onnxruntime as ort, pyrealsense2 as rs
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg

Z_MIN, Z_MAX = 0.20, 4.00  # meters

class PointCloudPublisher(Node):
    def __init__(self, topic_name="depth_cloud"):

        super().__init__("pointcloud_pub")
        self.pub = self.create_publisher(PointCloud2, topic_name, 10)

    def publish_pointcloud(self, filled_depth_m, intrinsics):

        H, W = filled_depth_m.shape
        fx, fy = intrinsics.fx, intrinsics.fy
        cx, cy = intrinsics.ppx, intrinsics.ppy

        # Pixel grid
        i, j = np.indices((H, W))
        z = filled_depth_m
        x = (j - cx) * z / fx
        y = (i - cy) * z / fy

        # Flatten to (N, 3)
        points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

        # Create PointCloud2
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id="camera_link"
        cloud_msg = point_cloud2.create_cloud_xyz32(
            header=header,
            points=points
        )

        self.pub.publish(cloud_msg)

# ---------------- ONNX session (TRT -> CUDA -> CPU) ----------------
def make_sess(path):
    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.intra_op_num_threads = max(os.cpu_count() or 1, 1)
    so.inter_op_num_threads = 1

    avail = ort.get_available_providers()
    providers = []
    if "TensorrtExecutionProvider" in avail:
        providers.append(("TensorrtExecutionProvider", {
            "trt_fp16_enable": True,
            "trt_engine_cache_enable": True,
            "trt_engine_cache_path": "trt_cache",
            "trt_max_workspace_size": 1 << 29,
        }))
    if "CUDAExecutionProvider" in avail:
        providers.append(("CUDAExecutionProvider", {"cudnn_conv_use_max_workspace": "1"}))
    providers.append("CPUExecutionProvider")

    sess = ort.InferenceSession(path, so, providers=providers)
    print("Active providers:", sess.get_providers())

    inputs = sess.get_inputs()
    out_name = sess.get_outputs()[0].name
    Hm = Wm = None
    for i in inputs:
        s = i.shape
        if len(s) == 4 and isinstance(s[2], int) and isinstance(s[3], int):
            Hm, Wm = int(s[2]), int(s[3]); break
    print("ONNX inputs:", [(i.name, i.shape) for i in inputs], "-> HxW:", Hm, Wm)
    return sess, inputs, out_name, Hm, Wm

def pack_inputs(input_defs, rgb, sparse, mask):
    feed = {}
    for inp in input_defs:
        name = inp.name; s = inp.shape
        c = s[1] if (len(s) == 4 and isinstance(s[1], int)) else None
        lname = name.lower()
        if "mask" in lname or "valid" in lname:
            feed[name] = mask
        elif c == 3 or "rgb" in lname or "image" in lname or "color" in lname or "input.1" in lname:
            feed[name] = rgb
        elif c == 1 or "depth" in lname or "sparse" in lname or name == "0":
            feed[name] = sparse
        else:
            feed[name] = sparse if len(feed) == 0 else rgb
    return feed

# ---------------- preprocess / inference ----------------
def prep_rgb(bgr, W, H):
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_AREA)
    mean = np.array([0.485, 0.456, 0.406], np.float32)
    std  = np.array([0.229, 0.224, 0.225], np.float32)
    rgb  = (rgb - mean) / std
    return np.transpose(rgb, (2, 0, 1))[None].astype(np.float32)

def run_fullframe(sess, in_defs, out_name, bgr, depth_m, valid, W_in, H_in):
    rgb = prep_rgb(bgr, W_in, H_in)
    sd  = (depth_m * valid).astype(np.float32)
    mk  = valid.astype(np.float32)
    sd_s = cv2.resize(sd, (W_in, H_in), interpolation=cv2.INTER_NEAREST)[None, None]
    mk_s = cv2.resize(mk, (W_in, H_in), interpolation=cv2.INTER_NEAREST)[None, None]
    out = sess.run([out_name], pack_inputs(in_defs, rgb, sd_s, mk_s))[0]  # (1,1,H,W)
    dense = out[0, 0]
    return cv2.resize(dense, (depth_m.shape[1], depth_m.shape[0]), interpolation=cv2.INTER_LINEAR)

# ---------------- temporal helper ----------------
class TemporalMedian:
    def __init__(self, H, W, K=6):
        self.buf = deque(maxlen=K)
        self.H, self.W = H, W
    def push(self, depth_m):
        self.buf.append(depth_m.copy())
    def fill(self, depth_m, valid_now):
        """Fill holes using temporal median from stored frames."""
        if not self.buf: return depth_m
        filled = depth_m.copy()
        hole = ~valid_now
        if not hole.any(): return filled
        stack = np.stack(self.buf, 0)  # (K,H,W)
        v = (stack > 0) & (stack >= Z_MIN) & (stack <= Z_MAX)
        anyv = v.any(0) & hole
        if anyv.any():
            stk = np.where(v, stack, np.nan)
            med = np.nanmedian(stk, 0)
            ok = np.isfinite(med) & anyv
            filled[ok] = med[ok]
        return filled

# ---------------- main ----------------
def main():

    rclpy.init()
    ros_node = PointCloudPublisher()

    ap = argparse.ArgumentParser()
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--w", type=int, required=True)
    ap.add_argument("--h", type=int, required=True)
    ap.add_argument("--every", type=int, default=3, help="Run inference every N frames")
    ap.add_argument("--temporal", type=int, default=6, help="Temporal median window size")
    ap.add_argument("--dilate", type=int, default=2, help="Dilation radius for hole mask")
    ap.add_argument("--jb", action="store_true", help="Joint bilateral only on filled pixels")
    ap.add_argument("--filters", action="store_true", help="Apply RealSense spatial+temporal filters")
    ap.add_argument("--rs-res", choices=["640x480", "424x240"], default="424x240")
    args = ap.parse_args()

    sess, in_defs, out_name, Hm, Wm = make_sess(args.onnx)
    W_in, H_in = int(args.w), int(args.h)

    # RealSense setup
    pipe = rs.pipeline(); cfg = rs.config()
    if args.rs_res == "640x480":
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    else:
        cfg.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 60)
        cfg.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()
    depth_scale = prof.get_device().first_depth_sensor().get_depth_scale()

    # Get depth intrinsics here
    depth_stream = prof.get_stream(rs.stream.depth)
    depth_intr = depth_stream.as_video_stream_profile().get_intrinsics()

    spatial = rs.spatial_filter(); temporal = rs.temporal_filter()
    spatial.set_option(rs.option.holes_fill, 1)
    hole = rs.hole_filling_filter(); hole.set_option(rs.option.holes_fill, 1)

    t_prev = time.time(); fps = 0.0
    frame = 0; filled_cached = None; tm = None

    try:
        while True:
            frames = pipe.wait_for_frames()
            frames = align.process(frames)
            df, cf = frames.get_depth_frame(), frames.get_color_frame()
            if not df or not cf: continue
            if args.filters: df = temporal.process(spatial.process(df))
            df = hole.process(df)

            depth_z = np.asanyarray(df.get_data())
            rgb     = np.asanyarray(cf.get_data())
            H, W    = depth_z.shape
            depth_m = depth_z.astype(np.float32) * depth_scale
            valid   = (depth_m > 0) & (depth_m >= Z_MIN) & (depth_m <= Z_MAX)

            if tm is None: tm = TemporalMedian(H, W, K=max(1, args.temporal))
            tm.push(depth_m)

            # 1) Temporally close flickering holes
            temp = tm.fill(depth_m, valid)
            valid_temp = (temp > 0) & (temp >= Z_MIN) & (temp <= Z_MAX)

            need_ml = (frame % args.every == 0) or (filled_cached is None)
            if need_ml:
                # 2) Full-frame model inference
                dense = run_fullframe(sess, in_defs, out_name, rgb, temp, valid_temp, W_in, H_in)

                # 3) Create hole mask after temporal fill, slightly expand to close "fringes"
                holes = (~valid_temp).astype(np.uint8)
                if args.dilate > 0:
                    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * args.dilate + 1, 2 * args.dilate + 1))
                    holes = cv2.dilate(holes, k, iterations=1)
                paste = holes.astype(bool)  # fill only here

                filled = temp.copy()
                filled[paste] = dense[paste]

                # 4) (optional) edge-aware smoothing only in filled pixels
                if args.jb:
                    try:
                        jf = cv2.ximgproc.jointBilateralFilter(
                            rgb, filled.astype(np.float32),
                            d=5, sigmaColor=0.03, sigmaSpace=2
                        )
                        m = paste.astype(np.float32)
                        filled = filled * (1 - m) + jf * m
                    except Exception:
                        pass

                filled = np.clip(filled, Z_MIN, Z_MAX)
                filled_cached = filled
            else:
                filled = filled_cached

            # Publish to ROS node
            ros_node.publish_pointcloud(filled, depth_intr)
            rclpy.spin_once(ros_node, timeout_sec=0)

            # Visualization
            raw_vis = np.asanyarray(colorizer.colorize(df).get_data())
            z8 = cv2.normalize(filled, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            done = cv2.applyColorMap(z8, cv2.COLORMAP_JET)
            view = np.hstack([rgb, raw_vis, done])

            # FPS (EMA smoothing)
            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - t_prev, 1e-3))
            t_prev = now
            cv2.putText(view, f"{fps:4.1f} FPS  every={args.every}  T={args.temporal}  RS={args.rs_res}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            cv2.imshow("RGB | Raw | Completed (fast+temporal)", view)
            if cv2.waitKey(1) & 0xFF in (27, ord('q')): break
            frame += 1
    finally:
        pipe.stop(); cv2.destroyAllWindows()
        ros_node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
