# perception.py — minimal, fast, and ready for depth 
import cv2, torch, numpy as np

# --- fill these later (calibration) ---
K = np.array([[600.0, 0.0, 320.0],
              [0.0, 600.0, 240.0],
              [0.0,   0.0,   1.0]], dtype=float)   # fx,fy,cx,cy
T_base_cam = np.eye(4, dtype=float)                # 4x4 cam->base
Z_TABLE_M = 0.10                                    # table height (m)
DEPTH_SCALE = 0.001                                 # units->m (e.g., RealSense)

# --- YOLOv5: load once, eval, pick device ---
_DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
_MODEL = None
def _get_model():
    global _MODEL
    if _MODEL is None:
        _MODEL = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        _MODEL = _MODEL.to(_DEVICE).eval()
    return _MODEL

def _backproject(u, v, Z):
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    Xc = (u - cx) * Z / fx
    Yc = (v - cy) * Z / fy
    return np.array([Xc, Yc, Z, 1.0], dtype=float)

def _cam_to_base(Pc_h):
    return (T_base_cam @ Pc_h)[:3]

def _get_cv_frame(index=0, w=640, h=480):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # works well on Windows
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    ok, bgr = cap.read()
    cap.release()
    if not ok: raise RuntimeError("No camera frame")
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

def compute_object_pose_base(depth_frame=None, depth_scale=DEPTH_SCALE):
    """Return (x,y,z) in BASE. Uses depth if given; else falls back to table Z."""
    model = _get_model()
    rgb = _get_cv_frame(0, 640, 480)

    with torch.no_grad():                  # inference only
        res = model(rgb, size=640)         # default imgsz
        preds = res.xyxy[0].cpu().numpy()  # no pandas dep
    if preds.size == 0:
        print("no detections")
        return 0.60, 0.00, 0.10

    # highest conf
    x1,y1,x2,y2,conf,cls_id = max(preds, key=lambda p: p[4])
    u = int(0.5*(x1+x2)); v = int(0.5*(y1+y2))
    
    # Get class name from model
    class_name = model.names[int(cls_id)]
    print(f"px=({u},{v}) conf={conf:.2f} class={int(cls_id)} ({class_name})")

    # depth if we have it; else table
    Zm = Z_TABLE_M
    if depth_frame is not None:
        z_raw = float(depth_frame[v, u])
        if z_raw > 0.0 and np.isfinite(z_raw):
            Zm = z_raw * depth_scale

    Pc = _backproject(u, v, Zm)            # cam frame
    xb, yb, zb = _cam_to_base(Pc)          # base frame
    return float(xb), float(yb), float(zb)

# ---------------- camera stubs (uncomment when ready) ----------------
# RealSense (pyrealsense2): depth aligned to color + real depth scale
# import pyrealsense2 as rs, numpy as np
# def get_realsense_frames():
#     pipe=rs.pipeline(); cfg=rs.config()
#     cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16,30)
#     cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8,30)
#     prof=pipe.start(cfg)
#     try:
#         depth_scale = float(prof.get_device().first_depth_sensor().get_depth_scale())
#         align = rs.align(rs.stream.color)               # depth -> color
#         fs = align.process(pipe.wait_for_frames())
#         d = np.asanyarray(fs.get_depth_frame().get_data())
#         bgr = np.asanyarray(fs.get_color_frame().get_data())
#         rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
#         return rgb, d, depth_scale
#     finally:
#         pipe.stop()

# OAK-D (DepthAI): align depth to RGB via StereoDepth alignment
# import depthai as dai, numpy as np
# def get_oakd_frames():
#     # build a tiny pipeline: ColorCamera + StereoDepth (align to RGB)
#     # pull frames via XLinkOut queues
#     raise NotImplementedError  # keep it short for now

# ZED 2/2i: grab(), then retrieve_measure(DEPTH) aligned to LEFT image
# import pyzed.sl as sl, numpy as np
# def get_zed_frames():
#     # open camera, zed.grab(), zed.retrieve_image(VIEW.LEFT), zed.retrieve_measure(MEASURE.DEPTH)
#     # convert to numpy; depth already aligned to LEFT
#     raise NotImplementedError

# -------------------- Test/Demo Mode --------------------
if __name__ == "__main__":
    print("Testing perception system...")
    print("Make sure a camera is connected and visible objects are in view.")
    
    try:
        x, y, z = compute_object_pose_base()
        print(f"✅ Object detected at: x={x:.3f}m, y={y:.3f}m, z={z:.3f}m")
        print("Perception system working correctly!")
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Check camera connection and dependencies.")
