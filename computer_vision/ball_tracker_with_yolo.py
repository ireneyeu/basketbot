# This script uses YOLOv8 for object detection and OpenCV for tracking a basketball

import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO

# Load YOLOv8 model (can be fine-tuned for better basketball performance)
model = YOLO("yolov8n.pt")  # lightweight model for speed

# Choose OpenCV tracker
def create_tracker(tracker_type="CSRT"):
    # Try legacy module first
    if hasattr(cv2, 'legacy'):
        try:
            if tracker_type == "CSRT":
                return cv2.legacy.TrackerCSRT_create()
            elif tracker_type == "KCF":
                return cv2.legacy.TrackerKCF_create()
            elif tracker_type == "MOSSE":
                return cv2.legacy.TrackerMOSSE_create()
        except AttributeError:
            pass

    # Fallback to standard module (for OpenCV >= 4.5)
    if tracker_type == "CSRT":
        return cv2.TrackerCSRT_create()
    elif tracker_type == "KCF":
        return cv2.TrackerKCF_create()
    elif tracker_type == "MOSSE":
        return cv2.TrackerMOSSE_create()

    raise ValueError(f"Unsupported tracker type: {tracker_type}")

# def create_tracker():
#     return cv2.legacy.TrackerCSRT_create()  # Use KCF, MOSSE, etc. for alternatives

# RealSense setup
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Tracking state
tracker = None
bbox = None
tracking = False

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())

        if not tracking:
            # YOLOv8 detection
            results = model(color_image)
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    label = model.names[cls_id]
                    if label == "sports ball":
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        bbox = (x1, y1, x2 - x1, y2 - y1)
                        tracker = create_tracker()
                        tracker.init(color_image, bbox)
                        tracking = True
                        break
                if tracking:
                    break
        else:
            success, bbox = tracker.update(color_image)
            if success:
                x, y, w, h = map(int, bbox)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

                # Get depth info
                depth = depth_frame.get_distance(cx, cy)
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth)
                print(f"3D Position: X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m")
            else:
                cv2.putText(color_image, "Lost tracking, retrying...", (30, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                tracking = False  # fall back to detection

        # Display
        cv2.imshow("YOLO + Tracker", color_image)
        if cv2.waitKey(1) == 27:
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
