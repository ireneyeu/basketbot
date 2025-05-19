# This script uses YOLOv8 for object detection and OpenCV for tracking a basketball

import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
import redis

# Load YOLOv8 model (can be fine-tuned for better basketball performance)
model = YOLO("yolov8n.pt")  # lightweight model for speed
BALL_POSITION_KEY = "sai::camera::BALL::sensors::position"
BALL_VELOCITY_KEY = "sai::camera::BALL::sensors::velocity"
redis_client = redis.Redis()

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

                # # Get depth info
                # depth = depth_frame.get_distance(cx, cy)

                # Improved depth estimation by sampling around center
                sample_radius = 2  # pixels around the center (creates a (2*radius+1)^2 grid)
                depth_values = []

                for dx in range(-sample_radius, sample_radius + 1):
                    for dy in range(-sample_radius, sample_radius + 1):
                        sx, sy = cx + dx, cy + dy
                        if 0 <= sx < depth_frame.get_width() and 0 <= sy < depth_frame.get_height():
                            d = depth_frame.get_distance(sx, sy)
                            if d > 0:  # Ignore invalid (zero) depth
                                depth_values.append(d)

                # Filter and select best depth
                if depth_values:
                    depth_array = np.array(depth_values)
                    median = np.median(depth_array)
                    filtered_depths = depth_array[np.abs(depth_array - median) < 0.1]  # reject outliers >10cm from median
                    if len(filtered_depths) > 0:
                        depth = np.min(filtered_depths)  # use closest reasonable value
                    else:
                        depth = median  # fallback to median
                else:
                    depth = 0.0  # fallback if no valid depth


                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                X, Y, Z = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth)
                print(f"3D Position: X={X:.2f}m, Y={Y:.2f}m, Z={Z:.2f}m")

                ## Publish to Redis CHECK ALL THIS BLOCK #######
                ##################################################################
                p_cam = np.array([X, Y, Z])
                R_cam_to_world = np.array([
                                            [0, -1, 0],
                                            [1, 0, 0],
                                            [0, 0, 1] ])
                t_cam_to_world = np.array([0.5, 0.2, 0.1])
                p_world = R_cam_to_world @ p_cam + t_cam_to_world
                
                # Compute velocity with exponential smoothing
                current_time = cv2.getTickCount() / cv2.getTickFrequency()  # in seconds
                if 'last_position' not in locals():
                    last_position = p_world
                    last_time = current_time
                    velocity = np.array([0.0, 0.0, 0.0])
                    smoothed_velocity = np.array([0.0, 0.0, 0.0])
                else:
                    dt = current_time - last_time
                    if dt > 0:
                        raw_velocity = (p_world - last_position) / dt
                        alpha = 0.2  # smoothing factor between 0 (more smooth) and 1 (no smoothing)
                        smoothed_velocity = alpha * raw_velocity + (1 - alpha) * smoothed_velocity
                    else:
                        smoothed_velocity = np.array([0.0, 0.0, 0.0])
                    last_position = p_world
                    last_time = current_time

                # Send to Redis
                redis_client.set(BALL_POSITION_KEY, ' '.join(map(str, p_world)))
                redis_client.set(BALL_VELOCITY_KEY, ' '.join(map(str, velocity)))
           
                ####################################################################

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
