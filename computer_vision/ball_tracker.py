# This script tracks an object after selecting it with a bounding box.

import cv2
import pyrealsense2 as rs
import numpy as np

# Choose tracker type
tracker_type = 'CSRT'  # Options: KCF, CSRT, MIL, BOOSTING, MEDIANFLOW, TLD, MOSSE

def create_tracker(tracker_type):
    if tracker_type == 'CSRT':
        return cv2.legacy.TrackerCSRT_create()
    elif tracker_type == 'KCF':
        return cv2.legacy.TrackerKCF_create()
    elif tracker_type == 'MOSSE':
        return cv2.legacy.TrackerMOSSE_create()
    else:
        raise ValueError(f"Unsupported tracker type: {tracker_type}")

# Setup RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

# Read one frame to initialize
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
color_image = np.asanyarray(color_frame.get_data())

# Let user select ROI (bounding box)
bbox = cv2.selectROI("Select Ball", color_image, fromCenter=False, showCrosshair=True)
cv2.destroyWindow("Select Ball")

# Initialize tracker
tracker = create_tracker(tracker_type)
tracker.init(color_image, bbox)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())

        # Update tracker
        success, bbox = tracker.update(color_image)

        if success:
            x, y, w, h = [int(v) for v in bbox]
            cx, cy = x + w // 2, y + h // 2

            # Draw tracking box
            cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

            # Get depth at center
            depth = depth_frame.get_distance(cx, cy)
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            point_3d = rs.rs2_deproject_pixel_to_point(depth_intrin, [cx, cy], depth)
            X, Y, Z = point_3d
            print(f"2D: ({cx}, {cy})  |  3D: ({X:.2f}, {Y:.2f}, {Z:.2f}) m")
        else:
            cv2.putText(color_image, "Tracking failure!", (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Show frame
        cv2.imshow("Basketball Tracker", color_image)
        if cv2.waitKey(1) == 27:  # ESC to exit
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
