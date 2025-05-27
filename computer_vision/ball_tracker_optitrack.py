
import numpy as np
import redis
import ast


BALL_POSITION_KEY = "sai::camera::BALL::sensors::position"
BALL_VELOCITY_KEY = "sai::camera::BALL::sensors::velocity"
BALL_APEX_KEY = "sai::sim::BALL::sensors::apex"
BALL_POS_OPTITRACK = "sai2::optitrack::rigid_body_pos::6"
## 
# R_cam_to_world = np.array([
#                                             [0, 0, 1],
#                                             [1, 0, 0],
#                                             [0, 1, 0] ])       ## Check in new setup
# t_cam_to_world = np.array([3.70, 0.46, 0.38])                  ## Check in new setup

redis_client = redis.Redis()

R_OPTI_to_world = np.array([
                                            [0, 0, 1],
                                            [1, 0, 0],
                                            [0, 1, 0] ])       ## Check in new setup
t_OPTI_to_world = np.array([3.70, 0.46, 0.38]) ## Check in new setup

# p_opti = np.array(redis_client.get(BALL_POS_OPTITRACK).decode("utf-8"), dtype="float32")
p_opti_str = redis_client.get(BALL_POS_OPTITRACK).decode('utf-8')
float_list = ast.literal_eval(p_opti_str)
p_opti = np.array(float_list, dtype=float)


print(R_OPTI_to_world)
print(p_opti)
p_world = R_OPTI_to_world @ p_opti + t_OPTI_to_world
print(p_world)



# try:
#     import time

#     while True:

        

#             ## Publish to Redis CHECK ALL THIS BLOCK #######
#             ##################################################################
#             p_cam = np.array([X, Y, Z])
#             # R_cam_to_world = np.array([
#             #                             [0, 0, -1],
#             #                             [1, 0, 0],
#             #                             [0, -1, 0] ])     # Previous setup
#             # t_cam_to_world = np.array([0.5, 0.5, 0.1])     # Previous setup

#             R_cam_to_world = np.array([
#                                         [1, 0, 0],
#                                         [0, 0, 1],
#                                         [0, -1, 0] ])     ## Check in new setup
#             t_cam_to_world = np.array([0.5, -2.0, 0.50])    ## Check in new setup
#             p_world = R_cam_to_world @ p_cam + t_cam_to_world

            
#             # Compute velocity with exponential smoothing
#             current_time = cv2.getTickCount() / cv2.getTickFrequency()  # in seconds
#             if 'last_position' not in locals():
#                 last_position = p_world
#                 last_time = current_time
#                 velocity = np.array([0.0, 0.0, 0.0])
#                 smoothed_velocity = np.array([0.0, 0.0, 0.0])
#             else:
#                 dt = current_time - last_time
#                 if dt > 0:
#                     raw_velocity = (p_world - last_position) / dt
#                     alpha = 0.2  # smoothing factor between 0 (more smooth) and 1 (no smoothing)
#                     smoothed_velocity = alpha * raw_velocity + (1 - alpha) * smoothed_velocity
#                 else:
#                     smoothed_velocity = np.array([0.0, 0.0, 0.0])
#                 last_position = p_world
#                 last_time = current_time

#             # Send to Redis
#             # Position smoothing and outlier rejection
#             alpha_pos = 0.3  # smoothing factor for position
#             max_delta_pos = 0.5  # max allowed jump (meters)

#             if 'smoothed_position' not in locals():
#                 smoothed_position = p_world.copy()
#             else:
#                 delta = np.linalg.norm(p_world - smoothed_position)
#                 if delta < max_delta_pos:
#                     smoothed_position = alpha_pos * p_world + (1 - alpha_pos) * smoothed_position
#                 else:
#                     print(f"Outlier detected (Δ={delta:.2f} m), ignoring jump.")
#                     # Optionally clamp: smoothed_position = smoothed_position + np.clip(p_world - smoothed_position, -max_delta_pos, max_delta_pos)

#             # Send position and velocity to Redis
#             redis_client.set(BALL_POSITION_KEY, ','.join(map(str, smoothed_position)))
#             redis_client.set(BALL_VELOCITY_KEY, ','.join(map(str, smoothed_velocity)))

#             # Predict apex height if vertical velocity is upward in world frame
#             g = 9.81  # gravitational acceleration in m/s²
#             vz = smoothed_velocity[2]  # Z is vertical in world frame
#             z_current = smoothed_position[2]

#             if vz > 0:
#                 z_apex = z_current + (vz ** 2) / (2 * g)
#             else:
#                 z_apex = 0.0

#             redis_client.set(BALL_APEX_KEY, str(z_apex))
#             print(f"X: {smoothed_position[0]:.2f}, Y: {smoothed_position[1]:.2f}, Z: {smoothed_position[2]:.2f}, Apex Z prediction: {z_apex:.2f} m")

        
#             ####################################################################

#         else:
#             cv2.putText(color_image, "Lost tracking, retrying...", (30, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
#             tracking = False  # fall back to detection

#     # Display
#     cv2.imshow("YOLO + Tracker", color_image)
#     if cv2.waitKey(1) == 27:
#         break
# finally:
#     pipeline.stop()
#     cv2.destroyAllWindows()
