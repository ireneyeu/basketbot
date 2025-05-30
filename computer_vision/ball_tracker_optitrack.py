import numpy as np
import redis
import ast
import time

BALL_POSITION_KEY = "sai::camera::BALL::sensors::position"
BALL_VELOCITY_KEY = "sai::camera::BALL::sensors::velocity"
BALL_APEX_KEY = "sai::sim::BALL::sensors::apex"
BALL_POS_OPTITRACK = "sai2::optitrack::rigid_body_pos::6"

redis_client = redis.Redis()

R_OPTI_to_world = np.array([
    [0, 0, 1],
    [1, 0, 0],
    [0, 1, 0]
])  # Check in new setup

t_OPTI_to_world = np.array([3.45, 0.25, -0.35])  # Check in new setup

# Initialize previous state
prev_position_world = None
prev_time = None
smoothed_velocity = np.array([0.0, 0.0, 0.0])
alpha = 0.2  # Smoothing factor (0 = no smoothing, 1 = immediate update)

while True:
    try:
        p_opti_str = redis_client.get(BALL_POS_OPTITRACK)
        if p_opti_str is None:
            continue
        p_opti_str = p_opti_str.decode('utf-8')
        p_opti_float_list = ast.literal_eval(p_opti_str)
        p_opti = np.array(p_opti_float_list, dtype=float)
    except Exception as e:
        print(f"Error reading/parsing position: {e}")
        time.sleep(0.01)
        continue

    # Transform to world coordinates
    p_world = R_OPTI_to_world @ p_opti + t_OPTI_to_world
    current_time = time.time()

    # Send position to Redis
    # print(f"Position: {p_world}")
    redis_client.set(BALL_POSITION_KEY, ','.join(map(str, p_world)))

    if prev_position_world is not None and prev_time is not None:
        dt = current_time - prev_time
        if dt > 0:
            raw_velocity = (p_world - prev_position_world) / dt

            # Apply exponential smoothing
            smoothed_velocity = alpha * raw_velocity + (1 - alpha) * smoothed_velocity

            # Store smoothed velocity in Redis
            redis_client.set(BALL_VELOCITY_KEY, ','.join(map(str, smoothed_velocity)))

            # Predict apex
            g = 9.81
            vz = smoothed_velocity[2]
            z_current = p_world[2]

            if vz > 0:
                z_apex = z_current + (vz ** 2) / (2 * g)
            else:
                z_apex = -0.20

            redis_client.set(BALL_APEX_KEY, str(z_apex))
            print(f"X: {p_world[0]:.2f}, Y: {p_world[1]:.2f}, Z: {p_world[2]:.2f}, Apex Z prediction: {z_apex:.2f} m")

    prev_position_world = p_world
    prev_time = current_time
    time.sleep(0.005)
