import numpy as np
import matplotlib.pyplot as plt
import time, redis
import signal

# Redis keys
EE_FORCES_KEY = "sai::sensors::PANDA::ft_sensor::end-effector::force";

# Redis connection
redis_client = redis.Redis()

# Data containers
EE_X_FORCE = []
EE_Y_FORCE = []
EE_Z_FORCE = []

# Graceful shutdown on Ctrl+C
running = True
def signal_handler(sig, frame):
    global running
    running = False
    print("\nStopping data collection...")
signal.signal(signal.SIGINT, signal_handler)

print("Listening to Redis for values... Press Ctrl+C to stop and plot.")
try:
    while running:
        try:
            robot_forces_redis = redis_client.get(EE_FORCES_KEY).decode("utf-8")
            robot_forces = [float(x) for x in robot_forces_redis.strip('[]').split(',')]
            ee_x_force = robot_forces[0]
            ee_y_force = robot_forces[1]
            ee_z_force = robot_forces[2]
            EE_X_FORCE.append(ee_x_force)
            EE_Y_FORCE.append(ee_y_force)
            EE_Z_FORCE.append(ee_z_force)

        except Exception as e:
            print(f"Error: {e}")
            pass

finally:
    # Plotting the results
    print("Data collection completed. Values received.")
    

    print("Plotting...")
    
    plt.figure()
    plt.plot(EE_X_FORCE, label='X Force', color='r')
    plt.plot(EE_Y_FORCE, label='Y Force', color='g')
    plt.plot(EE_Z_FORCE, label='Z Force', color='b')
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.title("End Effector Forces")
    plt.legend()
    plt.grid(True)
    # plt.savefig("end_effector_forces.png")
    plt.show()

    print("Plotting done.")
