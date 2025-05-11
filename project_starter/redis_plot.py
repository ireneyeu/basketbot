import numpy as np
import matplotlib.pyplot as plt
import time, redis
import signal

# Redis keys
EE_FORCES_KEY = "sai::sensors::PANDA::ft_sensor::end-effector::force"
BALL_POSITION_KEY = "sai::sim::BALL::sensors::position"
BALL_VELOCITY_KEY = "sai::sim::BALL::sensors::velocity"
EE_POSITION_KEY = "sai::sim::PANDA::end-effector::position"
EE_VELOCITY_KEY = "sai::sim::PANDA::end-effector::velocity"

# Redis connection
redis_client = redis.Redis()

# Data containers
EE_X_FORCE = []
EE_Y_FORCE = []
EE_Z_FORCE = []

BALL_X_POSITION = []
BALL_Y_POSITION = []
BALL_Z_POSITION = []

BALL_X_VELOCITY = []
BALL_Y_VELOCITY = []
BALL_Z_VELOCITY = []

EE_X = []
EE_Y = []
EE_Z = []

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
            EE_X_FORCE.append(robot_forces[0])
            EE_Y_FORCE.append(robot_forces[1])
            EE_Z_FORCE.append(robot_forces[2])

            ball_position_redis = redis_client.get(BALL_POSITION_KEY).decode("utf-8")
            ball_position = [float(x) for x in ball_position_redis.strip('[]').split(',')]
            BALL_X_POSITION.append(ball_position[0])
            BALL_Y_POSITION.append(ball_position[1])
            BALL_Z_POSITION.append(ball_position[2])

            ball_velocity_redis = redis_client.get(BALL_VELOCITY_KEY).decode("utf-8")
            ball_velocity = [float(x) for x in ball_velocity_redis.strip('[]').split(',')]
            BALL_X_VELOCITY.append(ball_velocity[0])
            BALL_Y_VELOCITY.append(ball_velocity[1])
            BALL_Z_VELOCITY.append(ball_velocity[2])

            ee_position_redis = redis_client.get(EE_POSITION_KEY).decode("utf-8")
            ee_position = [float(x) for x in ee_position_redis.strip('[]').split(',')]
            EE_X.append(ee_position[0])
            EE_Y.append(ee_position[1])
            EE_Z.append(ee_position[2])


        except Exception as e:
            print(f"Error: {e}")
            pass

finally:
    # Plotting the results
    print("Data collection completed. Values received.")
    

    print("Plotting...")
    
    # plt.figure()
    # plt.plot(EE_X_FORCE, label='X Force', color='r')
    # plt.plot(EE_Y_FORCE, label='Y Force', color='g')
    # plt.plot(EE_Z_FORCE, label='Z Force', color='b')
    # plt.xlabel("Time [s]")
    # plt.ylabel("Force [N]")
    # plt.title("End Effector Forces")
    # plt.legend()
    # plt.grid(True)
    # # plt.savefig("end_effector_forces.png")
    # plt.show()

    plt.figure()
    plt.plot(BALL_X_POSITION, label='X Position', color='r')
    plt.plot(BALL_Y_POSITION, label='Y Position', color='g')
    plt.plot(BALL_Z_POSITION, label='Z Position', color='b')
    plt.plot(EE_X, label='EE X', color='c', linestyle='--')
    plt.plot(EE_Y, label='EE Y', color='m', linestyle='-')
    plt.plot(EE_Z, label='EE Z', color='y', linestyle=':')
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("Ball Position")
    plt.legend()
    plt.grid(True)
    # plt.savefig("ball_position.png")
    plt.show()

    plt.figure()
    plt.plot(BALL_X_VELOCITY, label='X Velocity', color='r')
    plt.plot(BALL_Y_VELOCITY, label='Y Velocity', color='g')
    plt.plot(BALL_Z_VELOCITY, label='Z Velocity', color='b')
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Ball Velocity")
    plt.legend()
    plt.grid(True)
    # plt.savefig("ball_velocity.png")
    plt.show()

    print("Plotting done.")
