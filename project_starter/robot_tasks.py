import numpy as np
import time, json, redis
from enum import Enum, auto


class State(Enum):
    REST = auto()
    GOING_START_POS = auto()


goal_position_redis_key = "sai::controllers::PANDA::cartesian_controller::motion_force_task::goal_position"
current_position_redis_key = "sai::controllers::PANDA::cartesian_controller::motion_force_task::current_position"
goal_orientation_redis_key = "sai::controllers::PANDA::cartesian_controller::motion_force_task::goal_orientation"
current_orientation_redis_key = "sai::controllers::PANDA::cartesian_controller::motion_force_task::current_orientation"
ee_forces = "sai::sensors::PANDA::ft_sensor::end-effector::force"
ee_moments = "sai::sensors::PANDA::ft_sensor::end-effector::moment"

# redis client
redis_client = redis.Redis()
init_position = np.array(
    json.loads(redis_client.get(current_position_redis_key)))
init_orientation = np.array(
    json.loads(redis_client.get(current_orientation_redis_key)))
rest_position = init_position
state = State.REST

try:
    while True:
        if state == State.REST:
            print("In rest state")
            rest_position = np.array(
                json.loads(redis_client.get(current_position_redis_key)))
            cmd = input("Press 1 to go to start position, 2 to go to initial position or ctrl + C to quit: ")

        if cmd == '1':
            print("Going to start position")
            goal_position = np.array([0.5, 0, 0.25])
            redis_client.set(goal_position_redis_key,
                             json.dumps(goal_position.tolist()))
            goal_orientation = np.array([[1, 0, 0.0],
                                          [0, -1, 0], 
                                          [0.0, 0, -1]])
            redis_client.set(goal_orientation_redis_key,
                             json.dumps(goal_orientation.tolist()))
            state = State.REST
            print("Waiting for robot to reach start position")
            while True:
                current_position = np.array(
                    json.loads(redis_client.get(current_position_redis_key)))
                if np.linalg.norm(current_position - goal_position) < 0.01:
                    print("Reached start position")
                    current_orientation = np.array(
                        json.loads(redis_client.get(current_orientation_redis_key)))
                    print("Current orientation: ", current_orientation)
                    break
                time.sleep(0.5)
        elif cmd == '2':
            print("Going to initial position")
            goal_position = init_position
            goal_orientation = init_orientation
        
            redis_client.set(goal_position_redis_key,
                             json.dumps(goal_position.tolist()))
            redis_client.set(goal_orientation_redis_key,
                             json.dumps(goal_orientation.tolist()))
            state = State.REST
            print("Waiting for robot to reach initial position")
            while True:
                current_position = np.array(
                    json.loads(redis_client.get(current_position_redis_key)))
                if np.linalg.norm(current_position - goal_position) < 0.01:
                    print("Reached initial position")
                    break
                time.sleep(0.5)

except KeyboardInterrupt:
    print("Keyboard interrupt")
    pass
except Exception as e:
    print(e)
    pass
redis_client.set(goal_position_redis_key, json.dumps(init_position.tolist()))
