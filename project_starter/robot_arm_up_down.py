import numpy as np
import time, json, redis
from enum import Enum, auto


class State(Enum):
    GOING_UP = auto()
    GOING_DOWN = auto()


goal_position_redis_key = "opensai::controllers::PANDA::cartesian_controller::motion_force_task::goal_position"
current_position_redis_key = "opensai::controllers::PANDA::cartesian_controller::motion_force_task::current_position"

# redis client
redis_client = redis.Redis()
init_position = np.array(
    json.loads(redis_client.get(current_position_redis_key)))
state = State.GOING_UP

try:
    while True:
        if state == State.GOING_UP:
            print("Going UP")
            # ---------------------
            # modify the following line
            goal_position = init_position + np.array([0, 0, 0.1])
            # ---------------------
            redis_client.set(goal_position_redis_key,
                             json.dumps(goal_position.tolist()))
            state = State.GOING_DOWN
            time.sleep(1.5)

        elif state == State.GOING_DOWN:
            print("Going DOWN")
            # ---------------------
            # modify the following line
            goal_position = init_position + np.array([0, 0, -0.1])
            # ---------------------
            redis_client.set(goal_position_redis_key,
                             json.dumps(goal_position.tolist()))
            state = State.GOING_UP
            time.sleep(1.5)

except KeyboardInterrupt:
    print("Keyboard interrupt")
    pass
except Exception as e:
    print(e)
    pass
redis_client.set(goal_position_redis_key, json.dumps(init_position.tolist()))
