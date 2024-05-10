from jetbot import Robot
import time


# This will be used to store the actions that the robot
# can perform using the decorator register_action
_ACTIONS = {}

# Write a decorator for the methods of the class to register
# legal actions for the robot
def register_action(func):
    _ACTIONS[func.__name__] = func
    return func


class Navigation:

    def __init__(self, speed=0.3, move_time=0.5):
    
        self._robot = Robot()
        self._speed = speed
        self._move_time = move_time
    
    def execute(self, action):
        
        _ACTIONS[action](self)

    @classmethod
    def get_available_actions(cls):
        return list(_ACTIONS.keys())

    @register_action
    def stop(self):
        self._robot.stop()

    @register_action
    def forward(self):
        self._robot.forward(self._speed)
        time.sleep(self._move_time)
        self._robot.stop()
    
    @register_action
    def backward(self):
        self._robot.backward(self._speed)
        time.sleep(self._move_time)
        self._robot.stop()
    
    @register_action
    def left(self):
        self._robot.left(self._speed)
        time.sleep(self._move_time)
        self._robot.stop()
    
    @register_action
    def right(self):
        self._robot.right(self._speed)
        time.sleep(self._move_time)
        self._robot.stop()
