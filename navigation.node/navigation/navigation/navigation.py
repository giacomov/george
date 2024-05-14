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


class CalibratedRobot(Robot):

    def __init__(self, left_motor_calibration=0.9, right_motor_calibration=1.0, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._left_motor_calibration = left_motor_calibration
        self._right_motor_calibration = right_motor_calibration
    
    def forward(self, speed=1.0, duration=None):
        self.left_motor.value = speed * self._left_motor_calibration
        self.right_motor.value = speed * self._right_motor_calibration

    def backward(self, speed=1.0):
        self.left_motor.value = -speed  * self._left_motor_calibration
        self.right_motor.value = -speed * self._right_motor_calibration
 
    def left(self, speed=1.0):
        self.left_motor.value = -speed * self._left_motor_calibration
        self.right_motor.value = speed * self._right_motor_calibration

    def right(self, speed=1.0):
        self.left_motor.value = speed * self._left_motor_calibration
        self.right_motor.value = -speed * self._right_motor_calibration

    def stop(self):
        self.left_motor.value = 0
        self.right_motor.value = 0


class Navigation:

    def __init__(self, speed=0.3, move_time=0.2):
    
        self._robot = CalibratedRobot()
        self._speed = speed
        self._move_time = move_time
    
    def execute(self, action, steps=1):
        
        _ACTIONS[action](self, steps)

    @classmethod
    def get_available_actions(cls):
        return list(_ACTIONS.keys())
    
    @register_action
    def speed(self, new_speed=3):

        if new_speed > 10:
            self._speed = 1
        else:
            self._speed = new_speed / 10

    @register_action
    def stop(self, step=None):
        self._robot.stop()

    @register_action
    def forward(self, steps=1):
        self._robot.forward(self._speed)
        time.sleep(self._move_time * steps)
        self._robot.stop()
    
    @register_action
    def backward(self, steps=1):
        self._robot.backward(self._speed)
        time.sleep(self._move_time * steps)
        self._robot.stop()
    
    @register_action
    def left(self, steps=1):
        self._robot.left(self._speed)
        time.sleep(self._move_time * steps)
        self._robot.stop()
    
    @register_action
    def right(self, steps=1):
        self._robot.right(self._speed)
        time.sleep(self._move_time * steps)
        self._robot.stop()
