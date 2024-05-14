import concurrent.futures
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .navigation import Navigation
from .display import Display


class Interpreter(Node):
    """
    Interprets and executes the commands received on the voice_commands topic.
    """
    def __init__(self):
        
        super().__init__('interpreter')

        self._logger = self.create_publisher(String, "chatter", 10)

        self._locks = self.create_publisher(String, "locks", 10)

        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.callback,
            10
        )

        self._navigation = Navigation()
        self._available_actions = Navigation.get_available_actions()

        self.log('Interpreter node started')

        # Signal that we are ready to receive messages
        self._navigation.execute('left')
        self._navigation.execute('right')

        self._display = Display()
        self._display.welcome()
    
    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))

    def callback(self, msg):

        self.log(f'Received: {msg.data}')
        
        self.log("Locking stream")
        self._locks.publish(String(data='locked'))

        for action in self._available_actions:

            if action in msg.data.lower():
            
                self.log(f"Executing: {action}")
                self.perform_action(action)

                break
        
        else:

            self.log(f"Action not recognized. Available actions: {self._available_actions}")
            self._display.display_text("??")
        
        # Signal that we are ready to receive messages
        self.log("Unlocking stream")
        self._locks.publish(String(data='unlocked'))
    
    def perform_action(self, action):
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Submit the display_text and execute methods to be run concurrently
            future_display = executor.submit(self._display.display_text, action)
            future_navigation = executor.submit(self._navigation.execute, action)

            # wait to complete the movement
            concurrent.futures.wait([future_navigation])

def main(args=None):
    rclpy.init(args=args)
    interpreter = Interpreter()
    rclpy.spin(interpreter)
    interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
