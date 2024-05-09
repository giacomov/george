import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re

_RECOGNIZED_COMMANDS = [
    "left",
    "right",
    "forward",
    "backwards",
    "stop"
]

class Interpreter(Node):
    """
    Interprets and executes the commands received on the voice_commands topic.
    """
    def __init__(self):
        
        super().__init__('interpreter')

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10
        )

        self.get_logger().info('Interpreter node started')
        
    def callback(self, msg):
        
        self.get_logger().info(f'Received: {msg.data}')
        
        for cmd in _RECOGNIZED_COMMANDS:

            if cmd in msg.data:
            
                self.get_logger().info(f"Executing: {cmd}")
            
                break
        
        else:

            self.get_logger().info("Command not recognized")


def main(args=None):
    rclpy.init(args=args)
    interpreter = Interpreter()
    rclpy.spin(interpreter)
    interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
