import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .navigation import Navigation


class Interpreter(Node):
    """
    Interprets and executes the commands received on the voice_commands topic.
    """
    def __init__(self):
        
        super().__init__('interpreter')

        self._logger = self.create_publisher(String, "chatter", 100)

        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.callback,
            10
        )

        self._navigation = Navigation()

        self.log('Interpreter node started')
    
    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))

    def callback(self, msg):
        
        self.log(f'Received: {msg.data}')
        
        for action in Navigation.get_available_actions():

            if action in msg.data:
            
                self.log(f"Executing: {action}")

                self._navigation.execute(action)

                break
        
        else:

            self.log("Action not recognized")


def main(args=None):
    rclpy.init(args=args)
    interpreter = Interpreter()
    rclpy.spin(interpreter)
    interpreter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
