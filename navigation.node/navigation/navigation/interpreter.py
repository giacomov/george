import concurrent.futures
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

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

        # Signal that we are alive
        self._navigation.execute('left')
        self._navigation.execute('right')
        
        # Now wait for the whisper node to be ready
        client = self.create_client(Trigger, 'whisper_ready')

        while True:
            
            if client.wait_for_service(timeout_sec=1.0):
            
                req = Trigger.Request()
            
                future = client.call_async(req)
            
                rclpy.spin_until_future_complete(self, future)
            
                if future.result() is not None and future.result().success:
            
                    self.log("Whisper node is ready")
            
                    break

                else:

                    time.sleep(0.5)
            
            self.log('Waiting for whisper node to be ready')

        
        self._display = Display()
        self._display.welcome()
        self._unlock_voice_commands()
    
    def handle_ready_request(self, request, response):

        response.success = True
        return response

    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))

    def callback(self, msg):

        self.log(f'Received: {msg.data}')
        
        self._lock_voice_commands()

        for action in self._available_actions:

            if action in msg.data.lower():
            
                self.log(f"Executing: {action}")
                self.perform_action(action)

                break
        
        else:

            self.log(f"Action not recognized. Available actions: {self._available_actions}")
            self._display.display_text("??")
        
        # Signal that we are ready to receive messages
        self._unlock_voice_commands()
    
    def _lock_voice_commands(self):
        self.log("Locking stream")
        self._locks.publish(String(data='locked'))
    
    def _unlock_voice_commands(self):
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
