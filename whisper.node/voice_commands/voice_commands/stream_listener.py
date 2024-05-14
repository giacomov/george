import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


import subprocess
import re


class StreamListener(Node):

    def __init__(self, topic_name='voice_commands'):
        super().__init__('stream_listener')

        self._logger = self.create_publisher(String, "chatter", 10)

        self.log('Stream Listener node started')
        
        # Sync with the interpreter so we don't send new commands while 
        # the current one is being executed
        self.log(f"Listening to topic 'locks'")
        self._subscription = self.create_subscription(
            String,
            'locks',
            self.lock,
            10
        )

        self.log(f'Publishing to topic {topic_name}')
        self._publisher = self.create_publisher(String, topic_name, 10)

        self.srv = self.create_service(Trigger, 'whisper_ready', self.handle_ready_request)

        # Wait to be unlocked by the interpreter node
        self._locked = True

        # Start the _listen_stream method in a separate thread
        # to avoid running into the GIL and blocking the entire node
        self.stream_thread = threading.Thread(target=self._listen_stream)
        self.stream_thread.start()
    
    def handle_ready_request(self, request, response):
        self.log("Received request to check if we are ready")
        response.success = True
        response.message = 'Service is ready'
        return response

    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))
    
    def _listen_stream(self):
        
        process = subprocess.Popen(
            [
                '/whisper/stream', 
                '-c', '10', # microphone channel
                '-m', '/whisper/models/ggml-small.en-q5_1.bin', 
                '--step', '0',
                '--keep', '0',
                '--length', '3000',
                '--threads', '2'
            ], 
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True  # This is needed to get a string instead of bytes
        )

        while True:

            try:
                line = process.stdout.readline()
            except Exception as e:
                self.log(f"Error when reading line from stream: {e}. Will try continuing")
                continue
            
            if line == '' and process.poll() is not None:
                # Process is done
                remaining_output, remaining_errors = process.communicate()
                
                if remaining_output:
                
                    self.log(f"Stream ended with msg: {remaining_output.strip()}")
                
                if remaining_errors:

                    self.log(f"Stream ended with error: {remaining_errors.strip()}")
                
                return

            if line:

                match = re.search(r'\[\d\d:\d\d\.\d\d\d --> \d\d:\d\d\.\d\d\d\]\s+(.*)', line.strip())
            
                if match:
                    
                    message = match.group(1)
                    
                    if message.find("BLANK_AUDIO") >= 0:

                        self.log('No voice detected')

                    else:

                        if not self._locked:

                            self.log('Publishing: "%s"' % message)
                            self._publisher.publish(String(data=message))
                            
    def lock(self, msg):

        if msg.data == 'locked':
            # Pause process so we don't get any more inputs and we don't listen
            # to the noise of the motors while a command is being executed
            # The process will be unlocked when the interpreter node will
            # send a message to the locks topic
            self.log("Suspending stream")
            self._locked = True

        elif msg.data == 'unlocked':

            # This resumes the process so we can listen to new commands
            self.log("Resuming stream")
            self._locked = False
        
        else:
        
            self.log(f"Unknown message on locks: {msg.data}")


def main(args=None):

    rclpy.init(args=args)
    
    stream_listener = StreamListener()
    rclpy.spin(stream_listener)
    
    stream_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
