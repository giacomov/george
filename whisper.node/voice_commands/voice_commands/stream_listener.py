import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re


class StreamListener(Node):

    def __init__(self, topic_name='voice_commands'):
        super().__init__('stream_listener')

        self._logger = self.create_publisher(String, "chatter", 100)

        self.log('Stream Listener node started')

        self.log(f'Publishing to topic {topic_name}')
        self._publisher = self.create_publisher(String, topic_name, 10)
        self._timer = self.create_timer(0.1, self.listen_stream)  # Timer to check the stream

        self.log('Running whisper stream')
        self._process = subprocess.Popen(
            [
                '/whisper/stream', 
                '-c', '10', 
                '-m', '/whisper/models/ggml-base.en.bin', 
                '--step', '0', 
                '--length', '3000'
            ], 
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True  # This is needed to get a string instead of bytes
        )

        # Sync with the interpreter so we don't send new commands while 
        # the current one is being executed
        self._subscription = self.create_subscription(
            String,
            'locks',
            self.lock_callback,
            10
        )
    
    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))
    
    def listen_stream(self):

        try:
            self._listen_stream()
        except Exception as e:
            self.log(f'Error when reading stream: {e}')
    
    def _listen_stream(self):
        
        line = self._process.stdout.readline()
        
        if line:

            # Pause process so we don't get any more inputs and we don't listen
            # to the noise of the motors while a command is being executed
            # The process will be unlocked when the interpreter node will
            # send a message to the locks topic
            self.log("Suspending stream")
            self._process.send_signal(subprocess.signal.SIGSTOP)

            match = re.search(r'\[\d\d:\d\d\.\d\d\d --> \d\d:\d\d\.\d\d\d\]\s+(.*)', line.strip())
        
            if match:
                
                message = match.group(1)
                
                if message.find("BLANK_AUDIO") >= 0:

                    self.log('No voice detected')
                    
                    return
                
                self.log('Publishing: "%s"' % message)
                self._publisher.publish(String(data=message))
                
        else:
            self.log('No line received from stream')
    
    def lock_callback(self):
        # This resumes the process so we can listen to new commands
        self.log("Resuming stream")
        self._process.send_signal(subprocess.signal.SIGCONT)


def main(args=None):

    rclpy.init(args=args)
    
    stream_listener = StreamListener()
    rclpy.spin(stream_listener)
    
    stream_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
