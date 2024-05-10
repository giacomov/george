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
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(0.1, self.listen_stream)  # Timer to check the stream

        self.log('Running whisper stream')
        self.process = subprocess.Popen(
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
    
    def log(self, msg):
        self._logger.publish(String(data=f"{self.get_name()}: {msg}"))

    def listen_stream(self):
        
        line = self.process.stdout.readline()
        
        if line:

            self.log(line)

            match = re.search(r'\[\d\d:\d\d\.\d\d\d --> \d\d:\d\d\.\d\d\d\]\s+(.*)', line.strip())
        
            if match:
                
                message = match.group(1)
                
                if message.find("BLANK_AUDIO") >= 0:

                    self.log('No voice detected')
                    
                    return
                
                self.log('Publishing: "%s"' % message)
                self.publisher_.publish(String(data=message))
                
        else:
            self.log('No line received from stream')


def main(args=None):

    rclpy.init(args=args)
    
    stream_listener = StreamListener()
    rclpy.spin(stream_listener)
    
    stream_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
