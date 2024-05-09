import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re

class StreamListener(Node):
    def __init__(self):
        super().__init__('stream_listener')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.1, self.listen_stream)  # Timer to check the stream
        self.process = subprocess.Popen(
            [
                '/whisper/stream', 
                '-c', '10', 
                '-m', '/whisper/models/ggml-tiny.en.bin', 
                '--step', '0', 
                '--length', '3000'
            ], 
            stdout=subprocess.PIPE, 
            universal_newlines=True  # This is needed to get a string instead of bytes
        )

    def listen_stream(self):
        line = self.process.stdout.readline()
        if line:
            match = re.search(r'\[\d\d:\d\d\.\d\d\d --> \d\d:\d\d\.\d\d\d\]\s+(.*)', line)
            if match:
                message = match.group(1)
                self.publisher_.publish(String(data=message))
                self.get_logger().info('Publishing: "%s"' % message)
            else:
                self.get_logger().info('No match: "%s"' % line)

def main(args=None):
    rclpy.init(args=args)
    stream_listener = StreamListener()
    rclpy.spin(stream_listener)
    stream_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
