import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re

class StreamListener(Node):
    def __init__(self, topic_name='chatter'):
        super().__init__('stream_listener')

        self.get_logger().info('Stream Listener node started')

        self.get_logger().info(f'Publishing to topic {topic_name}')
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(0.1, self.listen_stream)  # Timer to check the stream

        self.get_logger().info('Running whisper stream')
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
        self.get_logger().info('Checking stream')
        line = self.process.stdout.readline()
        if line:
            match = re.search(r'\[\d\d:\d\d\.\d\d\d --> \d\d:\d\d\.\d\d\d\]\s+(.*)', line)
            if match:
                message = match.group(1)
                self.publisher_.publish(String(data=message))
                self.get_logger().info('Publishing: "%s"' % message)
            else:
                self.get_logger().info('No match: "%s"' % line)
        else:
            self.get_logger().info('No line received from stream')

def main(args=None):
    rclpy.init(args=args)
    stream_listener = StreamListener()
    rclpy.spin(stream_listener)
    stream_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
