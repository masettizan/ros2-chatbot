import rclpy
from rclpy.node import Node
import sys

from std_msgs.msg import String

class UserTextPublisher(Node):

        def __init__(self):
                super().__init__('user_text_publisher')
                self.publisher_ = self.create_publisher(String, 'text_input', 10)
                self.timer = self.create_timer(1.0, self.timer_callback)

        def timer_callback(self):
                while rclpy.ok(): # checks if ROS client library is still initialized
                        userText = input("Type your message here: ")
                        self.textCallback(userText)

        def textCallback(self, userText):
                msg = String()
                msg.data = userText
                self.publisher_.publish(msg)
                self.get_logger().info('Message sent: "%s"' % msg.data)

def main(args=None):
        rclpy.init(args=args)

        user_text_publisher = UserTextPublisher()

        rclpy.spin(user_text_publisher)

        user_text_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
        main()
