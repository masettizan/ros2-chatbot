import rclpy
from rclpy.node import Node
import sys
import io
from rclpy.action import ActionClient
from speech_to_text_msgs.action import ListenOnce
import subprocess
import time

from std_msgs.msg import String

'''
Node used for listening to user input, receiving it as text, processing it to be sent to the
chatbot for a response, and repeating the process after the TTS is finished speaking.
'''
class DictionConverter(Node):

    # initializes the text_input publisher to be sent to the listener node, the action client to enable
    # user diction, the stt subscription to receive the input as text, and the text_output subscription
    # to handle waiting for the TTS to finish
    def __init__(self):
        super().__init__('user_diction_converter')
        self.publisher_ = self.create_publisher(String, "text_input", 10)
        self._action_client = ActionClient(self, ListenOnce, '/speech_to_text/listen_once')
        self.subscription = self.create_subscription(String, '/speech_to_text/stt', self.textCallback, 10)
        self.subscription
        self.listener_subscription = self.create_subscription(String, 'text_output', self.listenerCallback, 10)
        self.listener_subscription

        self.send_goal()
        self.get_logger().info("Initialized publisher, subscriber, and action client.")
        self.get_logger().info("Listening...")

    # asks the action server to listen once to the user and return their input as text
    def send_goal(self):
        goal_msg = ListenOnce.Goal()
        goal_msg.calibrate = True

        try:
            self.get_logger().info("Waiting for an action server...")
            self._action_client.wait_for_server()

            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to action server: {str(e)}")

    # determines if the goal was processed properly
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # handles goal feedback
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.stt_strings))
        self.get_logger().info("Finished sending goal.")

    # uses the response from the AI to determine how long to wait to receive new user input
    def listenerCallback(self, userMsg):
        wordCount = len(userMsg.data.split())
        self.get_logger().info("Sending new goal in " + str(wordCount) + " seconds.")
        time.sleep(wordCount * 0.35) # approximately 0.35 seconds per word
        self.send_goal()

    # publishes the user input as text to the listener to be processed by the AI
    def textCallback(self, userText):
        self.get_logger().info('Text callback: "%s"' % userText)
        msg = userText
        self.get_logger().info('Resulting string: "%s"' % msg.data)
        if msg.data.strip():
            if not msg.data == "UnknownValueError":
                self.publisher_.publish(msg)
                print('Message sent: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    user_diction_converter = DictionConverter()

    rclpy.spin(user_diction_converter)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
