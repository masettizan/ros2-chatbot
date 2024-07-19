import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from audio_common_msgs.action import TTS
import requests
import openai
from openai.error import RateLimitError
import time
import logging
import anthropic
import os

from std_msgs.msg import String

class UserTextListener(Node):

    def __init__(self):
        super().__init__('user_text_listener')
        self.subscription = self.create_subscription(
            String,
            'text_input',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(String, 'text_output', 10)

        # parameters
        usedAI_param = "usedAI"

        self.declare_parameter(usedAI_param, "openai") # can be "anthropic" or "openai"

        self.usedAI = self.get_parameter(
            usedAI_param).get_parameter_value().string_value
        self.get_logger().info("Using AI " + self.usedAI + ".")

        print("Listening...")
        self._action_client = ActionClient(self, TTS, '/say')
        # /say is the action that we're using within the TTS action in audio_common_msgs
        self.personality = "Reply to everything with a funny tone. You should be making unnecessary jokes. These jokes can be questions, sarcastic comments, etc."

    def send_goal(self, text):
        goal_msg = TTS.Goal()
        goal_msg.text = text # from TTS.action file

        try:
            self.get_logger().info("Waiting for an action server...")
            self._action_client.wait_for_server()

            # Feedback handling
            print('Sending goal: "%s"' % text)

            return self._action_client.send_goal_async(goal_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to connect to action server: {str(e)}")

    def produceResponse(self, query):

        #API Key and URL handling
        API_KEY = None
        URL = None
        try:
            if self.usedAI == "anthropic":
                API_KEY = os.getenv('ANTHROPIC_API_KEY')
                URL = 'https://api.anthropic.com/v1/messages'
            elif self.usedAI == "openai":
                API_KEY = os.getenv('OPENAI_API_KEY')
                URL = 'https://api.openai.com/v1/engines/davinci-002/completions'
        except Exception as e: 
            self.get_logger().error("No such API key exists.")

        max_retries = 5
        retry_delay = 1
        logging.basicConfig(level=logging.INFO)

        for attempt in range(max_retries):
            try:
                logging.info(f"Making API request attempt {attempt + 1}")
                logging.info(f"Query: {query}")
                response_data = None

                if self.usedAI == "anthropic":
                    client = anthropic.Anthropic(
                        api_key = API_KEY,
                    )

                    headers = {
                        'x-api-key': API_KEY,
                        'Content-Type': 'application/json',
                        'anthropic-version' : '2023-06-01'
                    }

                    message = client.messages.create(
                        model="claude-3-5-sonnet-20240620",
                        max_tokens=1024,
                        system=self.personality,
                        messages=[
                            {'role': 'user', 'content': query}
                        ]
                    )
                    
                    response_data = message.content[0].text
                    logging.info(f"Received API response: {response_data}")
                elif self.usedAI == "openai":

                    response = openai.ChatCompletion.create(
                        model="gpt-3.5-turbo",
                        messages=[
                            {"role": "system", "content": self.personality},
                            {"role": "user", "content": query}
                        ],
                        max_tokens=1024,
                        temperature=0.7
                    )

                    response_data = response.choices[0].message['content'].strip()
                    logging.info(f"Received API response: {response_data}")

                return response_data.replace("'", "")
            except RateLimitError:
                if attempt < max_retries - 1:
                    print(f"Rate limit exceeded. Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                    retry_delay *= 2  # exponential backoff
                else:
                    return("Could not process input. Please try again later.")

    def listener_callback(self, msg):
        # self.get_logger().info('User said: "%s"' % msg.data)
        chat_bot_response = self.produceResponse(msg.data)
        self.get_logger().info('[USER]: "%s"' % msg.data)

        self.send_goal(chat_bot_response)

        chat_msg = String()
        chat_msg.data = chat_bot_response
        self.publisher_.publish(chat_msg)

def main(args=None):
    rclpy.init(args=args)

    user_text_listener = UserTextListener()

    rclpy.spin(user_text_listener)

    user_text_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

