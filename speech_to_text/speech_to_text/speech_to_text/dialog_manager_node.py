""" ROS2 Node for Dialog Manger """

import time
import rclpy
from simple_node import Node

from speech_to_text_msgs.msg import StringArray
from speech_to_text_msgs.action import ListenOnce
from std_srvs.srv import Empty
from std_msgs.msg import String

class DialogManagerNode(Node):
    """ Dialog Manager Node Class """

    def __init__(self) -> None:
        super().__init__("dialog_manager_node")

        self.is_new_msg = False
        self.new_msg = None

        # service clients
        self.__start_listening_client = self.create_client(
            Empty, "start_listening")
        self.__stop_listening_client = self.create_client(
            Empty, "stop_listening")
        self.__calibrating_client = self.create_client(
            Empty, "calibrate_listening")

        # pubs and subs
        self.__pub = self.create_publisher(String, "stt_dialog", 10) #CHANGED FROM StringArray

        self.__subscription = self.create_subscription(
            String, #CHANGED FROM StringArray
            "/speech_to_text/stt", #CHANGED THIS FROM stt_parse
            self.__stt_callback,
            10
        )

        # action server
        self.__action_server = self.create_action_server(
            ListenOnce,
            "listen_once",
            execute_callback=self.__execute_server
        )

    def __stt_callback(self, msg: String) -> None:  #changed msg: StringArray to msg:String
        """ final speech calback

        Args:
            msg (StringArray): list of tags
        """
        self.get_logger().info("Dialog Manager CALLBACK: " + str(msg.data)) #CHANGED FROM msg.strings
        self.get_logger().info("before: at stt_cb in dialog manager, new msg status: " + str(self.is_new_msg))
        #self.__pub.publish(msg.data)
        self.get_logger().info("Published from dialog_manager.")
        self.get_logger().info("At stt_cb in dialog_manager, new message status: " + str(self.is_new_msg))


        if not self.is_new_msg:
            self.get_logger().info("new message set")
            self.new_msg.data = msg.data # changed self.new_msg to self.new_msg.data
            self.is_new_msg = True

    def calibrate_stt(self) -> None:
        """ calibrate stt method """

        req = Empty.Request()
        self.__calibrating_client.wait_for_service()
        self.__calibrating_client.call(req)
        self.get_logger().info("calibrating stt")
        self.get_logger().info("At calib_stt in dialog_manager, new message status: " + str(self.is_new_msg))

    def start_stt(self) -> None:
        """ start stt method """

        req = Empty.Request()
        self.__start_listening_client.wait_for_service()
        self.__start_listening_client.call(req)
        self.get_logger().info("starting stt")
        self.get_logger().info("At start_stt in dialog manager, new message status: " + str(self.is_new_msg))

    def stop_stt(self) -> None:
        """ stop stt method """

        req = Empty.Request()
        self.__stop_listening_client.wait_for_service()
        self.__stop_listening_client.call(req)
        self.get_logger().info("stopping stt")
        self.get_logger().info("At stop_stt in dialog manager, new message status: " + str(self.is_new_msg))

    def __execute_server(self, goal_handle) -> ListenOnce.Result:
        """ action server execute callback

        Args:
            goal_handle ([type]): goal_handle

        Returns:
            ListenOnce.Result: action server result (list of tags)
        """

        self.is_new_msg = False
        self.new_msg = String() #CHANGED FROM StringArray

        if goal_handle.request.calibrate:
            self.calibrate_stt()

        # starting stt
        self.start_stt()

        # wait for message
        while (not self.is_new_msg and not goal_handle.is_cancel_requested):
            self.get_logger().info("Waiting for msg")
            time.sleep(0.5)

        # stoping stt
        self.stop_stt()

        # results
        result = ListenOnce.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info("Goal canceled.")
        else:
            result.stt_strings = [self.new_msg.data] #changed from new_msg.strings and added brackets
            goal_handle.succeed()
            self.get_logger().info("Goal succeeded.")
        return result


def main():
    rclpy.init()
    node = DialogManagerNode()
    node.join_spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
