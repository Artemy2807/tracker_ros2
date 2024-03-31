#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from typing import Tuple

class StreamNode(Node):
    def __init__(self, callback, bridge: CvBridge):
        super().__init__('stream_node')

        self.declare_parameter('image_topic', '/simple_drone/front/image_raw')

        self.source = self.get_parameter('image_topic').value
        self.callback = callback
        self.bridge = bridge

        self.image_sub = self.create_subscription(
            Image, 
            self.source, 
            self.callback, 
            1
        )

class TrackerNode(StreamNode):
    def __init__(self):
        super().__init__(
            callback = self.callback, 
            bridge = CvBridge()
        )
        self.init_params()

        self.old_points = None
        self.old_frame = None

        self.parameter_lucas_kanade = dict(
                                        winSize=(15, 15), 
                                        maxLevel=2, 
                                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 0.03)
                                      )

        cv2.namedWindow("Tracker Debug")
        cv2.setMouseCallback("Tracker Debug", self.select_point)

        self.object_publisher = self.create_publisher(
            Pose2D,
            self.get_parameter('output_topic').value, 
            1
        )

        self.yaw_setpoint_publisher = self.create_publisher(Float64, '/control_node/yaw_pid/setpoint', 1)
        self.height_setpoint_publisher = self.create_publisher(Float64, '/control_node/height_pid/setpoint', 1)

        self.enable_pid('/control_node/enable')

        self.get_logger().info('Tracker node started!')

    def init_params(self):
        self.declare_parameter('output_topic', '/tracked_object')

    def enable_pid(self, name: str):
        cli = self.create_client(SetBool, name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        # Enable control node...
        req = SetBool.Request()
        req.data = True
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def select_point(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.old_points = np.array([[x, y]], dtype=np.float32)

    def callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # setpoints...
        msg = Float64()
        msg.data = image.shape[1] / 2.0
        self.yaw_setpoint_publisher.publish(msg)
        msg.data = image.shape[0] / 2.0
        self.height_setpoint_publisher.publish(msg)

        self.frame = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if self.old_frame is None:
            self.old_frame = self.frame.copy()

        if self.old_points is not None:
            new_points, status, errors = cv2.calcOpticalFlowPyrLK(
                                                self.old_frame, 
                                                self.frame, 
                                                self.old_points, 
                                                None,
                                                **self.parameter_lucas_kanade
                                         )

            self.old_frame = self.frame.copy()
            self.old_points = new_points

            x, y = new_points.ravel()
            image = cv2.circle(image, (int(x), int(y)), 5, (0, 255, 0), -1)

            obj = Pose2D()
            obj.x = float(x)
            obj.y = float(y)
            self.object_publisher.publish(obj)

        cv2.imshow("Tracker Debug", image)
        cv2.waitKey(5)


def main(args=None):
    rclpy.init(args=args)

    tracker_node = TrackerNode()

    rclpy.spin(tracker_node)

    tracker_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()