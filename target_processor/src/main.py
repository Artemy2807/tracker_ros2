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

from enum import Enum
from typing import Tuple

class StreamNode(Node):
    class SourceType(Enum):
        File = 0
        Topic = 1

    def __init__(self, callback, bridge: CvBridge, source: str, type: SourceType):
        super().__init__('stream_node')

        self.source = source
        self.type = type
        self.callback = callback
        self.bridge = bridge

        if self.type == self.SourceType.File:
            self.cap = cv2.VideoCapture(self.source)
            if not self.cap:
                raise Exception(f"Couldn`t open video file {self.source}")
            else:
                print(f"Successfully open video file {self.source}")

            # Тестовые значения
            self.cm = np.array([[ 92.37552066, 0., 160.5], [0., 92.37552066, 120.5], [0., 0., 1.]], dtype="float64")
            self.dc = np.zeros(5, dtype="float64")

            rclpy.Timer(rclpy.Duration(1/30), self.video_callback)
        else:
            self.cm, self.dc = self.camera_cfg_cvt(
                wait_for_message(
                    msg_type=CameraInfo, 
                    node=self,
                    topic="/simple_drone/front/camera_info"
                )[-1]
            )
            self.image_sub = self.create_subscription(Image, self.source, self.callback, 1)

    def camera_cfg_cvt(self, msg: CameraInfo) -> Tuple[np.ndarray, np.ndarray]:
        return (np.reshape(np.array(msg.k, dtype="float64"), (3, 3)), np.array(msg.d, dtype="float64"))

    def video_callback(self, event):
        ret, frame = self.cap.read()

        if not ret:
            raise Exception("End of video file")

        self.callback(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

class TrackerNode(StreamNode):
    def __init__(self, source: str = "/simple_drone/front/image_raw", type: StreamNode.SourceType = StreamNode.SourceType.Topic):
        super().__init__(
            callback = self.callback, 
            bridge = CvBridge(),
            source = source,
            type = type
        )

        self.old_points = None
        self.old_frame = None

        self.parameter_lucas_kanade = dict(
                                        winSize=(15, 15), 
                                        maxLevel=2, 
                                        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 100, 0.03)
                                      )

        cv2.namedWindow("Tracker Debug")
        cv2.setMouseCallback("Tracker Debug", self.select_point)

        self.object_publisher = self.create_publisher(Pose2D, '/tracked_object', 1)
        self.yaw_setpoint_publisher = self.create_publisher(Float64, '/control_node/yaw_pid/setpoint', 1)
        self.height_setpoint_publisher = self.create_publisher(Float64, '/control_node/height_pid/setpoint', 1)

        self.enable_pid('/control_node/enable')

        self.get_logger().info('Tracker node started!')

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