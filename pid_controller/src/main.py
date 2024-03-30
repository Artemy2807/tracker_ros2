#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_srvs.srv import SetBool

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Pose2D

from pid_dynamic import PID


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.pids = {
            'yaw': PID(self, 'yaw', enabled=True),
            'height': PID(self, 'height', enabled=True)
        }
        self.add_on_set_parameters_callback(self.on_params_changed)

        for pid_name in self.pids:
            self.pids[pid_name].init_params()

        self.running = False
        self.tracked_object = None
        self.forward_vel = 0.7

        self.create_subscription(Pose2D, '/tracked_object', self.object_callback, 1)
        self.twist_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 1)
        self.switch_service = self.create_service(SetBool, '~/enable', self.handle_switch)
        self.update_timer = self.create_timer(0.1, self.update)

        self.start_time = self.get_clock().now()

        self.get_logger().info('Pid control started!')

    def on_params_changed(self, params):
        for pid_name in self.pids:
            self.pids[pid_name].on_params_changed(params)
        return SetParametersResult(successful=True)

    def handle_switch(self, req, resp):
        self.running = req.data
        resp.success = True
        resp.message = 'control_node node %s' % (('disabled', 'enabled')[self.running])
        self.twist_publisher.publish(Twist())
        self.start_time = self.get_clock().now()
        return resp

    def object_callback(self, msg):
        self.tracked_object = msg

    def update(self):
        if not self.running:
            return

        if (self.get_clock().now() - self.start_time).nanoseconds * 1e-9 >= 60 * 5:
            self.running = False
            self.twist_publisher.publish(Twist())
            return

        cmd_world = Twist()
        if self.tracked_object is not None:
            cmd_world.linear.x = self.forward_vel
            cmd_world.linear.z = self.pids['height'].update(self.tracked_object.y)
            cmd_world.angular.z = self.pids['yaw'].update(self.tracked_object.x)
        self.twist_publisher.publish(cmd_world)

def main(args=None):
    rclpy.init(args=args)

    control_node = ControlNode()

    rclpy.spin(control_node)

    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
