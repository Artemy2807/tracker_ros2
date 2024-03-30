#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64
from std_srvs.srv import SetBool

class PID:
    def __init__(self, node, name, enabled=False):
        self.node = node
        self.ns = f'~/{name}_pid'
        self.name = name
        self.enabled = enabled
        self.last_update_time = self.node.get_clock().now()

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0

        self.max_effort = 0.0
        self.angular = False
        self.inverted = False
        self.timeout = 1

        self.setpoint = 0.0
        self.e_sum = 0.0
        self.e_prev = 0.0
        self.effort = 0.0

        self.switch_service = self.node.create_service(SetBool, f'{self.ns}/switch', self.handle_switch_service)
        self.node.create_subscription(Float64, f'{self.ns}/setpoint', self.setpoint_callback, 1)

    def init_params(self):
        self.node.declare_parameter(f'{self.name}.kp', 0.0)
        self.node.declare_parameter(f'{self.name}.ki', 0.0)
        self.node.declare_parameter(f'{self.name}.kd', 0.0)
        self.node.declare_parameter(f'{self.name}.max_effort', 0.0)
        self.node.declare_parameter(f'{self.name}.angular', False)
        self.node.declare_parameter(f'{self.name}.inverted', False)
        self.node.declare_parameter(f'{self.name}.timeout', 1.0)

        param = self.node.get_parameter(f'{self.name}.kp')
        self.kp = param.value

        param = self.node.get_parameter(f'{self.name}.ki')
        self.ki = param.value

        param = self.node.get_parameter(f'{self.name}.kd')
        self.kd = param.value

        param = self.node.get_parameter(f'{self.name}.max_effort')
        self.max_effort = param.value

        param = self.node.get_parameter(f'{self.name}.angular')
        self.angular = param.value

        param = self.node.get_parameter(f'{self.name}.inverted')
        self.inverted = param.value

        param = self.node.get_parameter(f'{self.name}.timeout')
        self.timeout = param.value

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            if param.name == f'{self.name}.kp':
                self.kp = param.value
            elif param.name == f'{self.name}.ki':
                self.ki = param.value
            elif param.name == f'{self.name}.kd':
                self.kd = param.value
            elif param.name == f'{self.name}.max_effort':
                self.max_effort = param.value
            elif param.name == f'{self.name}.angular':
                self.angular = param.value
            elif param.name == f'{self.name}.inverted':
                self.inverted = param.value
            elif param.name == f'{self.name}.timeout':
                self.timeout = param.value
            else:
                continue

    def handle_switch_service(self, req, resp):
        self.switch(req.data)
        resp.success = True
        resp.message = f"{self.ns} {('disabled', 'enabled')[self.enabled]}"
        return resp

    def setpoint_callback(self, msg):
        self.setpoint = msg.data

    def switch(self, state):
        self.enabled = state
        self.reset()

    def reset(self):
        self.e_sum = 0.0
        self.e_prev = 0.0
        self.effort = 0.0

    def angular_constraint(self, e):
        return (e + 180) % 360 - 180

    def clamp(self, v):
        return min(max(v, -self.max_effort), self.max_effort)

    def update(self, state):
        if not self.enabled:
            return 0.0

        self.last_update_time = self.node.get_clock().now()

        error = self.setpoint - state if not self.inverted else state - self.setpoint
        if self.angular:
            error = self.angular_constraint(error)
        p = self.kp * error
        self.e_sum = self.clamp(self.e_sum + self.ki * error)
        d = self.kd * (error - self.e_prev)
        self.e_prev = error
        self.effort = self.clamp(p + self.e_sum + d)

        return self.effort

    def get_effort(self):
        if (self.node.get_clock().now() - self.last_update_time).nanoseconds * 1e-9 > self.timeout:
            self.reset()
        return self.effort
