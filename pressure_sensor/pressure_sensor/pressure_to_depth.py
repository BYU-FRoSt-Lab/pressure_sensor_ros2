#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry

from std_srvs.srv import Trigger

import numpy as np

GRAVITY = 9.81  # m/s^2


class DepthConverter(Node):
    def __init__(self):
        super().__init__('depth_converter')

        # -------------------------------
        # Parameters
        # -------------------------------
        # Either set fluid_density directly OR leave it unset and compute
        # density from salinity using the simple linear model.
        self.declare_parameter('fluid_density', 997.0)          # kg/m^3
        self.declare_parameter('water_salinity_ppt', 0.0)       # ppt
        self.declare_parameter('use_salinity_model', False)     # bool

        self.declare_parameter('fluid_pressure_atm', 87250.0)   # Pa
        self.declare_parameter('calibration_samples', 20)
        self.declare_parameter('calibration_timeout_s', 5.0)

        self.average_pressure = self.get_parameter(
            'fluid_pressure_atm').value

        # -------------------------------
        # ROS interfaces
        # -------------------------------
        self.depth_pub = self.create_publisher(
            Odometry,
            'depth_data',
            10
        )

        self.pressure_sub = self.create_subscription(
            FluidPressure,
            'pressure/data',
            self.pressure_callback,
            qos_profile_sensor_data
        )

        self.calibrate_srv = self.create_service(
            Trigger,
            'calibrate_depth',
            self.handle_calibration_request
        )

        self.calibration_timer = self.create_timer(
            0.5,
            self.calibration_timer_callback
        )
        self.calibration_timer.cancel()

        # -------------------------------
        # Calibration state
        # -------------------------------
        self.calibrating = False
        self.pressure_samples = []
        self.calibration_start_time = None

        # Optional: auto-calibrate on startup
        self.start_calibration()

    # -------------------------------
    # Callbacks
    # -------------------------------

    def pressure_callback(self, msg: FluidPressure):
        density = self.get_fluid_density()

        depth = (self.average_pressure - msg.fluid_pressure) / (
            density * GRAVITY
        )

        depth_msg = Odometry()
        depth_msg.header.stamp = msg.header.stamp
        depth_msg.pose.pose.position.z = depth

        self.depth_pub.publish(depth_msg)

        if self.calibrating:
            self.pressure_samples.append(msg.fluid_pressure)

    def handle_calibration_request(self, request, response):
        self.start_calibration()
        response.success = True
        response.message = 'Calibration started'
        return response

    def calibration_timer_callback(self):
        if not self.calibrating:
            return

        elapsed = (self.get_clock().now() -
                   self.calibration_start_time).nanoseconds * 1e-9

        max_samples = self.get_parameter(
            'calibration_samples').value
        timeout = self.get_parameter(
            'calibration_timeout_s').value

        if len(self.pressure_samples) >= max_samples or elapsed >= timeout:
            self.finish_calibration()

    # -------------------------------
    # Helpers
    # -------------------------------

    def get_fluid_density(self) -> float:
        if self.get_parameter('use_salinity_model').value:
            salinity = self.get_parameter('water_salinity_ppt').value
            return 997.0 + 0.8 * salinity
        else:
            return self.get_parameter('fluid_density').value

    # -------------------------------
    # Calibration logic
    # -------------------------------

    def start_calibration(self):
        self.get_logger().info('Starting pressure calibration')
        self.pressure_samples.clear()
        self.calibrating = True
        self.calibration_start_time = self.get_clock().now()
        self.calibration_timer.reset()

    def finish_calibration(self):
        self.calibration_timer.cancel()
        self.calibrating = False

        if len(self.pressure_samples) < 5:
            self.get_logger().warn(
                'Calibration failed: insufficient samples'
            )
            return

        samples = np.array(self.pressure_samples)
        mean_pressure = float(np.mean(samples))
        variance = float(np.var(samples))

        if variance > 100.0:  # Pa^2
            self.get_logger().warn(
                f'Calibration failed: high variance ({variance:.2f} Pa^2)'
            )
            return
        
        if abs(mean_pressure - self.average_pressure) > 40000.0:  # 
            self.get_logger().warn(
                'Calibration failed: mean pressure change too large'
            )
            return

        old_pressure = self.average_pressure
        self.average_pressure = mean_pressure

        self.set_parameters([
            rclpy.parameter.Parameter(
                'fluid_pressure_atm',
                rclpy.parameter.Parameter.Type.DOUBLE,
                mean_pressure
            )
        ])

        self.get_logger().info(
            f'Calibration complete | '
            f'offset={mean_pressure - old_pressure:.2f} Pa | '
            f'variance={variance:.2f} Pa^2'
        )


def main():
    rclpy.init()
    node = DepthConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
