#!/usr/bin/python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from pressure_sensor import ms5837

class PressurePublisher(Node):
    def __init__(self):
        super().__init__('pressure_pub')

        self.declare_parameter('sensor_type', 0)
        self.sensor_type = self.get_parameter('sensor_type').get_parameter_value().integer_value

        self.declare_parameter('i2c_bus', 1)
        self.i2c_bus = self.get_parameter('i2c_bus').get_parameter_value().integer_value

        self.declare_parameter('frame_id', 'pressure_link')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(FluidPressure, 'pressure/data', 10)

        self._last_pressure_pa = None

        self._init_sensor()

        self.declare_parameter('publish_frequency', 10.0)
        freq = self.get_parameter('publish_frequency').get_parameter_value().double_value
        freq = max(min(freq, 20.0), 0.1) # Limit frequency to between 0.1 Hz and 20 Hz
        period = 1.0 / freq
        self.get_logger().info(f"Publishing pressure data at {1/period} Hz (period: {period} s)")

        self.timer = self.create_timer(period, self.timer_callback)

    def _init_sensor(self):
        """Initialize the sensor, retry on failure."""
        try:
            if self.sensor_type == 0:
                self.sensor = ms5837.MS5837_02BA(bus=self.i2c_bus)
                self.get_logger().info("Using MS5837_02BA sensor (shallow)")
            else:
                self.sensor = ms5837.MS5837_30BA(bus=self.i2c_bus)
                self.get_logger().info("Using MS5837_30BA sensor (deep)")

            if not self.sensor.init():
                self.get_logger().warn("Sensor init failed, retrying in 1s...")
                time.sleep(1)
                self._init_sensor()
        except OSError as e:
            self.get_logger().warn(f"I2C init error: {e}, retrying in 1s...")
            time.sleep(1)
            self._init_sensor()

    def timer_callback(self):
        try:
            if self.sensor.read():
                pressure_pa = self.sensor.pressure() * 100  # mbar to Pa
                msg = FluidPressure()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.fluid_pressure = pressure_pa
                msg.variance = 0.0
                if self._last_pressure_pa is not None and pressure_pa == self._last_pressure_pa:
                    self.get_logger().debug("Pressure value identical to previous reading; skipping publish")
                    self.get_logger().debug("Is the freqeuency too high or is the sensor frozen?")
                    return

                self._last_pressure_pa = pressure_pa
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("Sensor read returned False, reinitializing sensor")
                self._init_sensor()
        except OSError as e:
            self.get_logger().warn(f"I2C read error: {e}, resetting sensor")
            try:
                self.sensor._bus.close()  # close old bus
            except Exception:
                pass
            time.sleep(0.1)
            self._init_sensor()


def main(args=None):
    rclpy.init(args=args)
    node = PressurePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()