#!/usr/bin/env python3

from blueye.sdk import Drone
try:
    import blueye.protocol as bp
    _HAVE_PROTOCOL = True
except ImportError:
    _HAVE_PROTOCOL = False

import math
import sys

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Imu, BatteryState, Temperature


class BlueyeTelemetry(Node):

    def __init__(self):
        super().__init__('blueye_telemetry')

        self.declare_parameter('rate',          10.0)
        self.declare_parameter('is_simulation', False)
        self.declare_parameter('drone_ip',      '')   # empty = SDK auto-discovery

        self.RATE          = self.get_parameter('rate').value
        self.IS_SIMULATION = self.get_parameter('is_simulation').value
        self.DRONE_IP      = self.get_parameter('drone_ip').value

        self._init_publishers()

        if not self.IS_SIMULATION:
            self._connect()

        self.timer = self.create_timer(1.0 / self.RATE, self._publish)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def _connect(self):
        try:
            if self.DRONE_IP:
                self.get_logger().info(f'Connecting to Blueye at {self.DRONE_IP} ...')
                self.drone = Drone(ip=self.DRONE_IP)
            else:
                self.get_logger().info('Connecting to Blueye via auto-discovery ...')
                self.drone = Drone()
            self.get_logger().info('Blueye drone connected.')
        except Exception as exc:
            self.get_logger().error(f'Could not connect to drone: {exc}')
            self.drone = None

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _init_publishers(self):
        self.depth_pub        = self.create_publisher(Float64,      'depth',             10)
        self.imu_pub          = self.create_publisher(Imu,          'imu',               10)
        self.battery_pub      = self.create_publisher(BatteryState, 'battery_state',     10)
        self.altitude_pub     = self.create_publisher(Float64,      'altitude',          10)
        self.water_temp_pub   = self.create_publisher(Temperature,  'water_temperature', 10)
        self.connected_pub    = self.create_publisher(Bool,         'connected_status',  10)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _now(self) -> Time:
        return self.get_clock().now().to_msg()

    @staticmethod
    def _euler_to_quaternion(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy,   # w
        )

    # ------------------------------------------------------------------
    # Publish loop
    # ------------------------------------------------------------------

    def _publish(self):
        if self.IS_SIMULATION or self.drone is None:
            return

        now = self._now()

        # --- depth (mm → m) -------------------------------------------
        try:
            depth_m = float(self.drone.depth) / 1000.0
            msg = Float64()
            msg.data = depth_m
            self.depth_pub.publish(msg)
        except Exception:
            pass

        # --- IMU ---------------------------------------------------------
        try:
            pose = self.drone.pose
            roll  = math.radians(float(list(pose.values())[0]))
            pitch = math.radians(float(list(pose.values())[1]))
            yaw   = math.radians(float(list(pose.values())[2]))
            qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = 'blueye_imu'

            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            # small diagonal covariance — orientation is filtered, not raw
            imu_msg.orientation_covariance = [
                1e-4, 0.0, 0.0,
                0.0, 1e-4, 0.0,
                0.0, 0.0, 1e-4,
            ]

            # Try to get raw IMU from blueye.protocol telemetry
            if _HAVE_PROTOCOL:
                try:
                    imu_tel = self.drone.telemetry.get_first(bp.ImuTel)
                    imu_msg.angular_velocity.x = float(imu_tel.imu.gyroscope.x)
                    imu_msg.angular_velocity.y = float(imu_tel.imu.gyroscope.y)
                    imu_msg.angular_velocity.z = float(imu_tel.imu.gyroscope.z)
                    imu_msg.angular_velocity_covariance = [
                        1e-3, 0.0, 0.0,
                        0.0, 1e-3, 0.0,
                        0.0, 0.0, 1e-3,
                    ]
                    imu_msg.linear_acceleration.x = float(imu_tel.imu.accelerometer.x)
                    imu_msg.linear_acceleration.y = float(imu_tel.imu.accelerometer.y)
                    imu_msg.linear_acceleration.z = float(imu_tel.imu.accelerometer.z)
                    imu_msg.linear_acceleration_covariance = [
                        1e-3, 0.0, 0.0,
                        0.0, 1e-3, 0.0,
                        0.0, 0.0, 1e-3,
                    ]
                except Exception:
                    imu_msg.angular_velocity_covariance[0] = -1.0
                    imu_msg.linear_acceleration_covariance[0] = -1.0
            else:
                imu_msg.angular_velocity_covariance[0] = -1.0
                imu_msg.linear_acceleration_covariance[0] = -1.0

            self.imu_pub.publish(imu_msg)
        except Exception:
            pass

        # --- battery -----------------------------------------------------
        try:
            bat_msg = BatteryState()
            bat_msg.header.stamp = now
            bat_msg.percentage = float(self.drone.battery_state_of_charge) / 100.0

            # Voltage and current available via protocol when possible
            if _HAVE_PROTOCOL:
                try:
                    bat_tel = self.drone.telemetry.get_first(bp.BatteryTel)
                    bat_msg.voltage = float(bat_tel.battery.voltage)
                    bat_msg.current = float(bat_tel.battery.current)
                except Exception:
                    bat_msg.voltage = float('nan')
                    bat_msg.current = float('nan')
            else:
                bat_msg.voltage = float('nan')
                bat_msg.current = float('nan')

            bat_msg.present = True
            self.battery_pub.publish(bat_msg)
        except Exception:
            pass

        # --- altitude from seabed (altimeter) ----------------------------
        if _HAVE_PROTOCOL:
            try:
                alt_tel = self.drone.telemetry.get_first(bp.AltimeterTel)
                alt_msg = Float64()
                # altimeter distance in metres
                alt_msg.data = float(alt_tel.altitude.distance)
                self.altitude_pub.publish(alt_msg)
            except Exception:
                pass

        # --- water temperature -------------------------------------------
        if _HAVE_PROTOCOL:
            try:
                temp_tel = self.drone.telemetry.get_first(bp.WaterTemperatureTel)
                temp_msg = Temperature()
                temp_msg.header.stamp = now
                temp_msg.temperature = float(temp_tel.temperature.value)
                temp_msg.variance = 0.0
                self.water_temp_pub.publish(temp_msg)
            except Exception:
                pass

        # --- connection status -------------------------------------------
        try:
            conn_msg = Bool()
            conn_msg.data = bool(self.drone.connection_established)
            self.connected_pub.publish(conn_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BlueyeTelemetry()
        rclpy.spin(node)
    except Exception as exc:
        print(f'blueye_telemetry exception: {exc}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()