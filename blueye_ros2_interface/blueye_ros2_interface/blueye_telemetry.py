#!/usr/bin/env python3

from blueye.sdk import Drone
try:
    import blueye.protocol as bp
    _HAVE_PROTOCOL = True
except ImportError:
    _HAVE_PROTOCOL = False

import dataclasses
import math

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from std_msgs.msg import Float64, String
from sensor_msgs.msg import Imu, BatteryState, Temperature
from geometry_msgs.msg import WrenchStamped, Vector3Stamped


class BlueyeTelemetry(Node):

    def __init__(self):
        super().__init__('blueye_telemetry')

        self.declare_parameter('rate',          10.0)
        self.declare_parameter('is_simulation', False)
        self.declare_parameter('drone_ip',      '')

        self.RATE          = self.get_parameter('rate').value
        self.IS_SIMULATION = self.get_parameter('is_simulation').value
        self.DRONE_IP      = self.get_parameter('drone_ip').value
        self._tick         = 0

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
            self._log_startup_info()
        except Exception as exc:
            self.get_logger().error(f'Could not connect to drone: {exc}')
            self.drone = None

    def _log_startup_info(self):
        if not _HAVE_PROTOCOL:
            return

        # Drone identity
        try:
            tel = self.drone.telemetry.get(bp.DroneInfoTel)
            if tel is not None:
                di = tel.drone_info
                self.get_logger().info(
                    f'Drone model: {di.model}, serial: {di.serial_number}, '
                    f'firmware: {di.blunux_version}'
                )
        except Exception:
            pass

        # Storage
        try:
            tel = self.drone.telemetry.get(bp.DataStorageSpaceTel)
            if tel is not None:
                ss = tel.storage_space
                free_gb  = ss.free_space  / 1e9
                total_gb = ss.total_space / 1e9
                self.get_logger().info(
                    f'Storage: {free_gb:.1f} GB free / {total_gb:.1f} GB total'
                )
        except Exception:
            pass

        # Connected clients
        try:
            tel = self.drone.telemetry.get(bp.ConnectedClientsTel)
            if tel is not None:
                self.get_logger().info(
                    f'Connected clients: controlling client id: {tel.client_id_in_control}'
                )
        except Exception:
            pass

        # Calibration state
        try:
            tel = self.drone.telemetry.get(bp.CalibrationStateTel)
            if tel is not None:
                cs = tel.calibration_state
                self.get_logger().info(
                    f'Calibration state: {int(cs.status)}'
                )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _init_publishers(self):
        self.depth_pub             = self.create_publisher(Float64,       'depth',                   10)
        self.imu_calibrated_pub    = self.create_publisher(Imu,           'imu_calibrated',          10)
        self.imu1_pub              = self.create_publisher(Imu,           'imu1',                    10)
        self.imu2_pub              = self.create_publisher(Imu,           'imu2',                    10)
        self.attitude_pub          = self.create_publisher(Vector3Stamped,'attitude',                10)
        self.battery_pub           = self.create_publisher(BatteryState,  'battery_state',           10)
        self.water_temp_pub        = self.create_publisher(Temperature,   'water_temperature',       10)
        self.canister_temp_pub     = self.create_publisher(Temperature,   'canister_temperature',    10)
        self.cpu_temp_pub          = self.create_publisher(Temperature,   'cpu_temperature',         10)
        self.canister_humidity_pub = self.create_publisher(Float64,       'canister_humidity',       10)
        self.control_force_pub     = self.create_publisher(WrenchStamped, 'control_force',           10)
        self.control_mode_pub      = self.create_publisher(String,        'control_mode',            10)
        self.ctrl_depth_err_pub    = self.create_publisher(Float64,       'controller_depth_error',  10)
        self.ctrl_depth_hlth_pub   = self.create_publisher(Float64,       'controller_depth_health', 10)
        self.ctrl_head_err_pub     = self.create_publisher(Float64,       'controller_heading_error',  10)
        self.ctrl_head_hlth_pub    = self.create_publisher(Float64,       'controller_heading_health', 10)
        self.mag_decl_pub          = self.create_publisher(Float64,       'magnetic_declination',    10)
        self.dive_time_pub         = self.create_publisher(Float64,       'dive_time',               10)

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
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _publish_raw_imu(self, tel, pub, frame_id: str, now):
        """Publish gyro+accel from a raw (non-calibrated-orientation) IMU telemetry."""
        if tel is None:
            return
        msg = Imu()
        msg.header.stamp    = now
        msg.header.frame_id = frame_id
        msg.orientation_covariance[0]     = -1.0   # orientation unknown
        msg.angular_velocity.x            = float(tel.imu.gyroscope.x)
        msg.angular_velocity.y            = float(tel.imu.gyroscope.y)
        msg.angular_velocity.z            = float(tel.imu.gyroscope.z)
        msg.angular_velocity_covariance   = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        msg.linear_acceleration.x         = float(tel.imu.accelerometer.x)
        msg.linear_acceleration.y         = float(tel.imu.accelerometer.y)
        msg.linear_acceleration.z         = float(tel.imu.accelerometer.z)
        msg.linear_acceleration_covariance = [1e-3, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 1e-3]
        pub.publish(msg)

    def _check_error_flags(self):
        try:
            tel = self.drone.telemetry.get(bp.ErrorFlagsTel)
            if tel is None:
                return
            active = [k for k, v in dataclasses.asdict(tel.error_flags).items() if v is True]
            if active:
                self.get_logger().warn(
                    f'Active error flags: {", ".join(active)}',
                    throttle_duration_sec=10.0,
                )
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Publish loop
    # ------------------------------------------------------------------

    def _publish(self):
        if self.IS_SIMULATION or self.drone is None:
            return

        now = self._now()
        self._tick += 1

        # depth (m) 
        try:
            msg = Float64()
            msg.data = float(self.drone.depth) / 1000.0
            self.depth_pub.publish(msg)
        except Exception:
            pass

        # calibrated IMU 
        try:
            pose = self.drone.pose
            roll  = math.radians(float(list(pose.values())[0]))
            pitch = math.radians(float(list(pose.values())[1]))
            yaw   = math.radians(float(list(pose.values())[2]))
            qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

            imu_msg = Imu()
            imu_msg.header.stamp    = now
            imu_msg.header.frame_id = 'blueye_imu'
            imu_msg.orientation.x   = qx
            imu_msg.orientation.y   = qy
            imu_msg.orientation.z   = qz
            imu_msg.orientation.w   = qw
            imu_msg.orientation_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]

            if _HAVE_PROTOCOL:
                imu_tel = self.drone.telemetry.get(bp.CalibratedImuTel)
                if imu_tel is not None:
                    imu_msg.angular_velocity.x = float(imu_tel.imu.gyroscope.x)
                    imu_msg.angular_velocity.y = float(imu_tel.imu.gyroscope.y)
                    imu_msg.angular_velocity.z = float(imu_tel.imu.gyroscope.z)
                    imu_msg.angular_velocity_covariance = [1e-3,0.0,0.0, 0.0,1e-3,0.0, 0.0,0.0,1e-3]
                    imu_msg.linear_acceleration.x = float(imu_tel.imu.accelerometer.x)
                    imu_msg.linear_acceleration.y = float(imu_tel.imu.accelerometer.y)
                    imu_msg.linear_acceleration.z = float(imu_tel.imu.accelerometer.z)
                    imu_msg.linear_acceleration_covariance = [1e-3,0.0,0.0, 0.0,1e-3,0.0, 0.0,0.0,1e-3]
                else:
                    imu_msg.angular_velocity_covariance[0]    = -1.0
                    imu_msg.linear_acceleration_covariance[0] = -1.0
            else:
                imu_msg.angular_velocity_covariance[0]    = -1.0
                imu_msg.linear_acceleration_covariance[0] = -1.0

            self.imu_calibrated_pub.publish(imu_msg)
        except Exception:
            pass

        # battery 
        try:
            soc = self.drone.battery.state_of_charge
            if soc is not None:
                bat_msg = BatteryState()
                bat_msg.header.stamp = now
                bat_msg.percentage   = float(soc)
                bat_msg.current      = float('nan')
                if _HAVE_PROTOCOL:
                    bat_tel = self.drone.telemetry.get(bp.BatteryTel)
                    bat_msg.voltage = float(bat_tel.battery.voltage) if bat_tel is not None else float('nan')
                else:
                    bat_msg.voltage = float('nan')
                bat_msg.present = True
                self.battery_pub.publish(bat_msg)
        except Exception:
            pass

        # water temperature 
        try:
            wt = self.drone.water_temperature
            if wt is not None:
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature  = float(wt)
                msg.variance     = 0.0
                self.water_temp_pub.publish(msg)
        except Exception:
            pass

        if not _HAVE_PROTOCOL:
            return

        #  raw IMUs 
        try:
            self._publish_raw_imu(
                self.drone.telemetry.get(bp.Imu1Tel), self.imu1_pub, 'blueye_imu1', now)
        except Exception:
            pass
        try:
            self._publish_raw_imu(
                self.drone.telemetry.get(bp.Imu2Tel), self.imu2_pub, 'blueye_imu2', now)
        except Exception:
            pass
        # attitude (raw roll/pitch/yaw in degrees) 
        try:
            att_tel = self.drone.telemetry.get(bp.AttitudeTel)
            if att_tel is not None:
                msg = Vector3Stamped()
                msg.header.stamp    = now
                msg.header.frame_id = 'blueye_body'
                msg.vector.x = float(att_tel.attitude.roll)
                msg.vector.y = float(att_tel.attitude.pitch)
                msg.vector.z = float(att_tel.attitude.yaw)
                self.attitude_pub.publish(msg)
        except Exception:
            pass

        # canister humidity 
        try:
            tel = self.drone.telemetry.get(bp.CanisterBottomHumidityTel)
            if tel is not None:
                msg = Float64()
                msg.data = float(tel.humidity.humidity)
                self.canister_humidity_pub.publish(msg)
        except Exception:
            pass

        # canister temp 
        try:
            tel = self.drone.telemetry.get(bp.CanisterBottomTemperatureTel)
            if tel is not None:
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature  = float(tel.temperature.temperature)
                msg.variance     = 0.0
                self.canister_temp_pub.publish(msg)
        except Exception:
            pass

        #  CPU temp 
        try:
            tel = self.drone.telemetry.get(bp.CPUTemperatureTel)
            if tel is not None:
                msg = Temperature()
                msg.header.stamp = now
                msg.temperature  = float(tel.temperature.value)
                msg.variance     = 0.0
                self.cpu_temp_pub.publish(msg)
        except Exception:
            pass

        # control forces (surge / sway / heave / yaw) 
        try:
            tel = self.drone.telemetry.get(bp.ControlForceTel)
            if tel is not None:
                msg = WrenchStamped()
                msg.header.stamp    = now
                msg.header.frame_id = 'blueye_body'
                msg.wrench.force.x  = float(tel.control_force.surge)
                msg.wrench.force.y  = float(tel.control_force.sway)
                msg.wrench.force.z  = float(tel.control_force.heave)
                msg.wrench.torque.z = float(tel.control_force.yaw)
                self.control_force_pub.publish(msg)
        except Exception:
            pass

        # control mode 
        try:
            tel = self.drone.telemetry.get(bp.ControlModeTel)
            if tel is not None:
                s = tel.state
                active = [k for k, v in {
                    'auto_depth':      s.auto_depth,
                    'auto_heading':    s.auto_heading,
                    'auto_altitude':   s.auto_altitude,
                    'station_keeping': s.station_keeping,
                    'weather_vaning':  s.weather_vaning,
                }.items() if v]
                msg = String()
                msg.data = ','.join(active) if active else 'manual'
                self.control_mode_pub.publish(msg)
        except Exception:
            pass

        # controller health 
        try:
            tel = self.drone.telemetry.get(bp.ControllerHealthTel)
            if tel is not None:
                ch = tel.controller_health
                def _f64(v):
                    m = Float64(); m.data = float(v); return m
                self.ctrl_depth_err_pub.publish(_f64(ch.depth_error))
                self.ctrl_depth_hlth_pub.publish(_f64(ch.depth_health))
                self.ctrl_head_err_pub.publish(_f64(ch.heading_error))
                self.ctrl_head_hlth_pub.publish(_f64(ch.heading_health))
        except Exception:
            pass

        # magnetic declination 
        try:
            tel = self.drone.telemetry.get(bp.MagneticDeclinationTel)
            if tel is not None:
                msg = Float64()
                msg.data = float(tel.magnetic_declination.declination)
                self.mag_decl_pub.publish(msg)
        except Exception:
            pass

        # dive time (seconds) 
        try:
            tel = self.drone.telemetry.get(bp.DiveTimeTel)
            if tel is not None:
                msg = Float64()
                msg.data = float(tel.dive_time.value)
                self.dive_time_pub.publish(msg)
        except Exception:
            pass

        # error flags (terminal warn, throttled) 
        self._check_error_flags()


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
