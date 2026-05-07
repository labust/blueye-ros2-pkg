#!/usr/bin/env python3
"""
ROV data logger - writes a timestamped CSV from live ROS2 topics.

Output: <log_dir>/rov_log_YYYYMMDD_HHMMSS.csv   (default: ~/rov_logs/)
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Imu, BatteryState, Temperature
from geometry_msgs.msg import Vector3Stamped
from geographic_msgs.msg import GeoPointStamped


_NAN = float('nan')

# Full CSV column layout — never changes regardless of what is enabled
_HEADER = [
    'timestamp_utc',
    # Blueye
    'depth_m',
    'roll_deg', 'pitch_deg', 'yaw_deg',
    'gyro_x_rads', 'gyro_y_rads', 'gyro_z_rads',
    'accel_x_ms2', 'accel_y_ms2', 'accel_z_ms2',
    'battery_pct', 'battery_voltage_v', 'battery_current_a',
    'altitude_m',
    'water_temp_c',
    # External GNSS + heading
    'fix_lat_deg', 'fix_lon_deg', 'fix_alt_m',
    'heading_deg',
    # WaterLinked
    'uwgps_rel_x_m', 'uwgps_rel_y_m', 'uwgps_rel_z_m',
    'uwgps_ned_n_m', 'uwgps_ned_e_m', 'uwgps_ned_d_m',
    'uwgps_lat_deg', 'uwgps_lon_deg', 'uwgps_alt_m',
]


class RovDataLogger(Node):

    def __init__(self):
        super().__init__('rov_data_logger')

        # Parameters 
        self.declare_parameter('log_dir',  os.path.expanduser('~/rov_logs'))
        self.declare_parameter('log_rate', 1.0)

        self.declare_parameter('log_blueye_depth',      True)
        self.declare_parameter('log_blueye_imu',        True)
        self.declare_parameter('log_blueye_battery',    True)
        self.declare_parameter('log_blueye_altitude',   True)
        self.declare_parameter('log_blueye_water_temp', True)

        self.declare_parameter('log_gnss_fix',  False)
        self.declare_parameter('log_heading',   False)

        self.declare_parameter('log_uwgps_relative', False)
        self.declare_parameter('log_uwgps_ned',      False)
        self.declare_parameter('log_uwgps_global',   False)

        log_dir  = self.get_parameter('log_dir').value
        log_rate = self.get_parameter('log_rate').value

        self._en_depth      = self.get_parameter('log_blueye_depth').value
        self._en_imu        = self.get_parameter('log_blueye_imu').value
        self._en_battery    = self.get_parameter('log_blueye_battery').value
        self._en_altitude   = self.get_parameter('log_blueye_altitude').value
        self._en_water_temp = self.get_parameter('log_blueye_water_temp').value
        self._en_fix        = self.get_parameter('log_gnss_fix').value
        self._en_heading    = self.get_parameter('log_heading').value
        self._en_rel        = self.get_parameter('log_uwgps_relative').value
        self._en_ned        = self.get_parameter('log_uwgps_ned').value
        self._en_global     = self.get_parameter('log_uwgps_global').value

        # Open CSV 
        os.makedirs(os.path.expanduser(log_dir), exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(os.path.expanduser(log_dir), f'rov_log_{ts}.csv')
        self._file   = open(filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(_HEADER)
        self.get_logger().info(f'Logging to {filepath}')
        self._log_enabled_topics()

        # Cached values 
        self._depth = self._imu = self._battery = None
        self._altitude = self._water_temp = None
        self._fix = self._heading = None
        self._rel = self._ned = self._geo = None

        # Conditional subscriptions 
        if self._en_depth:
            self.create_subscription(Float64, 'depth', self._depth_cb, 10)
        if self._en_imu:
            self.create_subscription(Imu, 'imu', self._imu_cb, 10)
        if self._en_battery:
            self.create_subscription(BatteryState, 'battery_state', self._battery_cb, 10)
        if self._en_altitude:
            self.create_subscription(Float64, 'altitude', self._altitude_cb, 10)
        if self._en_water_temp:
            self.create_subscription(Temperature, 'water_temperature', self._water_temp_cb, 10)
        if self._en_fix:
            self.create_subscription(NavSatFix, 'fix', self._fix_cb, 10)
        if self._en_heading:
            self.create_subscription(Float64, 'heading_deg', self._heading_cb, 10)
        if self._en_rel:
            self.create_subscription(
                Vector3Stamped, 'locator_position_relative_wrt_topside', self._rel_cb, 10)
        if self._en_ned:
            self.create_subscription(
                Vector3Stamped, 'locator_position_topside_ned', self._ned_cb, 10)
        if self._en_global:
            self.create_subscription(
                GeoPointStamped, 'locator_position_global', self._geo_cb, 10)

        self._timer = self.create_timer(1.0 / log_rate, self._write_row)

    # Logging summary 

    def _log_enabled_topics(self):
        enabled = []
        if self._en_depth:      enabled.append('blueye/depth')
        if self._en_imu:        enabled.append('blueye/imu')
        if self._en_battery:    enabled.append('blueye/battery_state')
        if self._en_altitude:   enabled.append('blueye/altitude')
        if self._en_water_temp: enabled.append('blueye/water_temperature')
        if self._en_fix:        enabled.append('fix')
        if self._en_heading:    enabled.append('heading_deg')
        if self._en_rel:        enabled.append('uwgps/locator_position_relative_wrt_topside')
        if self._en_ned:        enabled.append('uwgps/locator_position_topside_ned')
        if self._en_global:     enabled.append('uwgps/locator_position_global')
        self.get_logger().info('Subscribed topics: ' + ', '.join(enabled))

    # Callbacks 

    def _depth_cb(self, msg):      self._depth      = msg.data
    def _imu_cb(self, msg):        self._imu        = msg
    def _battery_cb(self, msg):    self._battery    = msg
    def _altitude_cb(self, msg):   self._altitude   = msg.data
    def _water_temp_cb(self, msg): self._water_temp = msg.temperature
    def _fix_cb(self, msg):        self._fix        = msg
    def _heading_cb(self, msg):    self._heading    = msg.data
    def _rel_cb(self, msg):        self._rel        = msg
    def _ned_cb(self, msg):        self._ned        = msg
    def _geo_cb(self, msg):        self._geo        = msg

    # Row builder 

    @staticmethod
    def _quat_to_euler_deg(o):
        x, y, z, w = o.x, o.y, o.z, o.w
        roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
        yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
        return roll, pitch, yaw

    def _write_row(self):
        now = datetime.utcnow().isoformat(timespec='milliseconds')

        # depth
        depth = self._depth if self._depth is not None else _NAN

        # IMU
        roll = pitch = yaw = _NAN
        gx = gy = gz = _NAN
        ax = ay = az = _NAN
        if self._imu:
            cov_ori = self._imu.orientation_covariance[0]
            cov_gyro = self._imu.angular_velocity_covariance[0]
            cov_acc  = self._imu.linear_acceleration_covariance[0]
            if cov_ori != -1.0:
                roll, pitch, yaw = self._quat_to_euler_deg(self._imu.orientation)
            if cov_gyro != -1.0:
                gx = self._imu.angular_velocity.x
                gy = self._imu.angular_velocity.y
                gz = self._imu.angular_velocity.z
            if cov_acc != -1.0:
                ax = self._imu.linear_acceleration.x
                ay = self._imu.linear_acceleration.y
                az = self._imu.linear_acceleration.z

        # battery
        bat_pct = bat_v = bat_i = _NAN
        if self._battery:
            bat_pct = self._battery.percentage * 100.0
            bat_v   = self._battery.voltage  if not math.isnan(self._battery.voltage)  else _NAN
            bat_i   = self._battery.current  if not math.isnan(self._battery.current)  else _NAN

        altitude   = self._altitude   if self._altitude   is not None else _NAN
        water_temp = self._water_temp if self._water_temp is not None else _NAN

        # fix
        fix_lat = fix_lon = fix_alt = _NAN
        if self._fix:
            fix_lat = self._fix.latitude
            fix_lon = self._fix.longitude
            fix_alt = self._fix.altitude if not math.isnan(self._fix.altitude) else _NAN

        heading = self._heading if self._heading is not None else _NAN

        # WaterLinked
        rx = ry = rz = _NAN
        if self._rel:
            rx, ry, rz = self._rel.vector.x, self._rel.vector.y, self._rel.vector.z

        nn = ne = nd = _NAN
        if self._ned:
            nn, ne, nd = self._ned.vector.x, self._ned.vector.y, self._ned.vector.z

        glat = glon = galt = _NAN
        if self._geo:
            glat = self._geo.position.latitude
            glon = self._geo.position.longitude
            galt = self._geo.position.altitude

        self._writer.writerow([
            now,
            depth,
            roll, pitch, yaw,
            gx, gy, gz,
            ax, ay, az,
            bat_pct, bat_v, bat_i,
            altitude,
            water_temp,
            fix_lat, fix_lon, fix_alt,
            heading,
            rx, ry, rz,
            nn, ne, nd,
            glat, glon, galt,
        ])
        self._file.flush()

    def destroy_node(self):
        self._file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RovDataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
