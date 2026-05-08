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

from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix, Imu, BatteryState, Temperature
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
from geographic_msgs.msg import GeoPointStamped


_NAN = float('nan')

_HEADER = [
    'timestamp_utc',
    'depth_m',
    'cal_gyro_x_rads', 'cal_gyro_y_rads', 'cal_gyro_z_rads',
    'cal_accel_x_ms2', 'cal_accel_y_ms2', 'cal_accel_z_ms2',
    'imu1_gyro_x_rads', 'imu1_gyro_y_rads', 'imu1_gyro_z_rads',
    'imu1_accel_x_ms2', 'imu1_accel_y_ms2', 'imu1_accel_z_ms2',
    'imu2_gyro_x_rads', 'imu2_gyro_y_rads', 'imu2_gyro_z_rads',
    'imu2_accel_x_ms2', 'imu2_accel_y_ms2', 'imu2_accel_z_ms2',
    'att_roll_deg', 'att_pitch_deg', 'att_yaw_deg',
    'battery_pct', 'battery_voltage_v',
    'water_temp_c',
    'canister_humidity_pct', 'canister_temp_c',
    'ctrl_surge', 'ctrl_sway', 'ctrl_heave', 'ctrl_yaw',
    'ctrl_mode',
    'ctrl_depth_error', 'ctrl_depth_health',
    'ctrl_heading_error', 'ctrl_heading_health',
    'magnetic_declination_deg',
    'cpu_temp_c',
    'dive_time_s',
    'fix_lat_deg', 'fix_lon_deg', 'fix_alt_m',
    'heading_deg',
    'uwgps_rel_x_m', 'uwgps_rel_y_m', 'uwgps_rel_z_m',
    'uwgps_ned_n_m', 'uwgps_ned_e_m', 'uwgps_ned_d_m',
    'uwgps_lat_deg', 'uwgps_lon_deg', 'uwgps_alt_m',
]


class RovDataLogger(Node):

    def __init__(self):
        super().__init__('rov_data_logger')

        # parameters
        self.declare_parameter('log_dir',  os.path.expanduser('~/rov_logs'))
        self.declare_parameter('log_rate', 1.0)

        # Blueye topics
        self.declare_parameter('log_blueye_depth',           True)
        self.declare_parameter('log_blueye_imu_calibrated',   True)
        self.declare_parameter('log_blueye_imu1',            True)
        self.declare_parameter('log_blueye_imu2',            True)
        self.declare_parameter('log_blueye_attitude',        True)
        self.declare_parameter('log_blueye_battery',         True)
        self.declare_parameter('log_blueye_water_temp',      True)
        self.declare_parameter('log_blueye_canister',        True)
        self.declare_parameter('log_blueye_control_force',   True)
        self.declare_parameter('log_blueye_control_mode',    True)
        self.declare_parameter('log_blueye_controller_health', True)
        self.declare_parameter('log_blueye_magnetic_declination', True)
        self.declare_parameter('log_blueye_cpu_temp',        True)
        self.declare_parameter('log_blueye_dive_time',       True)
        # External / optional
        self.declare_parameter('log_gnss_fix',       False)
        self.declare_parameter('log_heading',        False)
        self.declare_parameter('log_uwgps_relative', False)
        self.declare_parameter('log_uwgps_ned',      False)
        self.declare_parameter('log_uwgps_global',   False)

        log_dir  = self.get_parameter('log_dir').value
        log_rate = self.get_parameter('log_rate').value

        self._en_depth       = self.get_parameter('log_blueye_depth').value
        self._en_imu         = self.get_parameter('log_blueye_imu_calibrated').value
        self._en_imu1        = self.get_parameter('log_blueye_imu1').value
        self._en_imu2        = self.get_parameter('log_blueye_imu2').value
        self._en_attitude    = self.get_parameter('log_blueye_attitude').value
        self._en_battery     = self.get_parameter('log_blueye_battery').value
        self._en_water_temp  = self.get_parameter('log_blueye_water_temp').value
        self._en_canister    = self.get_parameter('log_blueye_canister').value
        self._en_ctrl_force  = self.get_parameter('log_blueye_control_force').value
        self._en_ctrl_mode   = self.get_parameter('log_blueye_control_mode').value
        self._en_ctrl_health = self.get_parameter('log_blueye_controller_health').value
        self._en_mag_decl    = self.get_parameter('log_blueye_magnetic_declination').value
        self._en_cpu_temp    = self.get_parameter('log_blueye_cpu_temp').value
        self._en_dive_time   = self.get_parameter('log_blueye_dive_time').value
        self._en_fix         = self.get_parameter('log_gnss_fix').value
        self._en_heading     = self.get_parameter('log_heading').value
        self._en_rel         = self.get_parameter('log_uwgps_relative').value
        self._en_ned         = self.get_parameter('log_uwgps_ned').value
        self._en_global      = self.get_parameter('log_uwgps_global').value

        os.makedirs(os.path.expanduser(log_dir), exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(os.path.expanduser(log_dir), f'rov_log_{ts}.csv')
        self._file   = open(filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        self._writer.writerow(_HEADER)
        self.get_logger().info(f'Logging to {filepath}')
        self._log_enabled_topics()

        self._depth       = None
        self._imu         = None
        self._imu1        = None
        self._imu2        = None
        self._attitude    = None
        self._battery     = None
        self._water_temp  = None
        self._can_hum     = None
        self._can_temp    = None
        self._ctrl_force  = None
        self._ctrl_mode   = None
        self._ctrl_d_err  = None
        self._ctrl_d_hlth = None
        self._ctrl_h_err  = None
        self._ctrl_h_hlth = None
        self._mag_decl    = None
        self._cpu_temp    = None
        self._dive_time   = None
        self._fix         = None
        self._heading     = None
        self._rel         = None
        self._ned         = None
        self._geo         = None

        if self._en_depth:
            self.create_subscription(Float64,      'depth',                         self._depth_cb,      10)
        if self._en_imu:
            self.create_subscription(Imu,          'imu_calibrated',                self._imu_cb,        10)
        if self._en_imu1:
            self.create_subscription(Imu,          'imu1',                          self._imu1_cb,       10)
        if self._en_imu2:
            self.create_subscription(Imu,          'imu2',                          self._imu2_cb,       10)
        if self._en_attitude:
            self.create_subscription(Vector3Stamped,'attitude',                     self._attitude_cb,   10)
        if self._en_battery:
            self.create_subscription(BatteryState, 'battery_state',                 self._battery_cb,    10)
        if self._en_water_temp:
            self.create_subscription(Temperature,  'water_temperature',             self._water_temp_cb, 10)
        if self._en_canister:
            self.create_subscription(Float64,      'canister_humidity',             self._can_hum_cb,    10)
            self.create_subscription(Temperature,  'canister_temperature',          self._can_temp_cb,   10)
        if self._en_ctrl_force:
            self.create_subscription(WrenchStamped,'control_force',                 self._ctrl_force_cb, 10)
        if self._en_ctrl_mode:
            self.create_subscription(String,       'control_mode',                  self._ctrl_mode_cb,  10)
        if self._en_ctrl_health:
            self.create_subscription(Float64, 'controller_depth_error',   self._ctrl_d_err_cb,  10)
            self.create_subscription(Float64, 'controller_depth_health',  self._ctrl_d_hlth_cb, 10)
            self.create_subscription(Float64, 'controller_heading_error', self._ctrl_h_err_cb,  10)
            self.create_subscription(Float64, 'controller_heading_health',self._ctrl_h_hlth_cb, 10)
        if self._en_mag_decl:
            self.create_subscription(Float64,      'magnetic_declination',           self._mag_decl_cb,  10)
        if self._en_cpu_temp:
            self.create_subscription(Temperature,  'cpu_temperature',               self._cpu_temp_cb,   10)
        if self._en_dive_time:
            self.create_subscription(Float64,      'dive_time',                     self._dive_time_cb,  10)
        if self._en_fix:
            self.create_subscription(NavSatFix,    'fix',                           self._fix_cb,        10)
        if self._en_heading:
            self.create_subscription(Float64,      'heading_deg',                   self._heading_cb,    10)
        if self._en_rel:
            self.create_subscription(Vector3Stamped,'locator_position_relative_wrt_topside', self._rel_cb, 10)
        if self._en_ned:
            self.create_subscription(Vector3Stamped,'locator_position_topside_ned', self._ned_cb,        10)
        if self._en_global:
            self.create_subscription(GeoPointStamped,'locator_position_global',     self._geo_cb,        10)

        self._timer = self.create_timer(1.0 / log_rate, self._write_row)


    def _log_enabled_topics(self):
        enabled = []
        if self._en_depth:       enabled.append('depth')
        if self._en_imu:         enabled.append('imu_calibrated')
        if self._en_imu1:        enabled.append('imu1')
        if self._en_imu2:        enabled.append('imu2')
        if self._en_attitude:    enabled.append('attitude')
        if self._en_battery:     enabled.append('battery_state')
        if self._en_water_temp:  enabled.append('water_temperature')
        if self._en_canister:    enabled.append('canister_humidity+temperature')
        if self._en_ctrl_force:  enabled.append('control_force')
        if self._en_ctrl_mode:   enabled.append('control_mode')
        if self._en_ctrl_health: enabled.append('controller_health (4)')
        if self._en_mag_decl:    enabled.append('magnetic_declination')
        if self._en_cpu_temp:    enabled.append('cpu_temperature')
        if self._en_dive_time:   enabled.append('dive_time')
        if self._en_fix:         enabled.append('fix')
        if self._en_heading:     enabled.append('heading_deg')
        if self._en_rel:         enabled.append('uwgps_relative')
        if self._en_ned:         enabled.append('uwgps_ned')
        if self._en_global:      enabled.append('uwgps_global')
        self.get_logger().info('Subscribed topics: ' + ', '.join(enabled))


    def _depth_cb(self, msg):        self._depth      = msg.data
    def _imu_cb(self, msg):          self._imu        = msg
    def _imu1_cb(self, msg):         self._imu1       = msg
    def _imu2_cb(self, msg):         self._imu2       = msg
    def _attitude_cb(self, msg):     self._attitude   = msg
    def _battery_cb(self, msg):      self._battery    = msg
    def _water_temp_cb(self, msg):   self._water_temp = msg.temperature
    def _can_hum_cb(self, msg):      self._can_hum    = msg.data
    def _can_temp_cb(self, msg):     self._can_temp   = msg.temperature
    def _ctrl_force_cb(self, msg):   self._ctrl_force = msg
    def _ctrl_mode_cb(self, msg):    self._ctrl_mode  = msg.data
    def _ctrl_d_err_cb(self, msg):   self._ctrl_d_err  = msg.data
    def _ctrl_d_hlth_cb(self, msg):  self._ctrl_d_hlth = msg.data
    def _ctrl_h_err_cb(self, msg):   self._ctrl_h_err  = msg.data
    def _ctrl_h_hlth_cb(self, msg):  self._ctrl_h_hlth = msg.data
    def _mag_decl_cb(self, msg):     self._mag_decl   = msg.data
    def _cpu_temp_cb(self, msg):     self._cpu_temp   = msg.temperature
    def _dive_time_cb(self, msg):    self._dive_time  = msg.data
    def _fix_cb(self, msg):          self._fix        = msg
    def _heading_cb(self, msg):      self._heading    = msg.data
    def _rel_cb(self, msg):          self._rel        = msg
    def _ned_cb(self, msg):          self._ned        = msg
    def _geo_cb(self, msg):          self._geo        = msg


    @staticmethod
    def _gyro_accel(imu_msg):
        """Extract gyro (x,y,z) and accel (x,y,z) from an Imu message, NaN if unavailable."""
        if imu_msg is None:
            return _NAN, _NAN, _NAN, _NAN, _NAN, _NAN
        gx = gy = gz = ax = ay = az = _NAN
        if imu_msg.angular_velocity_covariance[0] != -1.0:
            gx = imu_msg.angular_velocity.x
            gy = imu_msg.angular_velocity.y
            gz = imu_msg.angular_velocity.z
        if imu_msg.linear_acceleration_covariance[0] != -1.0:
            ax = imu_msg.linear_acceleration.x
            ay = imu_msg.linear_acceleration.y
            az = imu_msg.linear_acceleration.z
        return gx, gy, gz, ax, ay, az

    # ---- row writer ----------------------------------------------------

    def _write_row(self):
        now = datetime.utcnow().isoformat(timespec='milliseconds')

        depth = self._depth if self._depth is not None else _NAN

        gx = gy = gz = ax = ay = az = _NAN
        if self._imu:
            gx, gy, gz, ax, ay, az = self._gyro_accel(self._imu)

        i1gx, i1gy, i1gz, i1ax, i1ay, i1az = self._gyro_accel(self._imu1)
        i2gx, i2gy, i2gz, i2ax, i2ay, i2az = self._gyro_accel(self._imu2)
        att_r = att_p = att_y = _NAN
        if self._attitude:
            att_r = self._attitude.vector.x
            att_p = self._attitude.vector.y
            att_y = self._attitude.vector.z

        bat_pct = bat_v = _NAN
        if self._battery:
            bat_pct = self._battery.percentage * 100.0
            bat_v   = self._battery.voltage if not math.isnan(self._battery.voltage) else _NAN

        water_temp = self._water_temp if self._water_temp is not None else _NAN
        can_hum    = self._can_hum    if self._can_hum    is not None else _NAN
        can_temp   = self._can_temp   if self._can_temp   is not None else _NAN

        cf_surge = cf_sway = cf_heave = cf_yaw = _NAN
        if self._ctrl_force:
            cf_surge = self._ctrl_force.wrench.force.x
            cf_sway  = self._ctrl_force.wrench.force.y
            cf_heave = self._ctrl_force.wrench.force.z
            cf_yaw   = self._ctrl_force.wrench.torque.z

        ctrl_mode  = self._ctrl_mode  if self._ctrl_mode  is not None else ''
        ctrl_d_err  = self._ctrl_d_err  if self._ctrl_d_err  is not None else _NAN
        ctrl_d_hlth = self._ctrl_d_hlth if self._ctrl_d_hlth is not None else _NAN
        ctrl_h_err  = self._ctrl_h_err  if self._ctrl_h_err  is not None else _NAN
        ctrl_h_hlth = self._ctrl_h_hlth if self._ctrl_h_hlth is not None else _NAN

        mag_decl  = self._mag_decl  if self._mag_decl  is not None else _NAN
        cpu_temp  = self._cpu_temp  if self._cpu_temp  is not None else _NAN
        dive_time = self._dive_time if self._dive_time is not None else _NAN

        fix_lat = fix_lon = fix_alt = _NAN
        if self._fix:
            fix_lat = self._fix.latitude
            fix_lon = self._fix.longitude
            fix_alt = self._fix.altitude if not math.isnan(self._fix.altitude) else _NAN
        heading = self._heading if self._heading is not None else _NAN

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
            gx, gy, gz,
            ax, ay, az,
            i1gx, i1gy, i1gz, i1ax, i1ay, i1az,
            i2gx, i2gy, i2gz, i2ax, i2ay, i2az,
            att_r, att_p, att_y,
            bat_pct, bat_v,
            water_temp,
            can_hum, can_temp,
            cf_surge, cf_sway, cf_heave, cf_yaw,
            ctrl_mode,
            ctrl_d_err, ctrl_d_hlth,
            ctrl_h_err, ctrl_h_hlth,
            mag_decl,
            cpu_temp,
            dive_time,
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
