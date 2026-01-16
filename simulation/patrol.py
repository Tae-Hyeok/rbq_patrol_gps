#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT

from rbq_msgs.msg import HighLevelCommand

import math, time


class Patrol(Node):
    def __init__(self):
        super().__init__("patrol_node")
        
        self.wp_file = '/home/dev/gps_setting_ws/gps_trajectory/wp_2.txt'
        self.wp = []
        self.wp_idx = 0
 
        self.load_wp()

        self.patrol, self.end = False, False

        self.prev_err = None
        self.err_count = 0
        self.find_wp = False
        
        self.vel = 0.5
        self.k = 0.5

        self.cur_lat, self.cur_lon, self.cur_heading = None, None, None
        
        self.sub_fix = self.create_subscription(NavSatFix, '/rbq/patrol_gps_node/sim_fix', self.fix_callback, qos_profile_sensor_data)
        self.sub_navpvt = self.create_subscription(NavPVT, '/rbq/patrol_gps_node/sim_navpvt', self.navpvt_callback, qos_profile_sensor_data)
        self.sub_cmd = self.create_subscription(String, '/rbq/patrol_gps_node/sim_cmd', self.cmd_callback, 10)
        
        self.pub_auto_start = self.create_publisher(Bool, '/rbq/motion/autoStart', 10)
        self.pub_cmd = self.create_publisher(HighLevelCommand, '/rbq/motion/cmd_highLevel', 10)
        self.pub_control_mode = self.create_publisher(Bool, '/rbq/motion/switchControlMode', 10)
        self.pub_gait = self.create_publisher(Int8, '/rbq/motion/switchGait', 10)
        
        self.timer = self.create_timer(1.0 / 5.0, self.control_rbq)
        
        self.pub_auto_start.publish(Bool(data=True))
        
        time.sleep(1)
        self.pub_gait.publish(Int8(data=1))
        
        self.get_logger().info("Patrol node started")

    def load_wp(self):
        with open(self.wp_file, 'r', encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if not s:
                    continue
                data = s.split()
                lat, lon = float(data[0]), float(data[1])
                self.wp.append((lat, lon))

    def fix_callback(self, msg):
        self.cur_lat, self.cur_lon = msg.latitude, msg.longitude

    def navpvt_callback(self, msg):
        self.cur_heading = float(msg.heading) * 1e-5
    
    def publish_cmd(self, vel_x, omega_z):
        cmd = HighLevelCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()

        cmd.vel_x = vel_x
        cmd.vel_y = 0.0
        cmd.omega_z = omega_z
        
        self.pub_cmd.publish(cmd)

    def publish_stop(self):
        self.publish_cmd(vel_x=0.0, omega_z=0.0)

    def cmd_callback(self, msg):
        cmd = msg.data
        if cmd == 'g':
            print(f"COMMAND: {cmd}")

            self.pub_control_mode.publish(Bool(data=True))
            self.pub_gait.publish(Int8(data=42))

            if self.end:
                self.get_logger().warn("Already finished.")
                return

            self.patrol = True
            self.get_logger().info("Start patrol")

        elif cmd == 's':
            print(f"COMMAND: {cmd}")
            self.patrol = False
            self.publish_stop()
            self.get_logger().info("Stop patrol")
    
        else:
            print("COMMAND ERROR")

    def haversine(self, lat0, lon0, lat, lon):
        r = 6371000.0 # meters
        
        dlat, dlon = math.radians(lat - lat0), math.radians(lon - lon0)
        lat0, lat = math.radians(lat0), math.radians(lat)
        a = math.sin(dlat/2)**2 + math.sin(dlon/2)**2 * math.cos(lat0) * math.cos(lat)
        return 2 * r * math.asin(math.sqrt(a))

    def bearing(self, lat0, lon0, lat, lon):
        lat0, lon0 = math.radians(lat0), math.radians(lon0)
        lat, lon = math.radians(lat), math.radians(lon)
        
        x = math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * math.cos(lon - lon0)
        y = math.sin(lon - lon0) * math.cos(lat)
        b = math.degrees(math.atan2(y, x))
        return (b + 360) % 360.0

    def wrapto180(self, deg):
        return (deg + 180.0) % 360.0 - 180.0

    def clip(self, val, low, high):
        return low if val < low else high if val > high else val

    def control_rbq(self):
        if not self.patrol:
            self.publish_stop()
            return

        if self.cur_lat is None or self.cur_lon is None or self.cur_heading is None:
            self.publish_stop()
            return

        if self.wp_idx >= len(self.wp):
            self.patrol = False
            self.end = True
            self.publish_stop()
            self.get_logger().info("Arrived final WP.")
            return

        tgt_lat, tgt_lon = self.wp[self.wp_idx]
        dist = self.haversine(self.cur_lat, self.cur_lon, tgt_lat, tgt_lon)

        if dist < 3.0:
            self.wp_idx += 1
            if self.wp_idx >= len(self.wp):
                self.patrol = False
                self.end = True
                self.publish_stop()
                self.get_logger().info("Arrived final WP.")
            return

        bearing_deg = self.bearing(self.cur_lat, self.cur_lon, tgt_lat, tgt_lon)

        err_deg = self.wrapto180(bearing_deg - self.cur_heading)
        abs_deg = abs(err_deg)

        if self.prev_err is None:
            self.prev_err = abs_deg

        if self.prev_err < abs_deg:
            self.err_count += 1

            if self.err_count == 3:
                self.find_wp = True

        self.prev_err = abs_deg

        # err_rad = math.radians(err_deg)
        # omega = self.k * err_rad

        omega = self.k * err_deg
        if self.find_wp:
            self.publish_cmd(vel_x=self.vel, omega_z=-omega)
            self.err_count -= 1
            if self.err_count == 0:
                self.find_wp = False
        else:
            self.publish_cmd(vel_x=self.vel, omega_z=omega)

        print(
            f"[WP {self.wp_idx + 1}/{len(self.wp)}] "
            f"dist={dist:.2f}m bearing={bearing_deg:.1f} "
            f"err={err_deg:+.1f}deg "
            f"v={self.vel:.2f} omega={omega:+.2f} "
            f"find_wp={self.find_wp})"
        )

    '''
    # gpt ver 1
    def control_rbq(self):
        # -----------------------------
        # lazy-init (init 수정 금지)
        # -----------------------------
        if not hasattr(self, "_prev_dist"):
            self._prev_dist = None
            self._stuck_cnt = 0
        if not hasattr(self, "_prev_abs_err"):
            self._prev_abs_err = None
            self._err_stuck_cnt = 0
        if not hasattr(self, "_prev_latlon"):
            self._prev_latlon = None
            self._head_course_deg = None

        # -----------------------------
        # 기본 가드
        # -----------------------------
        if not self.patrol:
            self.publish_stop()
            return

        if self.cur_lat is None or self.cur_lon is None or self.cur_heading is None:
            self.publish_stop()
            return

        if self.end:
            self.patrol = False
            self.publish_stop()
            return

        if self.wp_idx >= len(self.wp):
            self.end = True
            self.patrol = False
            self.publish_stop()
            self.get_logger().info("Arrived final WP.")
            return

        # -----------------------------
        # 목표 / dist / bearing
        # -----------------------------
        tgt_lat, tgt_lon = self.wp[self.wp_idx]
        dist_m = self.haversine(self.cur_lat, self.cur_lon, tgt_lat, tgt_lon)

        # 도착 판정
        if dist_m < 3.0:
            self.wp_idx += 1
            if self.wp_idx >= len(self.wp):
                self.end = True
                self.patrol = False
                self.publish_stop()
                self.get_logger().info("Arrived final WP.")
            return

        bearing_deg = self.bearing(self.cur_lat, self.cur_lon, tgt_lat, tgt_lon)

        # -----------------------------
        # head_used: GPS 코스(위치 변화) 우선, 없으면 PVT
        # -----------------------------
        if self._prev_latlon is None:
            self._prev_latlon = (float(self.cur_lat), float(self.cur_lon))
        else:
            plat, plon = self._prev_latlon
            d_move = self.haversine(plat, plon, float(self.cur_lat), float(self.cur_lon))
            if d_move > 0.05:  # 5cm 이상 이동 시 코스 갱신
                self._head_course_deg = self.bearing(plat, plon, float(self.cur_lat), float(self.cur_lon))
                self._prev_latlon = (float(self.cur_lat), float(self.cur_lon))

        if self._head_course_deg is not None:
            head_used = float(self._head_course_deg) % 360.0
            head_src = "COURSE"
        else:
            head_used = float(self.cur_heading) % 360.0
            head_src = "PVT"

        err_deg = self.wrapto180(bearing_deg - head_used)
        abs_err = abs(err_deg)
        err_rad = math.radians(err_deg)

        # -----------------------------
        # stuck 감지
        # -----------------------------
        if self._prev_dist is not None:
            if dist_m > (self._prev_dist - 0.02):
                self._stuck_cnt += 1
            else:
                self._stuck_cnt = 0
        self._prev_dist = dist_m

        if self._prev_abs_err is not None:
            if abs_err > (self._prev_abs_err - 1.0):
                self._err_stuck_cnt += 1
            else:
                self._err_stuck_cnt = 0
        self._prev_abs_err = abs_err

        # -----------------------------
        # 제어 파라미터
        # -----------------------------
        v_go = float(self.vel)  # 0.5
        v_min_turn = 0.30  # 코스 업데이트를 위해 회전 중 최소 전진
        k = float(self.k)  # 5.0

        omega_cap_hi = 6.0
        omega_cap_md = 4.0
        omega_cap_lo = 2.0
        omega_cap_go = 1.2

        def clip(x, lo, hi):
            return lo if x < lo else hi if x > hi else x

        # -----------------------------
        # 상태 결정 (여기서 omega는 "수학적 부호" 기준)
        # -----------------------------
        if abs_err > 60.0:
            v = max(v_min_turn, 0.35)
            omega = clip(k * err_rad, -omega_cap_hi, omega_cap_hi)
            mode = "TURN_MAX"
        elif abs_err > 25.0:
            v = max(v_min_turn, 0.40)
            omega = clip(k * err_rad, -omega_cap_md, omega_cap_md)
            mode = "TURN_HARD"
        elif abs_err > 10.0:
            v = v_go
            omega = clip(k * err_rad, -omega_cap_lo, omega_cap_lo)
            mode = "TURN"
        else:
            v = v_go
            omega = clip(0.9 * k * err_rad, -omega_cap_go, omega_cap_go)
            mode = "GO"

        # -----------------------------
        # 킥
        # -----------------------------
        if (self._stuck_cnt >= 8) or (self._err_stuck_cnt >= 10):
            v = max(0.45, v)
            omega = math.copysign(omega_cap_hi, err_deg if err_deg != 0 else 1.0)
            mode = "KICK_TURN"
            self._stuck_cnt = 0
            self._err_stuck_cnt = 0

        # =========================================================
        # ★ 핵심 수정: RBQ omega_z 부호가 반대라서 뒤집어서 보냄
        # =========================================================
        omega_cmd = -omega

        self.publish_cmd(v, omega_cmd)

        print(
            f"[wp {self.wp_idx + 1}/{len(self.wp)}] {mode} "
            f"dist={dist_m:.2f}m bearing={bearing_deg:.1f} "
            f"head_used={head_used:.1f}({head_src}) err={err_deg:+.1f}deg "
            f"v={v:.2f} omega={omega_cmd:+.2f} (raw:{omega:+.2f})"
        )
    '''

    '''
    def control_rbq(self):
        if not self.patrol:
            self.publish_stop()
            return
        
        if self.cur_lat is None or self.cur_lon is None or self.cur_heading is None:
            self.publish_stop()
            return
        
        if self.end:
            self.patrol = False
            self.publish_stop()
            return

        if self.wp_idx >= len(self.wp):
            self.end = True
            self.patrol = False
            self.publish_stop()
            self.get_logger().info("Arrived final WP.")
            return
        
        tlat, tlon = self.wp[self.wp_idx]
        dist = self.haversine(self.cur_lat, self.cur_lon, tlat, tlon)
        
        if dist < 3.0:
            self.wp_idx += 1
            if self.wp_idx >= len(self.wp):
                self.end = True
                self.patrol = False
                self.publish_stop()
                self.get_logger().info("Arrived final WP.")
            return
        
        print(f"{self.wp_idx}/{len(self.wp)} dist={dist:.2f}m")
        
        target_bearing = self.bearing(self.cur_lat, self.cur_lon, tlat, tlon)
        err_deg = self.wrapto180(target_bearing - self.cur_heading)
        err_rad = self.k * math.radians(err_deg)
        
        target_vel = self.vel / abs(err_rad) * 5
        target_vel = min(self.vel, target_vel)
        
        print(f"Bearing error (radians): {err_rad}")
        print(f"Target velocity: {target_vel}")

        self.publish_cmd(vel_x=target_vel, omega_z=err_rad)  
    '''      

    def timer_callback(self):
        if self.latest_lat is None or self.latest_lon is None:
            return

        fix = NavSatFix()
        fix.latitude, fix.longitude = float(self.latest_lat), float(self.latest_lon)

        self.pub_fix.publish(fix)

        navpvt = NavPVT()
        navpvt.heading = self.latest_heading
        navpvt.fix_type = 3
        navpvt.flags = 0x83
        
        navpvt.num_sv = 8

        self.pub_navpvt.publish(navpvt)

    def odom_callback(self, msg):
        x, y = float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)
        
        # FLU to ENU
        E, N = self.flu2enu(x, y)
        
        # ENU to WGS84(lat, lon, alt)
        lat, lon, _ = pm.enu2geodetic(E, N, 0.0, 
                                      36.387408400000000, 127.404563400000000, 0.000000000000000,
                                      deg=True)

        self.latest_lat, self.latest_lon = lat, lon

        if self.prev_E is not None:
            dE = E - self.prev_E
            dN = N - self.prev_N
            
            dist = math.hypot(dE, dN)

            if dist > 0.03:
            	heading = self.degto1e5(math.degrees(math.atan2(dE, dN)))

            	self.prev_heading = heading
            else:
                heading = self.prev_heading
        else:
            heading = self.prev_heading

        self.prev_E, self.prev_N = E, N

        self.latest_heading = heading

    def degto1e5(self, deg):
        return int(round(deg % 360.0 * 1e5))
        
    
    def flu2enu(self, x, y):
        E = -y # x * math.sin(0) - y * math.cos(0)
        N = x # x * math.cos(0) + y * math.sin(0)
        return E, N


def main(args=None):
    rclpy.init(args=args)
    node = Patrol()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


