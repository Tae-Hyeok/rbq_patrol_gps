#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped

from sensor_msgs.msg import NavSatFix
from ublox_msgs.msg import NavPVT

from tf2_ros import TransformBroadcaster

import os, time, math


class PatrolGPSNode(Node):
    def __init__(self):
        super().__init__("patrol_gps_node")
        
        self.lat0, self.lon0 = None, None
        self.x, self.y = None, None
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        self.timer, self.f = None, None
        
        self.num_sv, self.fix_type, self.gnss_fix_ok, self.diff_soln_used, self.carr_stat = None, None, None, None, None
        self.lat, self.lon, self.alt = None, None, None
        
        self.sub_gps = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.gps_callback, qos_profile_sensor_data)
        self.sub_gps_stat = self.create_subscription(NavPVT, '/ublox_gps_node/navpvt', self.gps_stat_callback, qos_profile_sensor_data)
        self.sub_cmd = self.create_subscription(String, '/patrol_gps_node/cmd', self.cmd_callback, 10)
        
        self.pub_path = self.create_publisher(Path, '/patrol_gps_node/path', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('patrol_gps_node started')
    
    def gps_callback(self, msg):
        # print(f"LATITUDE: {msg.latitude}")
        # print(f"LONGITUDE: {msg.longitude}")
        # print(f"ALTITUDE: {msg.altitude}")
        
        self.lat, self.lon, self.alt = msg.latitude, msg.longitude, msg.altitude
        self.x, self.y = self.latlon2xy(self.lat, self.lon)

    def gps_stat_callback(self, msg):
        num_sv, fix_type, flags = int(msg.num_sv), int(msg.fix_type), int(msg.flags)
        
        # print(f"NUM_SV: {num_sv}")
        # print(f"fix_type: { {0:'NO_FIX', 2:'2D', 3:'3D'}.get(fix_type, 'UNKNOWN') }")
        
        gnss_fix_ok = bool(flags & 0x01)
        diff_soln_used = bool(flags & 0x02)
        
        carr_bits = flags & 0xC0
        carr_map = {
            0x00: 'NO_SOLUTION',
            0x40: 'FLOAT',
            0x80: 'FIXED',
            0xC0: 'UNKNOWN',
        }
        
        carr_stat = carr_map.get(carr_bits, 'UNKNOWN')
        
        # print(f"GNSS_FIX_OK: {gnss_fix_ok}")
        # print(f"DIFF_SOLN_USED: {diff_soln_used}")
        # print(f"CARRIER_PHASE: {carr_stat}")

        self.num_sv = num_sv
        self.fix_type = fix_type
        self.gnss_fix_ok = gnss_fix_ok
        self.diff_soln_used = diff_soln_used
        self.carr_stat = carr_stat

    def cmd_callback(self, msg):
        cmd = msg.data
        if cmd == 'r':
            print(f"COMMAND: {cmd}")
            # if 1:
            if self.is_reliable():
                print(f"fix_type: {self.fix_type}, gnss_fix_ok: {self.gnss_fix_ok}, diff_soln_used: {self.diff_soln_used}, carr_stat: {self.carr_stat}")
                print("Start recording")
                self.recording()
            else:
                print("ERROR: GPS data is not reliable yet")
                print(f"fix_type: {self.fix_type}, gnss_fix_ok: {self.gnss_fix_ok}, diff_soln_used: {self.diff_soln_used}, carr_stat: {self.carr_stat}")
        elif cmd == 'q':
            print(f"COMMAND: {cmd}")
            print("Stop recording")
            self.timer.cancel()
            self.f.flush()
            self.f.close()

            self.lat0, self.lon0 = None, None
            self.x, self.y = None, None

            print(f"Is GPS data reliable? {self.is_reliable()}")
        elif cmd == 's':
            print("STATUS")
            print(f"fix_type: {self.fix_type}, gnss_fix_ok: {self.gnss_fix_ok}, diff_soln_used: {self.diff_soln_used}, carr_stat: {self.carr_stat}")
        else:
            print("COMMAND ERROR")

    def is_reliable(self):
        if self.fix_type != 3: return False
        if self.gnss_fix_ok != True: return False
        # if self.diff_soln_used != True: return False
        # if self.carr_stat != 'FLOAT' and self.carr_stat != 'FIXED': return False

        print("GPS data is reliable")
        return True
    
    def recording(self):
        os.makedirs('./gps_trajectory', exist_ok=True)
        t = time.strftime('%Y%m%d_%H%M%S')
        self.f = open(f'./gps_trajectory/trajectory_{t}.txt', 'w', encoding='utf-8', buffering=1)
        
        self.timer = self.create_timer(1.0 / 10.0, self.fw_callback)
    
    def fw_callback(self):
        self.f.write(f'{self.lat:.15f} {self.lon:.15f}\n')
        
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.position.z = 0.0
        ps.pose.orientation.w = 1.0

        tf = TransformStamped()
        tf.header.stamp = ps.header.stamp
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'gps'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf)
        
        self.path_msg.poses.append(ps)
        self.path_msg.header.stamp = ps.header.stamp
        self.pub_path.publish(self.path_msg)

    def haversine(self, lat, lon):
        r = 6371000.0 # meters
        
        dlat, dlon = math.radians(lat - self.lat0), math.radians(lon - self.lon0)
        lat0, lat = math.radians(self.lat0), math.radians(lat)
        a = math.sin(dlat/2)**2 + math.sin(dlon/2)**2 * math.cos(lat0) * math.cos(lat)
        return 2 * r * math.asin(math.sqrt(a))
    
    def bearing(self, lat, lon):
        lat0, lon0 = math.radians(self.lat0), math.radians(self.lon0)
        lat, lon = math.radians(lat), math.radians(lon)
        
        x = math.cos(lat0) * math.sin(lat) - math.sin(lat0) * math.cos(lat) * math.cos(lon - lon0)
        y = math.sin(lon - lon0) * math.cos(lat)
        b = math.degrees(math.atan2(y, x))
        return (b + 360) % 360.0
    
    def latlon2xy(self, lat, lon):
        if self.lat0 == None and self.lon0 == None:
            self.lat0, self.lon0 = lat, lon
        d = self.haversine(lat, lon)
        b = math.radians(self.bearing(lat, lon))
        x = d * math.sin(b)
        y = d * math.cos(b)
        return x, y
        

def main(args=None):
    rclpy.init(args=args)
    node = PatrolGPSNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


