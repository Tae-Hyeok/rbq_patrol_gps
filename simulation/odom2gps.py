#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from ublox_msgs.msg import NavPVT

import math
import pymap3d as pm


class OdomToGPS(Node):
    def __init__(self):
        super().__init__("odom_to_gps_node")
        
        self.latest_lat, self.latest_lon = None, None
        self.latest_heading = None
        
        self.prev_E, self.prev_N = None, None
        self.prev_heading = 0
        
        self.sub_odom = self.create_subscription(Odometry, '/rbq/stateEstimation/odometry', self.odom_callback, qos_profile_sensor_data)
        self.pub_fix = self.create_publisher(NavSatFix, '/rbq/patrol_gps_node/sim_fix', 10)
        self.pub_navpvt = self.create_publisher(NavPVT, '/rbq/patrol_gps_node/sim_navpvt', 10)
        
        self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)
        
        self.get_logger().info("Odom to GPS node started")
    
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
    node = OdomToGPS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


