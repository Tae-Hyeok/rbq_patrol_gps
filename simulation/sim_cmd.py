#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys


class SimCmdPublisher(Node):
    def __init__(self):
        super().__init__("sim_cmd_publisher")
        
        self.pub_cmd = self.create_publisher(String, '/rbq/patrol_gps_node/sim_cmd', 10)
        
        self.print_help()
    
    def print_help(self):
        print("Press g: START PATROL")
        print("Press s: STOP PATROL")
        print("Press q: STOP CMD")

    def cmd_loop(self):
        while rclpy.ok():
            line = sys.stdin.readline()
            
            if not line: continue
            cmd = line.strip()
            
            self.pub_cmd.publish(String(data=cmd))
            
            if cmd == "q":
                print("Stop cmd") 
                break


def main(args=None):
    rclpy.init(args=args)
    node = SimCmdPublisher()
    
    try:
        node.cmd_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


