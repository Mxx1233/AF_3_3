#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rusty_racer_interfaces.msg import MotorCommand, LaneDeviation
import csv
from datetime import datetime
import sys

class ControlMonitor(Node):
    def __init__(self, duration=None):
        super().__init__('control_monitor')
        
        self.create_subscription(MotorCommand, '/motor_command', self.motor_cb, 10)
        self.create_subscription(LaneDeviation, '/lane_deviation', self.lane_cb, 10)
        
        self.latest_motor = None
        self.latest_lane = None
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = f'data/run_{timestamp}.csv'
        self.csv_file = open(self.filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'lat_err', 'hdg_err', 'curvature',
            'steer_angle', 'motor_level'
        ])
        
        self.get_logger().info(f'Recording to: {self.filename}')
        self.create_timer(0.1, self.write_data)
        
        if duration:
            self.create_timer(duration, self.stop_recording)
    
    def motor_cb(self, msg):
        self.latest_motor = msg
    
    def lane_cb(self, msg):
        self.latest_lane = msg
    
    def write_data(self):
        if not all([self.latest_motor, self.latest_lane]):
            return
        
        t = self.get_clock().now().nanoseconds / 1e9
        
        self.csv_writer.writerow([
            t,
            self.latest_lane.lateral_error,
            self.latest_lane.heading_error,
            self.latest_lane.curvature,
            self.latest_motor.steering_angle,
            self.latest_motor.motor_level
        ])
    
    def stop_recording(self):
        self.get_logger().info(f'Recording stopped: {self.filename}')
        self.csv_file.close()
        rclpy.shutdown()

def main():
    rclpy.init()
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else None
    node = ControlMonitor(duration)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
        node.csv_file.close()

if __name__ == '__main__':
    main()