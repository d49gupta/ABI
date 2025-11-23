#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
import time

class calibrationBlock(Node):
    def __init__(self, laser_resolution=0.05, conveyor_intervals=None):
        super().__init__('calibration_block_node')
        self.publisher_ = self.create_publisher(Marker, 'calibration_block_marker', 10)
        self.circle_marker = Marker()
        self.line_marker = Marker()
        self.eddy_marker = Marker()
        self.conveyor_marker = Marker()

        self.laser_resolution = laser_resolution
        self.laser_x = None
        self.laser_y = None
        self.conveyor_intervals = conveyor_intervals
        self.current_interval_index = 0
        self.conveyor_position = 1.0*self.conveyor_intervals[self.current_interval_index]
        self.state = "idle"

    def make_circle_marker(self, radius=1.0, color=(0.0, 0.0, 1.0), height=1.0):
        self.circle_marker.points.clear()
        self.circle_marker.header.frame_id = "map"
        self.circle_marker.header.stamp = self.get_clock().now().to_msg()
        self.circle_marker.ns = "circle"
        self.circle_marker.id = 0
        self.circle_marker.type = Marker.LINE_STRIP
        self.circle_marker.action = Marker.ADD
        self.circle_marker.pose.orientation.w = 1.0
        self.circle_marker.scale.x = 0.02  # line thickness
        self.circle_marker.color.r, self.circle_marker.color.g, self.circle_marker.color.b, self.circle_marker.color.a = *color, 1.0

        num_points = 100
        for i in range(num_points + 1):
            theta = 2 * math.pi * i / num_points
            x = self.conveyor_position + radius * math.cos(theta)
            y = radius * math.sin(theta)
            self.circle_marker.points.append(Point(x=x, y=y, z=height))
        return self.circle_marker

    def make_lines_marker(self, length=1.0, color=(1.0, 0.0, 0.0), height=1.0):
        self.line_marker.points.clear()
        self.line_marker.header.frame_id = "map"
        self.line_marker.header.stamp = self.get_clock().now().to_msg()
        self.line_marker.ns = "lines"
        self.line_marker.id = 1
        self.line_marker.type = Marker.LINE_LIST
        self.line_marker.action = Marker.ADD
        self.line_marker.pose.orientation.w = 1.0
        self.line_marker.scale.x = 0.03
        self.line_marker.color.r, self.line_marker.color.g, self.line_marker.color.b, self.line_marker.color.a = *color, 1.0

        # X-axis line
        self.line_marker.points.append(Point(x=self.conveyor_position - length, y=0.0, z=height))
        self.line_marker.points.append(Point(x=self.conveyor_position + length, y=0.0, z=height))

        # Y-axis line
        self.line_marker.points.append(Point(x=self.conveyor_position, y=-length, z=height))
        self.line_marker.points.append(Point(x=self.conveyor_position, y=length, z=height))

        laser_vals = np.arange(-length, length + self.laser_resolution, self.laser_resolution)
        x_zero = np.zeros_like(laser_vals)
        y_zero = np.zeros_like(laser_vals)
        z_height = np.full_like(laser_vals, height)

        self.laser_x = np.column_stack((laser_vals + self.conveyor_position, y_zero, z_height))
        self.laser_y = np.column_stack((x_zero + self.conveyor_position, laser_vals, z_height))
        return self.line_marker
    
    def make_eddy_marker(self, position=(0.0, 0.0, 0.0), color=(1.0, 1.0, 1.0)):
        self.eddy_marker.header.frame_id = "map"
        self.eddy_marker.header.stamp = self.get_clock().now().to_msg()
        self.eddy_marker.ns = "eddy_point"
        self.eddy_marker.id = 2
        self.eddy_marker.type = Marker.SPHERE
        self.eddy_marker.action = Marker.ADD
        self.eddy_marker.pose.position.x = self.conveyor_position
        self.eddy_marker.pose.position.y = position[1]
        self.eddy_marker.pose.position.z = position[2]
        self.eddy_marker.pose.orientation.w = 1.0

        self.eddy_marker.scale.x = 0.05
        self.eddy_marker.scale.y = 0.05
        self.eddy_marker.scale.z = 0.05
        self.eddy_marker.color.r, self.eddy_marker.color.g, self.eddy_marker.color.b, self.eddy_marker.color.a = *color, 1.0
        return self.eddy_marker
    
    def move_block(self, steps=50):
        if self.current_interval_index >= len(self.conveyor_intervals) - 1:
            self.state = "complete"
            return

        self.state = "moving"
        start_pos = self.conveyor_intervals[self.current_interval_index]
        end_pos = self.conveyor_intervals[self.current_interval_index + 1]
        dx = end_pos - start_pos

        step_size = dx / steps
        for i in range(steps + 1):
            self.conveyor_position = start_pos + i * step_size
            self.publish_markers()
            time.sleep(0.1)

        self.current_interval_index += 1
        self.state = "idle"
        self.get_logger().info(f"Conveyor moved to interval {self.current_interval_index}: x = {self.conveyor_position:.2f}")

    def publish_markers(self):
        self.publisher_.publish(self.make_circle_marker())
        self.publisher_.publish(self.make_lines_marker())
        self.publisher_.publish(self.make_eddy_marker())


class conveyorBelt(Node):
        def __init__(self, conveyor_length=10.0, num_points=4):
            super().__init__('marker_node')
            self.publisher_ = self.create_publisher(Marker, 'conveyor_marker', 10)
            self.conveyor_marker = Marker()
            self.conveyor_length = conveyor_length

            usable_length = conveyor_length * 0.8
            if num_points == 1:
                self.conveyor_intervals = [0]
            else:
                self.conveyor_intervals = [(-usable_length/2) + i*(usable_length/(num_points-1)) for i in range(num_points)]

        def make_conveyor_marker(self, width=2.0, color=(0.0, 0.0, 1.0), height=1.0):
            self.conveyor_marker.header.frame_id = "map"
            self.conveyor_marker.header.stamp = self.get_clock().now().to_msg()
            self.conveyor_marker.ns = "conveyor"
            self.conveyor_marker.id = 3
            self.conveyor_marker.type = Marker.LINE_LIST
            self.conveyor_marker.action = Marker.ADD
            self.conveyor_marker.pose.orientation.w = 1.0
            self.conveyor_marker.scale.x = 0.03
            self.conveyor_marker.color.r, self.conveyor_marker.color.g, self.conveyor_marker.color.b, self.conveyor_marker.color.a = *color, 1.0

            # X-axis line
            self.conveyor_marker.points.append(Point(x=-self.conveyor_length, y=-width, z=height))
            self.conveyor_marker.points.append(Point(x=self.conveyor_length, y=-width, z=height))
            self.conveyor_marker.points.append(Point(x=-self.conveyor_length, y=width, z=height))
            self.conveyor_marker.points.append(Point(x=self.conveyor_length, y=width, z=height))

            return self.conveyor_marker

        def publish_markers(self):
            self.publisher_.publish(self.make_conveyor_marker())

def main(args=None):
    rclpy.init(args=args)
    node = calibrationBlock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
