#!/usr/bin/env python3

import csv
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


MAP_FRAME = "map"
WAYPOINT_FILE = "/home/f1tenth/F1/maps/wp.csv"


class WaypointMarkerPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_marker_publisher")

        self.marker_pub = self.create_publisher(Marker, "/waypoints_marker", 10)

        self.waypoints = self.load_waypoints(WAYPOINT_FILE)

        self.timer = self.create_timer(0.5, self.publish_marker)

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints")
        self.get_logger().info("Publishing marker on /waypoints_marker")
        self.get_logger().info(f"Frame: {MAP_FRAME}")
        self.get_logger().info(f"Waypoint file: {WAYPOINT_FILE}")

    def load_waypoints(self, file_path):
        waypoints = []

        if not os.path.exists(file_path):
            self.get_logger().error(f"Waypoint file not found: {file_path}")
            return waypoints

        with open(file_path, "r") as f:
            reader = csv.reader(f)

            for row in reader:
                if len(row) < 2:
                    continue

                # header skip
                if row[0].strip().lower() == "x":
                    continue

                try:
                    x = float(row[0])
                    y = float(row[1])
                    waypoints.append((x, y))
                except ValueError:
                    continue

        return waypoints

    def make_point(self, x, y, z=0.05):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = float(z)
        return p

    def publish_marker(self):
        if not self.waypoints:
            return

        marker = Marker()
        marker.header.frame_id = MAP_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        # line width
        marker.scale.x = 0.05

        # blue color
        marker.color.r = 0.0
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Add all waypoints in order
        for x, y in self.waypoints:
            marker.points.append(self.make_point(x, y))

        # Close the loop only for RViz visualization.
        # This does NOT modify wp.csv.
        first_x, first_y = self.waypoints[0]
        marker.points.append(self.make_point(first_x, first_y))

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
