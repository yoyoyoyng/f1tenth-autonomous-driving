#!/usr/bin/env python3

import os
import csv
import math
import atexit
from time import gmtime, strftime

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion


# ==============================
# Fixed settings for our F1TENTH car
# ==============================
GLOBAL_FRAME = "map"
BASE_FRAME = "base_link"

# Save waypoint files inside ~/F1/maps
OUTPUT_DIR = "/home/f1tenth/F1/maps"

SAVE_INTERVAL = 0.05      # seconds. How often to check TF.
MIN_DISTANCE = 0.1       # meters. Save only if car moved at least this far.
DEFAULT_SPEED = 0.40      # m/s. Target speed saved in waypoint file.


class MapWaypointsLogger(Node):
    def __init__(self):
        super().__init__("map_waypoints_logger")

        os.makedirs(OUTPUT_DIR, exist_ok=True)

        filename = strftime("wp-map-%Y-%m-%d-%H-%M-%S.csv", gmtime())
        self.file_path = os.path.join(OUTPUT_DIR, filename)

        self.file = open(self.file_path, "w", newline="")
        self.writer = csv.writer(self.file)

        # Format: x, y, yaw, speed
        self.writer.writerow(["x", "y", "yaw", "speed"])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_x = None
        self.prev_y = None
        self.num_saved = 0

        self.timer = self.create_timer(SAVE_INTERVAL, self.save_waypoint)

        atexit.register(self.shutdown)

        self.get_logger().info("Map waypoint logger started.")
        self.get_logger().info(f"GLOBAL_FRAME: {GLOBAL_FRAME}")
        self.get_logger().info(f"BASE_FRAME: {BASE_FRAME}")
        self.get_logger().info(f"OUTPUT_FILE: {self.file_path}")
        self.get_logger().info(f"MIN_DISTANCE: {MIN_DISTANCE} m")
        self.get_logger().info(f"DEFAULT_SPEED: {DEFAULT_SPEED} m/s")

    def save_waypoint(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                GLOBAL_FRAME,
                BASE_FRAME,
                Time()
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"Cannot get TF {GLOBAL_FRAME} -> {BASE_FRAME}: {e}",
                throttle_duration_sec=2.0
            )
            return

        x = trans.transform.translation.x
        y = trans.transform.translation.y

        q = trans.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)

        if self.prev_x is not None:
            dist = math.hypot(x - self.prev_x, y - self.prev_y)
            if dist < MIN_DISTANCE:
                return

        self.writer.writerow([
            f"{x:.6f}",
            f"{y:.6f}",
            f"{yaw:.6f}",
            f"{DEFAULT_SPEED:.6f}"
        ])
        self.file.flush()

        self.prev_x = x
        self.prev_y = y
        self.num_saved += 1

        self.get_logger().info(
            f"[{self.num_saved}] x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}, speed={DEFAULT_SPEED:.2f}"
        )

    def shutdown(self):
        if hasattr(self, "file") and not self.file.closed:
            self.file.close()
            self.get_logger().info(f"Saved {self.num_saved} waypoints.")
            self.get_logger().info(f"File saved at: {self.file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = MapWaypointsLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
