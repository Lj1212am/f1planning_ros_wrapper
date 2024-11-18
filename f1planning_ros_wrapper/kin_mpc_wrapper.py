# MIT License

# Copyright (c) Hongrui Zheng, Johannes Betz

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
ROS Wrapper Implementation of the STMPC Waypoint Tracker Example

Author: Your Name
Last Modified: Date
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import sys
import os

# Add the path to the STMPC planner if necessary
sys.path.append('/home/nvidia/f1-fifth/src/f1planning_ros_wrapper/f1tenth_planning')

from f1tenth_planning.control.kinematic_mpc.kinematic_mpc import KMPCPlanner
from f1tenth_gym.envs.track import Track
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

# Number of waypoints to consider
WAYPOINT_NUM = 1000

class STMPCPlannerNode(Node):
    def __init__(self):
        super().__init__('stmpc_planner_node')

        # Configuration parameters
        self.real_car = False
        self.config_path = "/home/nvidia/f1-fifth/src/trajectory_csv"
        self.csv = "interpolated_trajectory_2.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1)

        x = self.waypoints[:, 1]
        y = self.waypoints[:, 2]
        v = self.waypoints[:, 5]

        # Initialize the track using the waypoints
        self.track = Track.from_refline(x[10:WAYPOINT_NUM], y[10:WAYPOINT_NUM], v[10:WAYPOINT_NUM])

        # Initialize the STMPC Planner
        self.planner = KMPCPlanner(track=self.track, debug=False)

        # Publishers and Subscribers
        drive_topic = '/drive'
        odom_topic = '/gnss_to_local/odometry' if self.real_car else '/ego_racecar/odom'

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.state_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints_markers', 10)

        # Timer for publishing waypoints as markers
        self.create_timer(0.1, self.publish_waypoints_as_markers)

        # Initializations
        self.current_state = None
        self.steering_angle = 0.0
        self.speed = 0.0

        # Waypoints for visualization
        waypointx = self.track.raceline.xs[:WAYPOINT_NUM]
        waypointy = self.track.raceline.ys[:WAYPOINT_NUM]
        waypointyaw = self.track.raceline.yaws[:WAYPOINT_NUM]
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))

    def state_callback(self, odom_msg):
        """
        Callback for Odometry updates, processes the current pose and sends it to the STMPC planner.
        """
        if self.real_car:
            position = odom_msg.pose.pose.position
            curr_quat = odom_msg.pose.pose.orientation
            yaw = math.atan2(2 * (curr_quat.w * curr_quat.z + curr_quat.x * curr_quat.y),
                             1 - 2 * (curr_quat.y ** 2 + curr_quat.z ** 2))
            linear_vel_x = -odom_msg.twist.twist.linear.x
            linear_vel_y = -odom_msg.twist.twist.linear.y
        else:
            position = odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            linear_vel_x = odom_msg.twist.twist.linear.x
            linear_vel_y = odom_msg.twist.twist.linear.y
            yaw = self.quaternion_to_euler(orientation)

        yaw_rate = odom_msg.twist.twist.angular.z

        # Slip angle calculation
        if linear_vel_x != 0:
            slip_angle = math.atan2(linear_vel_y, linear_vel_x)
        else:
            linear_vel_x = 1.0
            slip_angle = 0.0

        state_dict = {
            'pose_x': position.x,
            'pose_y': position.y,
            'delta': self.steering_angle,
            'linear_vel_x': linear_vel_x,
            'linear_vel_y': linear_vel_y,
            'pose_theta': yaw,
            'ang_vel_z': yaw_rate,
            'beta': slip_angle
        }

        # Plan using the STMPC planner
        try:
            steerv, accl = self.planner.plan(state_dict)
        except Exception as e:
            self.get_logger().error(f"Error in planner: {e}")
            steerv, accl = 0.0, 0.0

        # Update steering angle and speed
        dt = self.planner.config.DTK
        self.steering_angle += steerv * dt
        self.speed = linear_vel_x + accl * dt

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        self.pub_drive.publish(drive_msg)

    def publish_waypoints_as_markers(self):
        """Publish waypoints as visualization markers in RViz."""
        marker_array = MarkerArray()
        for i, waypoint in enumerate(self.waypoints[:WAYPOINT_NUM]):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i
            marker.scale.x = 1.0
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.pose.position.x = float(waypoint[0])
            marker.pose.position.y = float(waypoint[1])
            marker.pose.position.z = 0.2

            yaw = float(waypoint[2])
            q = self.yaw_to_quaternion(yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def quaternion_to_euler(self, orientation):
        """
        Convert quaternion (from Odometry) to yaw (Euler angle).
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y ** 2 + z ** 2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to a quaternion (x, y, z, w)."""
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]

def main(args=None):
    rclpy.init(args=args)
    node = STMPCPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
