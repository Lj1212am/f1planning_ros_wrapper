import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
import math
import os
# NMPC Imports
from dataclasses import dataclass, field
from f1tenth_gym.envs.track import Track
import casadi as ca

import sys


sys.path.append('/home/rajnish/ros2_ws/src/f1planning_ros_wrapper/f1tenth_planning')

from f1tenth_planning.control.nonlinear_mpc.nonlinear_frenet_dmpc import NMPCPlanner, mpc_config
# ROS2 message and service imports
from nav_msgs.msg import Odometry
from autoware_auto_vehicle_msgs.msg import GearCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand, AckermannLateralCommand, LongitudinalCommand
from autoware_auto_vehicle_msgs.srv import ControlModeCommand
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Time
# Number of waypoints to process
waypoint_num = 1000
class NMPCPlannerNode(Node):
    def __init__(self):
        super().__init__('nmpc_planner_node')
        self.real_car = True
        self.config_path = "/home/rajnish/ros2_ws/src/trajectory_csv/"
        self.csv = "interpolated_trajectory_1.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1)
        x = self.waypoints[:, 1]*10.0
        y = self.waypoints[:, 2]*10.0
        v = self.waypoints[:, 5]
        # Initialize the track using waypoints
        self.track = Track.from_refline(x[10:waypoint_num], y[10:waypoint_num], v[10:waypoint_num])
        drive_topic = '/control/command/control_cmd'
        if self.real_car:
            odom_topic = '/awsim/ground_truth/localization/kinematic_state'
        else:
            odom_topic = '/ego_racecar/odom'
        # Create the QoS profile with Best Effort reliability, Volatile durability, and Keep Last history
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        gear_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,  # Reliable QoS policy
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Transient Local Durability policy
            history=HistoryPolicy.KEEP_LAST,  # Keep Last History policy
            depth=1  # Buffer size for the history
        )
        # qos_profile = QoSProfile.history_system_default()
        # Initialize subscribers and publishers with the QoS profile
        self.sub_odom = self.create_subscription(
            Odometry, odom_topic, self.state_callback, qos_profile)
        self.sub_ackermann = self.create_subscription(
            AckermannControlCommand, drive_topic, self.ackerman_callback, qos_profile)
        self.pub_drive = self.create_publisher(
            AckermannControlCommand, drive_topic, gear_qos_profile)
        self.pub_mpc_sol = self.create_publisher(
            Marker, 'mpc_solution', qos_profile)
        self.pub_gear = self.create_publisher(
            GearCommand, '/control/command/gear_cmd', gear_qos_profile)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'waypoints_markers', qos_profile)
        self.sub_mu = self.create_subscription(Float32, 'friction_value', self.friction_callback, 10) 
        # Create a timer for planning (adjust the rate as needed)
        self.timer = self.create_timer(0.1, self.publish_control)
        # Initialize the NMPCPlanner with default parameters
        self.get_logger().info('Setting up NMPC Planner')
        self.config = mpc_config()
        self.planner = NMPCPlanner(track=self.track, config=self.config, debug=False)
        self.get_logger().info('NMPC Planner initialized')
        # Prepare waypoints for visualization
        waypointx = self.track.raceline.xs[:waypoint_num]
        waypointy = self.track.raceline.ys[:waypoint_num]
        waypointyaw = self.track.raceline.yaws[:waypoint_num]
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))
        self.publish_waypoints_as_markers(self.waypoints)
        # Initialize control variables
        self.steering_angle = 0.0
        self.speed = 0.0
        self.mu = None
        # Publish the gear command to set the vehicle in drive mode
        gear_cmd = GearCommand()
        gear_cmd.stamp = self.get_clock().now().to_msg()
        gear_cmd.command = GearCommand.DRIVE
        self.pub_gear.publish(gear_cmd)
        self.get_logger().info('Published GearCommand: GEAR_DRIVE')
        # Create a service client for ControlModeCommand
        self.control_mode_client = self.create_client(ControlModeCommand, '/vehicle/control_mode_command')
        # while not self.control_mode_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Service not available, waiting again...')
        # Switch to autonomous mode
        self.switch_to_autonomous_mode()

    def switch_to_autonomous_mode(self):
        # Create the request
        request = ControlModeCommand.Request()
        request.stamp = self.get_clock().now().to_msg()
        request.mode = ControlModeCommand.Request.AUTONOMOUS  # Mode value is 1
        # Call the service
        future = self.control_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Switched to autonomous mode.')
            else:
                self.get_logger().warn('Failed to switch to autonomous mode.')
        else:
            self.get_logger().error('Service call failed or returned None.')
    
    def friction_callback(self, mu_msg):
        self.mu = float(mu_msg.data)
    
    def ackerman_callback(self, ackerman_msg):
        # Update the steering angle from the received message
        self.steering_angle = ackerman_msg.lateral.steering_tire_angle
    
    def render_mpc_sol(self):
        # Implement visualization of MPC solution if needed
        pass
    
    def publish_control(self):
        # Implement control publishing logic if needed
        pass
    
    def state_callback(self, odom_msg):
        # Extract the pose from the Odometry message
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        linear_vel_x = odom_msg.twist.twist.linear.x
        linear_vel_y = odom_msg.twist.twist.linear.y
        # Convert quaternion to Euler angles for yaw
        yaw = self.quaternion_to_euler(orientation)
        yaw_rate = odom_msg.twist.twist.angular.z
        # Slip angle = arctan(vy / vx)
        if linear_vel_x != 0:  # Avoid division by zero
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
        accl = 0.0
        steerv = 0.0
        # Plan using the NMPC planner
        try:
            accl, steerv = self.planner.plan(state_dict, self.mu)
            self.render_mpc_sol()
        except Exception as e:
            self.get_logger().error(f'Error in planning: {e}')
        # Integrate steerv to get steering angle and integrate accl to get speed
        dt = self.planner.config.DTK
        steering_angle = self.steering_angle + steerv * dt
        linear_vel_x = linear_vel_x + accl * dt
        self.speed = linear_vel_x
        # Publish the drive command
        drive_msg = AckermannControlCommand()
        drive_msg.stamp = self.get_clock().now().to_msg()
        # Set the lateral command
        drive_msg.lateral.stamp = drive_msg.stamp
        drive_msg.lateral.steering_tire_angle = steering_angle
        drive_msg.lateral.steering_tire_rotation_rate = 0.0  # Set as appropriate
        # Set the longitudinal command
        drive_msg.longitudinal.stamp = drive_msg.stamp
        drive_msg.longitudinal.speed = self.speed
        drive_msg.longitudinal.acceleration = accl  # Set as appropriate
        drive_msg.longitudinal.jerk = 0.0  # Set as appropriate

        gear_cmd = GearCommand()
        gear_cmd.stamp = self.get_clock().now().to_msg()
        gear_cmd.command = GearCommand.DRIVE
        self.pub_gear.publish(gear_cmd)
        # Publish the message
        self.pub_drive.publish(drive_msg)
        self.get_logger().info(f'Published drive command with speed: {self.speed}, steering_angle: {steering_angle}')
        # Update the steering angle
        self.steering_angle = steering_angle

    def publish_waypoints_as_markers(self, waypoints=None, ref=False):
        """ Publish waypoints as visualization markers in RViz """
        marker_array = MarkerArray()
        # Create markers for waypoints
        for i, waypoint in enumerate(waypoints[:waypoint_num]):
            marker = Marker()
            marker.header.frame_id = "map"  # Set appropriate frame ID
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i  # Each marker needs a unique ID
            marker.scale.x = 1.0  # Arrow length
            marker.scale.y = 0.2  # Arrow width
            marker.scale.z = 0.2  # Arrow height
            # Set waypoint positions and orientations
            marker.pose.position.x = float(waypoint[0])  # x-coordinate
            marker.pose.position.y = float(waypoint[1])  # y-coordinate
            marker.pose.position.z = 0.2  # z-coordinate
            # Convert yaw to quaternion for orientation
            yaw = float(waypoint[2])  # yaw angle
            q = self.yaw_to_quaternion(yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            # Set the color of the marker
            if ref:
                marker.color.a = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.a = 1.0  # Alpha
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            # Append to the marker array
            marker_array.markers.append(marker)
        # Publish the MarkerArray
        self.marker_pub.publish(marker_array)
    
    def quaternion_to_euler(self, orientation):
        """
        Convert quaternion (from Odometry) to yaw (Euler angle).
        """
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        # Yaw calculation
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """ Convert yaw angle to a quaternion (x, y, z, w) """
    
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]
def main(args=None):
    rclpy.init(args=args)
    node = NMPCPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()