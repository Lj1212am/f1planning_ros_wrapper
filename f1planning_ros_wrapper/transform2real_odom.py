#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class OdometryTransformer(Node):
    def __init__(self):
        super().__init__('odometry_transformer')

        # Parameters for initial position
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value

        # Subscriber to the /ego_racecar/odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)

        # Publisher to the /transformed/odometry topic
        self.publisher_ = self.create_publisher(
            Odometry,
            '/transformed/odometry',
            10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/transformed/pose',
            10

        )
        # Timer for publishing at a fixed rate (30 Hz)
        self.publish_rate_hz = 30.0
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        # Store the last transformed message
        self.last_transformed_msg = None
        self.last_pose_msg = None

    def odom_callback(self, msg):
        # Transform the incoming odometry message
        transformed_msg = Odometry()
        transformed_msg.header = msg.header
        transformed_msg.child_frame_id = msg.child_frame_id

        # Transform position
        transformed_msg.pose.pose.position.x = msg.pose.pose.position.x - self.initial_x
        transformed_msg.pose.pose.position.y = msg.pose.pose.position.y - self.initial_y
        transformed_msg.pose.pose.position.z = msg.pose.pose.position.z

        # Transform orientation
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Adjust yaw
        yaw += math.pi

        # Normalize yaw to [-pi, pi]
        yaw = (yaw + math.pi) % (2 * math.pi) - math.pi

        # Convert yaw back to quaternion
        new_q = self.yaw_to_quaternion(yaw)
        transformed_msg.pose.pose.orientation = new_q


        # Transform linear velocities
        transformed_msg.twist.twist.linear.x = -msg.twist.twist.linear.x
        transformed_msg.twist.twist.linear.y = -msg.twist.twist.linear.y
        transformed_msg.twist.twist.linear.z = msg.twist.twist.linear.z

        # Copy angular velocities
        transformed_msg.twist.twist.angular = msg.twist.twist.angular

        # Copy covariance if needed
        transformed_msg.pose.covariance = msg.pose.covariance
        transformed_msg.twist.covariance = msg.twist.covariance

        # Store the transformed message
        self.last_transformed_msg = transformed_msg

        # Create and store the PoseWithCovarianceStamped message
        pose_cov_stmp = PoseWithCovarianceStamped()
        pose_cov_stmp.header = transformed_msg.header
        pose_cov_stmp.pose = transformed_msg.pose
        self.last_pose_msg = pose_cov_stmp

    def timer_callback(self):
        # Publish the last transformed message at a fixed rate
        if self.last_transformed_msg is not None:
            # Update the timestamp
            self.last_transformed_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.last_transformed_msg)
            self.pose_pub.publish(self.last_pose_msg)

            # Publish the last PoseWithCovarianceStamped message
        if self.last_pose_msg is not None:
            self.last_pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_pub.publish(self.last_pose_msg)
        else:
            # No message has been received yet; you may choose to log or handle this case
            pass

    def yaw_to_quaternion(self, yaw):
        # Converts yaw angle to quaternion
        half_yaw = yaw * 0.5
        q = Odometry().pose.pose.orientation
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

def main(args=None):
    rclpy.init(args=args)
    odometry_transformer = OdometryTransformer()
    rclpy.spin(odometry_transformer)
    odometry_transformer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
