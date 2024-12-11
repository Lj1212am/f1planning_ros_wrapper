import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np

import sys
import math
import os

sys.path.append('/home/lee/work/f1-fifth/src/f1planning_ros_wrapper/f1tenth_planning')

#NMPC Imports
from dataclasses import dataclass, field
from f1tenth_gym.envs.track import Track
import casadi as ca


from f1tenth_planning.control.nonlinear_mpc.nonlinear_frenet_dmpc import NMPCPlanner, mpc_config


# Ros2 imports
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
import message_filters
from visualization_msgs.msg import MarkerArray, Marker

# from 0 - 1000
waypoint_num = -10
class NMPCPlannerNode(Node):
    def __init__(self):
        super().__init__('nmpc_planner_node')
        
        self.real_car = False
        self.config_path = "/home/lee/work/f1-fifth/src/trajectory_csv"
        
        self.csv = "slalom_centerline.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        # self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1) 
        
        # self.waypoints[:, 3] += math.pi/2
        # self.sin_yaw = np.sin(self.waypoints[:, 3])
        # self.cos_yaw = np.cos(self.waypoints[:, 3])
        
        # x = self.waypoints[:, 1]
        # y = self.waypoints[:, 2]
        x = self.waypoints[:, 0] * 3.0
        y = self.waypoints[:, 1] * 3.0
        
        # move x and y to zero
        x = x - x[0]
        y = y - y[0]
        
        # velx = self.waypoints[:, 2]
        # vely = self.waypoints[:, 3]
        
        # v = np.sqrt(velx**2 + vely**2)
        # v = self.waypoints[:, 5] *3.0 /10.0
        v = np.ones_like(x) * 3.0
        
        
        # Now pass the processed x, y, and velx to the Track class
        # self.track = Track.from_refline(x[10:waypoint_num], y[10:waypoint_num], v[10:waypoint_num])
        self.track = Track.from_refline(x, y, v)
        # Initialize Subscribers / Publishers for the controller
        
        drive_topic = '/drive'
        if self.real_car:
            odom_topic = '/transformed/odometry'
            # odom_topic = '/gnss_to_local/odometry'
        else:
            odom_topic = '/ego_racecar/odom'

        self.initial_x = None
        self.initial_y = None
        
        if self.real_car:
            self.initial_x = 0.0 #1118.0
            self.initial_y = 0.0 #946.1487523074607

        # ackermann_sub = message_filters.Subscriber(self, AckermannDriveStamped, drive_topic)
        # odom_sub = message_filters.Subscriber(self, Odometry, odom_topic)
        
        # self.ts = message_filters.ApproximateTimeSynchronizer([ackermann_sub, odom_sub], 1, 0.1)
        # self.ts.registerCallback(self.state_callback)

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.state_callback, 1)
        
        
        # self.sub_odom = self.create_subscription(Odometry, odom_topic, self.state_callback, 1)
        self.sub_ackermann = self.create_subscription(AckermannDriveStamped, drive_topic, self.ackerman_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        # self.pub_mpc_sol = self.create_publisher(Marker, 'mpc_solution', 10)
        self.sub_mu = self.create_subscription(Float32, 'friction_value', self.friction_callback, 10) 
        # Publisher for visualizing waypoints as MarkerArray
        
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints_markers', 10)
        
        #Create a timer publisher for planning
        self.timer = self.create_timer(0.1, self.publish_control)
        
        
        # Initialize the NMPCPlanner with default parameters
        print('setting config')
        self.config = mpc_config()
        print('initing controlller')
        self.planner = NMPCPlanner(track = self.track, config=self.config, debug=False)
        print('finished initing controller')
        
        waypointx = self.track.raceline.xs[:waypoint_num]
        waypointy = self.track.raceline.ys[:waypoint_num]
        waypointyaw = self.track.raceline.yaws[:waypoint_num]
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))
        
        ##setup a timer callpacl to publish waypoints as markers
        self.timer = self.create_timer(1.0, self.publish_waypoints_as_markers)
        # self.publish_waypoints_as_markers(self.waypoints)
        
        self.old_steerv = 0.0
        self.old_accl = 0.0
        # Initialize placeholders
        self.current_state = None
        
        #soft start
        # accl = 9.0
        # steer = 0.0
        self.steering_angle = 0.0
        self.speed = 0.0
        self.mu = None
        
        # drive = AckermannDriveStamped()
        # # drive.drive.steering_angle = 0.3
        # drive.drive.speed = 1.0
        # for i in range(10):
        #     self.pub_drive.publish(drive)
        # self.pub_drive.publish(drive)
    
    def ackerman_callback(self, ackerman_msg):
        self.steering_angle = ackerman_msg.drive.steering_angle
        
    def friction_callback(self, mu_msg):
        self.mu = float(mu_msg.data)
    
    def render_mpc_sol(self):
        """
        Publish the MPC solution as a Marker in RViz.
        """
        
        ref_traj_frenet = self.planner.ref_path
        
        ref_traj_x = ref_traj_frenet[0,:]
        ref_traj_y = ref_traj_frenet[1,:]
        ref_traj_yaw = ref_traj_frenet[4,:]
       
        ref_waypoints = np.column_stack((ref_traj_x, ref_traj_y, ref_traj_yaw))
        
        self.publish_waypoints_as_markers(ref_waypoints, False)
        
        
        # if self.planner.ox is not None and self.planner.oy is not None:
        #                # Create a new marker
        #     marker = Marker()
        #     marker.header.frame_id = "map"  # Adjust this to match your RViz frame
        #     marker.header.stamp = self.get_clock().now().to_msg()
        #     marker.ns = "mpc_solution"
        #     marker.id = 0
        #     marker.type = Marker.ARROW  # Render as a line connecting the points
        #     marker.action = Marker.ADD
        #     marker.scale.x = 1.1  # Line width
        #     marker.scale.y = 1.1  # Line width
        #     marker.scale.z = 1.1  # Line width
        #     marker.color.a = 1.0  # Alpha (transparency)
        #     marker.color.r = 0.0
        #     marker.color.g = 0.0
        #     marker.color.b = 1.0  # Blue color

        #        # Set the positions of the start and end points
        #     start = Point()
                            
        #     quaternion = self.yaw_to_quaternion(psi)
        #     marker.pose.orientation.x = quaternion[0]
        #     marker.pose.orientation.y = quaternion[1]
        #     marker.pose.orientation.z = quaternion[2]
        #     marker.pose.orientation.w = quaternion[3]
            

        #     # Publish the marker to render in RViz
        #     self.pub_mpc_sol.publish(marker)
    
    def publish_control(self):
        
        pass

    def gnss_callback(self, nav_msg):
        self.gnss_vel_x = -nav_msg.twist.twist.linear.x
        self.gnss_vel_y = -nav_msg.twist.twist.linear.y

        print("gnss speed: ", self.gnss_speed, "drive topic speed: ", self.drive_msg.drive.speed)

        
    
    # def state_callback(self, ackerman_msg, odom_msg):
    def state_callback(self, odom_msg):
        """
        Callback for Odometry updates, processes the current pose and sends it to the NMPC planner.
        """
        if self.real_car:
            position = odom_msg.pose.pose.position
            position.x = position.x - self.initial_x
            position.y = position.y - self.initial_y
            curr_quat = odom_msg.pose.pose.orientation
            yaw = math.atan2(2 * (curr_quat.w * curr_quat.z + curr_quat.x * curr_quat.y),
                              1 - 2 * (curr_quat.y ** 2 + curr_quat.z ** 2))
            yaw += math.pi
            linear_vel_x = -1 *odom_msg.twist.twist.linear.x
            linear_vel_y = -1 * odom_msg.twist.twist.linear.y # Tomorrow we will fix here
        else:
            # steering_angle = ackerman_msg.drive.steering_angle
            # Extract the pose from the Odometry message
            position = odom_msg.pose.pose.position

            orientation = odom_msg.pose.pose.orientation
        
            linear_vel_x = odom_msg.twist.twist.linear.x
            linear_vel_y = odom_msg.twist.twist.linear.y

        # Convert quaternion to Euler angles for yaw
            yaw = self.quaternion_to_euler(orientation)


        yaw_rate = odom_msg.twist.twist.angular.z
        # if yaw_rate < 2.0:
        #     yaw_rate = 0.0
        
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
        # # if linear velocity < 1 set it greater than 1 else accel is 9 and steerv is 0
        # if linear_vel_x < 0.1 and not self.real_car:
        #     accl = 9.0
        #     steerv = 1.0
        # else:
        # Plan using the NMPC planner
        print('abt to plan so hard')
        
        try:
            accl, steerv = self.planner.plan(state_dict, self.mu)
            # # TODO convert back to cartesian
            # for i, (s, ey) in enumerate(zip(self.planner.ox, self.planner.oy)):
            #     curr_x, curr_y, _ = self.planner.track.frenet_to_cartesian(s, ey, 0.0, use_raceline=True)
            #     self.planner.ox[i] = curr_x
            #     self.planner.oy[i] = curr_y
            print('done planning')
            # self.render_mpc_sol()
        except Exception as e:
            print('error planning', e)
                
            
        
        # integrate steerv to get steering angle and integrate accl to get speed us dt =0.1
        dt = self.planner.config.DTK
        # print('dt', dt)
        self.steering_angle = self.steering_angle + steerv * dt
        linear_vel_x = linear_vel_x + accl * dt
        self.speed = linear_vel_x
        
        # print('steering angle', self.steering_angle, 'speed', self.speed)

        # Publish the drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = self.speed
        drive_msg.drive.steering_angle = self.steering_angle
        # if linear_vel_x < 0.1:
        self.pub_drive.publish(drive_msg)

        
        
    
    def publish_waypoints_as_markers(self, waypoints=None, ref=False):
        """ Publish waypoints as visualization markers in RViz """
        marker_array = MarkerArray()
        
        if waypoints is None:
            waypoints = self.waypoints

        # Create markers for first 20 waypoints
        # print('num waypoints', len(waypoints))
        for i, waypoint in enumerate(waypoints[:waypoint_num]):
        # for i, waypoint in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"  # Set appropriate frame ID
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i  # Each marker needs a unique ID
            marker.id = i + 1000  # Each marker needs a unique ID
            marker.scale.x = 1.0  # Arrow length
            marker.scale.y = 0.2  # Arrow width
            marker.scale.z = 0.2  # Arrow height
            if ref:
                marker.id = i + 1000  # Each marker needs a unique ID
                marker.scale.x = 1.0  # Arrow length
                marker.scale.y = 0.2  # Arrow width
                marker.scale.z = 0.2  # Arrow height

            # Set waypoint positions and orientations
            marker.pose.position.x = float(waypoint[0])  # x-coordinate
            marker.pose.position.y = float(waypoint[1])  # y-coordinate
            marker.pose.position.z = 0.2  # z-coordinate (flat 2D track)

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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
