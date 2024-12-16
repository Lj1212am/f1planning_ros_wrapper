import csv
import signal
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np

import sys
import math
import os

sys.path.append('/home/nvidia/ros_ws/src/f1-fifth/src/f1planning_ros_wrapper/f1tenth_planning')

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
from PyQt5.QtWidgets import QApplication
from pyqtgraph.Qt import QtCore
import pyqtgraph as pg
import threading
import matplotlib.pyplot as plt
from matplotlib import cm, colormaps
from matplotlib.colors import Normalize

# from 0 - 1000
waypoint_num = -1
class NMPCPlannerNode(Node):
    def __init__(self):
        super().__init__('nmpc_planner_node')
        self.plot = False
        self.real_car = True
        # Declare a ROS parameter for the output CSV file name suffix
        self.declare_parameter('csv_suffix', 'nmpc')

        self.config_path = "/home/nvidia/ros_ws/src/f1-fifth/src/trajectory_csv"
        
        self.csv = "rotated_raceline_slalom_wide.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1) 
        # self.waypoints = np.loadtxt(self.map_name, delimiter=',', skiprows=1) 
        
        # self.waypoints[:, 3] += math.pi/2
        # self.sin_yaw = np.sin(self.waypoints[:, 3])
        # self.cos_yaw = np.cos(self.waypoints[:, 3])
        
        x = self.waypoints[:, 1]  #* 2.0#+ 1.2
        y = self.waypoints[:, 2]  #* 2.0#  1.1
        # x = self.waypoints[:, 0] * 3.0
        # y = self.waypoints[:, 1] * 3.0
        # velx = self.waypoints[:, 2]
        # vely = self.waypoints[:, 3]
        
        # v = np.sqrt(velx**2 + vely**2)
        # v = self.waypoints[:, 5]
        v = np.ones_like(x)  * 2.0
        
        
        # Now pass the processed x, y, and velx to the Track class
        # self.track = Track.from_refline(x[10:waypoint_num], y[10:waypoint_num], v[10:waypoint_num])
        self.track = Track.from_refline(x, y, v)
        # Initialize Subscribers / Publishers for the controller

        # Get the CSV suffix from the parameter
        csv_suffix = self.get_parameter('csv_suffix').get_parameter_value().string_value

        # Construct the output CSV file name
        self.csv_path = "/home/nvidia/ros_ws/src/f1-fifth/src/f1planning_ros_wrapper/real_world_results"
        self.output_csv_file = f"cross_track_error_log_{csv_suffix}.csv"

        self.output_csv_path = os.path.join(self.csv_path, self.output_csv_file)
        
        # Open the CSV file and write the header
        self.csv_file = open(self.output_csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "cross_track_error", "current_velocity", "goal_velocity"])
        self.get_logger().info(f'Initialized CSV logging at {self.output_csv_path}')
        
        drive_topic = '/drive'
        if self.real_car:
            odom_topic = '/transformed/odometry'
            # odom_topic = '/gnss_to_local/odometry'
        else:
            odom_topic = '/ego_racecar/odom'

        self.initial_x = None #0.0 #None
        self.initial_y = None#0.0 #None
        
        # if self.real_car:
        #     self.initial_x = 0.0 #1118.0
        #     self.initial_y = 0.0 #946.1487523074607

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

        # Initialize the NMPCPlanner with default parameters
        print('setting config')
        self.config = mpc_config()
        print('initing controlller')
        self.planner = NMPCPlanner(track = self.track, config=self.config, debug=False)
        
        
        waypointx = self.track.raceline.xs[:waypoint_num]
        waypointy = self.track.raceline.ys[:waypoint_num]
        waypointyaw = self.track.raceline.yaws[:waypoint_num]
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))
        WAYPOINTS_SUBSAMPLE_STEP = 10
        self.waypoints = self.waypoints[::WAYPOINTS_SUBSAMPLE_STEP, :]
        
        
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


        if self.plot:
           
            # Initialize PyQtGraph
            pg.setConfigOption('background', 'w')
            pg.setConfigOption('foreground', 'k')
            
            # self.app = QApplication([])  # Ensure QApplication is created in the main thread
            print('finished initing pqt')
            self.win = pg.GraphicsLayoutWidget(show=True, title="NMPC Position Tracking")
            self.plot = self.win.addPlot(title="Trajectory and Waypoints")
            self.plot.enableAutoRange('xy', True)
            self.plot.setAspectLocked(True)
           


            self.waypoints_plot = pg.ScatterPlotItem(size=5, brush=pg.mkBrush(0, 0, 255), name="Waypoints")
            self.trajectory_plot = pg.PlotCurveItem(pen=pg.mkPen('r', width=2), name="Trajectory")
            self.current_location_plot = pg.ScatterPlotItem(size=10, brush=pg.mkBrush(0, 255, 0), name="Current Location")
            self.predict_traj_plot = pg.PlotCurveItem(pen=pg.mkPen('b', style=QtCore.Qt.DashLine, width=1.5), name="Predicted Trajectory")
            
            self.plot.addItem(self.waypoints_plot)
            self.plot.addItem(self.trajectory_plot)
            self.plot.addItem(self.current_location_plot)
            self.plot.addItem(self.predict_traj_plot)

            self.waypoints_plot.setData([{'pos': (wp[1], wp[2]), 'data': 1} for wp in self.waypoints])
            self.points = []
            self.current_point = []

            self.update_timer = QtCore.QTimer()
            self.update_timer.timeout.connect(self.update_plot)
            self.update_timer.start(50)

            self.legend = self.plot.addLegend()
            self.legend.addItem(self.waypoints_plot, 'Waypoints')
            self.legend.addItem(self.trajectory_plot, 'Trajectory')
            self.legend.addItem(self.current_location_plot, 'Current Position')
            # Add predicted and reference trajectories to the legend
            self.legend.addItem(self.predict_traj_plot, 'Predicted Trajectory')
        print('finished initing controller')

    def update_plot(self):
        if self.points:
            points_array = np.array(self.points)
            self.trajectory_plot.setData(points_array[:, 0], points_array[:, 1])
        if self.current_point:
            self.current_location_plot.setData([{'pos': self.current_point[0], 'data': 1}])

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
        self.ref_traj_plot.setData(ref_traj_x, ref_traj_y)
        
    def state_callback(self, odom_msg):
        """
        Callback for Odometry updates, processes the current pose and sends it to the NMPC planner.
        """

        print('state callback')
        if self.real_car:
            if self.initial_x==None or self.initial_y==None:
                self.initial_x = odom_msg.pose.pose.position.x
                self.initial_y = odom_msg.pose.pose.position.y
                print(f"Initial pose on real car: {self.initial_x}, {self.initial_y}")
            
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
        
        try:
            accl, steerv = self.planner.plan(state_dict, self.mu)
            # self.render_mpc_sol()

            # Log the cross-track error to the CSV file
            current_time = self.get_clock().now().to_msg()
            timestamp = f"{current_time.sec}.{current_time.nanosec}"
            cross_track_error = self.planner.ey  # Access the cross-track error
            current_velocity = self.planner.curr_vel  # Access the current velocity
            goal_velocity = self.planner.goal_vel  # Access the goal velocity
            self.csv_writer.writerow([timestamp, cross_track_error, current_velocity, goal_velocity])
            self.csv_file.flush()  # Ensure data is written to disk
            self.get_logger().info(f'Logged data: CTE={cross_track_error}, Curr_Vel={current_velocity}, Goal_Vel={goal_velocity}')
        
        except Exception as e:
            self.get_logger().error(f'Error in planning: {e}')
                
            
        
        # integrate steerv to get steering angle and integrate accl to get speed us dt =0.1
        dt = self.planner.config.DTK
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

        # Visualization logic
        if self.plot:
            # Append current car position to trajectory points
            self.points.append((position.x, position.y))

            # Update real trajectory
            points_array = np.array(self.points)
            self.trajectory_plot.setData(points_array[:, 0], points_array[:, 1])
        
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

    nmpc_node = NMPCPlannerNode()

    if nmpc_node.plot:
        # Handle Ctrl+C for clean shutdown
        def handle_interrupt(signal, frame):
            print("Ctrl+C detected, shutting down...")
            QApplication.instance().quit()
            rclpy.shutdown()

        signal.signal(signal.SIGINT, handle_interrupt)

        # Start ROS spinning in a separate thread
        executor_thread = threading.Thread(target=rclpy.spin, args=(nmpc_node,), daemon=True)
        executor_thread.start()

        QApplication.instance().exec_()
    else:
        rclpy.spin(nmpc_node)

    # Clean up
    nmpc_node.destroy_node()
    rclpy.shutdown()
    if nmpc_node.plot:
        executor_thread.join()


if __name__ == '__main__':
    main()