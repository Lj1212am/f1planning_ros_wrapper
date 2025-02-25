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

sys.path.append('/home/lee/work/f1-fifth/src/f1planning_ros_wrapper/f1tenth_planning')

# NMPC Imports
from dataclasses import dataclass, field
from f1tenth_gym.envs.track import Track
import casadi as ca

from f1tenth_planning.control.nonlinear_mpc.nonlinear_dmpc import NMPCPlanner, mpc_config

# Ros2 imports
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
import message_filters
from visualization_msgs.msg import MarkerArray, Marker
# from PyQt5.QtWidgets import QApplication
# from pyqtgraph.Qt import QtCore
# import pyqtgraph as pg
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
        self.real_car = False
        # Declare a ROS parameter for the output CSV file name suffix
        self.declare_parameter('csv_suffix', 'nmpc')

        self.config_path = "/home/lee/work/f1-fifth/src/trajectory_csv"
        self.csv = "rotated_safe_slalom.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1)
        
        clark_park_origin_x = 0.0
        clark_park_origin_y = 0.0 
        x = self.waypoints[:, 1] * 1.7 + clark_park_origin_x
        y = self.waypoints[:, 2] + clark_park_origin_y
        v = np.ones_like(x) * 6.0
        
        self.track = Track.from_refline(x, y, v)
        
        # CSV logging initialization
        csv_suffix = self.get_parameter('csv_suffix').get_parameter_value().string_value
        self.csv_path = "/home/lee/work/f1-fifth/src/f1planning_ros_wrapper/real_world_results"
        self.output_csv_file = f"cross_track_error_log_{csv_suffix}.csv"
        self.output_csv_path = os.path.join(self.csv_path, self.output_csv_file)
        self.csv_file = open(self.output_csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["timestamp", "cross_track_error", "current_velocity", "goal_velocity", "x_m", "y_m"])
        self.get_logger().info(f'Initialized CSV logging at {self.output_csv_path}')
        
        drive_topic = '/drive'
        if self.real_car:
            odom_topic = '/gnss_to_local/odometry'
        else:
            odom_topic = '/ego_racecar/odom'

        self.initial_x = None
        self.initial_y = None

        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.state_callback, 1)
        self.sub_ackermann = self.create_subscription(AckermannDriveStamped, drive_topic, self.ackerman_callback, 1)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, drive_topic, 1)
        self.pub_mpc_sol = self.create_publisher(Marker, 'mpc_solution', 10)
        self.sub_mu = self.create_subscription(Float32, 'friction_value', self.friction_callback, 10) 

        self.get_logger().info('Setting NMPC configuration')
        self.config = mpc_config()
        self.get_logger().info('Initializing NMPC controller')
        self.planner = NMPCPlanner(track=self.track, config=self.config, debug=False)
        self.planner.config.dlk = (
            self.track.raceline.ss[1] - self.track.raceline.ss[0]
        )  # waypoint spacing
        
        waypointx = self.track.raceline.xs[:waypoint_num]
        waypointy = self.track.raceline.ys[:waypoint_num]
        waypointyaw = self.track.raceline.yaws[:waypoint_num]
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))
        WAYPOINTS_SUBSAMPLE_STEP = 10
        self.waypoints = self.waypoints[::WAYPOINTS_SUBSAMPLE_STEP, :]
        
        self.marker_pub = self.create_publisher(MarkerArray, 'marker_mpc_sol', 1)
        self.ref_pub = self.create_publisher(MarkerArray, 'marker_ref_path', 1)

        self.old_steerv = 0.0
        self.old_accl = 0.0
        self.current_state = None
        self.first_call = True
        self.pub_pose = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        if not self.real_car:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.pub_drive.publish(drive_msg)

        self.steering_angle = 0.0
        self.speed = 0.0
        self.mu = None

        # === NEW: Shared control plan variables for continuous 100 Hz publishing ===
        # These will hold the full control plan (5 commands, each for 0.1 s) as a list of tuples.
        self.control_plan = None           
        self.control_plan_index = 0        
        self.control_plan_elapsed = 0.0      
        self.control_lock = threading.Lock()  
        # A timer that fires every 0.01 s (100 Hz) to publish drive commands.
        self.control_timer = self.create_timer(0.01, self.control_publish_cb)
        # ==========================================================================

        if self.plot:
            pg.setConfigOption('background', 'w')
            pg.setConfigOption('foreground', 'k')
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
            self.legend.addItem(self.predict_traj_plot, 'Predicted Trajectory')
        self.get_logger().info('Finished initializing controller')
        
        self.reference_marker_array = self._init_marker_array(self.config.TK + 1, color=(0.0, 1.0, 0.0))
        self.solution_marker_array = self._init_marker_array(self.config.TK + 1, color=(1.0, 0.0, 0.0))

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
        if self.planner.ox is not None and self.planner.oy is not None:
            # x_arr, y_arr = [], []
            # for (s, ey) in zip(self.planner.ox, self.planner.oy):
            #     x, y, _ = self.planner.track.frenet_to_cartesian(s, ey, 0.0, use_raceline=False)
            #     x_arr.append(x)
            #     y_arr.append(y)
            # x_arr = np.array(x_arr)
            # y_arr = np.array(y_arr)
            # points = np.array([x_arr, y_arr]).T
            points = np.array([self.planner.ox, self.planner.oy]).T
            self.publish_waypoints_as_markers(points, False)
    
    def render_mpc_ref(self):
        """
        Publish the reference trajectory as a Marker in RViz.
        """
        if self.planner.ref_path is not None:
            points = self.planner.ref_path[:2, :].T
            self.publish_waypoints_as_markers(points, True)
            
    def state_callback(self, odom_msg):
        """
        Callback for Odometry updates: processes the current pose and computes a new control plan.
        Instead of integrating and publishing commands directly here, we update the shared control plan.
        """
        if self.real_car:
            if self.initial_x is None or self.initial_y is None:
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
            linear_vel_x = -1 * odom_msg.twist.twist.linear.x
            linear_vel_y = -1 * odom_msg.twist.twist.linear.y
        else:
            position = odom_msg.pose.pose.position
            orientation = odom_msg.pose.pose.orientation
            linear_vel_x = odom_msg.twist.twist.linear.x
            linear_vel_y = odom_msg.twist.twist.linear.y
            yaw = self.quaternion_to_euler(orientation)
        yaw_rate = odom_msg.twist.twist.angular.z
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
        
        # For very slow speeds, use a default safe control plan.
        if linear_vel_x < 1.0 and not self.real_car:
            default_plan = [(9.0, 0.0)] * 5
            with self.control_lock:
                self.control_plan = default_plan
                self.control_plan_index = 0
                self.control_plan_elapsed = 0.0
        else:
            try:
                # Get the full control plan from the NMPC solver.
                # (The updated solver now returns arrays for acceleration and steering rate.)
                oa, odelta_v = self.planner.plan(state_dict, self.mu)
                self.render_mpc_sol()
                self.render_mpc_ref()
                
                # Log cross-track error and velocities.
                current_time = self.get_clock().now().to_msg()
                timestamp = f"{current_time.sec}.{current_time.nanosec}"
                cross_track_error = 0.0 # self.planner.ey
                current_velocity = self.planner.curr_vel
                goal_velocity = self.planner.goal_vel
                # self.csv_writer.writerow([timestamp, cross_track_error, current_velocity, goal_velocity, position.x, position.y])
                # self.csv_file.flush()
                self.get_logger().info(f'Logged data: CTE={cross_track_error}, Curr_Vel={current_velocity}, Goal_Vel={goal_velocity}')
                
                # Overwrite the control plan with the new plan.
                control_plan = list(zip(oa, odelta_v))
                with self.control_lock:
                    self.control_plan = control_plan
                    self.control_plan_index = 0
                    self.control_plan_elapsed = 0.0
            except Exception as e:
                self.get_logger().error(f'Error in planning: {e}')
        
        # Visualization: update trajectory.
        if self.plot:
            self.points.append((position.x, position.y))
            points_array = np.array(self.points)
            self.trajectory_plot.setData(points_array[:, 0], points_array[:, 1])
    
    def control_publish_cb(self):
        """
        Timer callback running at 100 Hz (every 0.01 s). It integrates the current control command
        from the stored control plan over each 0.01 s timestep and steps to the next command every 0.1 s.
        If a new plan is computed before finishing the old one, it is safely overwritten.
        """
        dt = 0.01  # 100 Hz period
        with self.control_lock:
            if self.control_plan is not None:
                current_command = self.control_plan[self.control_plan_index]
            else:
                current_command = (0.0, 0.0)
            current_accl, current_steerv = current_command
            self.steering_angle += current_steerv * dt
            self.speed += current_accl * dt
            if self.control_plan is not None:
                self.control_plan_elapsed += dt
                if self.control_plan_elapsed >= 0.1:
                    self.control_plan_index += 1
                    self.control_plan_elapsed = 0.0
                    if self.control_plan_index >= len(self.control_plan):
                        self.control_plan_index = len(self.control_plan) - 1
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = self.speed
            drive_msg.drive.steering_angle = self.steering_angle
        self.pub_drive.publish(drive_msg)
    
    def _init_marker_array(self, num_markers, color=(1.0, 0.0, 1.0)):
        marker_array = MarkerArray()
        for i in range(num_markers):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.id = i
            marker.scale.x = 1.0
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.2
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker_array.markers.append(marker)
        return marker_array
    
    def _update_marker_array(self, marker_array, points):
        num_update = len(points)
        if(len(marker_array.markers) < len(points)):
            num_update = len(marker_array.markers)
            
        for i in range(num_update):
            marker = marker_array.markers[i]
            marker.pose.position.x = points[i][0]
            marker.pose.position.y = points[i][1]
        for i in range(num_update, len(marker_array.markers)):
            marker = marker_array.markers[i]
            marker.action = Marker.DELETE
            
        return marker_array

    def publish_waypoints_as_markers(self, waypoints=None, ref=False):
        """ Publish waypoints as visualization markers in RViz """
        if waypoints is None:
            return
        if ref:
            marker_array = self.reference_marker_array
        else:
            marker_array = self.solution_marker_array
        marker_array = self._update_marker_array(marker_array, waypoints)
        if ref:
            self.ref_pub.publish(marker_array)
        else:
            self.marker_pub.publish(marker_array)
    
    def quaternion_to_euler(self, orientation):
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]


def main(args=None):
    rclpy.init(args=args)
    nmpc_node = NMPCPlannerNode()
    if nmpc_node.plot:
        def handle_interrupt(signal, frame):
            print("Ctrl+C detected, shutting down...")
            QApplication.instance().quit()
            rclpy.shutdown()
        signal.signal(signal.SIGINT, handle_interrupt)
        executor_thread = threading.Thread(target=rclpy.spin, args=(nmpc_node,), daemon=True)
        executor_thread.start()
        QApplication.instance().exec_()
    else:
        rclpy.spin(nmpc_node)
    nmpc_node.destroy_node()
    rclpy.shutdown()
    if nmpc_node.plot:
        executor_thread.join()


if __name__ == '__main__':
    main()
