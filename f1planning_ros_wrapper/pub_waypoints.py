
import time
import rclpy
from rclpy.node import Node
import numpy as np

import sys
import os

sys.path.append('/home/nvidia/ros_ws/src/f1-fifth/src/f1planning_ros_wrapper/f1tenth_planning')

#NMPC Imports
from f1tenth_gym.envs.track import Track

# Ros2 imports
from visualization_msgs.msg import MarkerArray, Marker

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('nmpc_planner_node')
        self.config_path = "/home/nvidia/ros_ws/src/f1-fifth/src/trajectory_csv"
        
        ##setup a timer callpacl to publish waypoints as markers
        self.timer = self.create_timer(0.1, self.publish_waypoints_as_markers)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoints_markers', 10)
        
        self.csv = "rotated_raceline_slalom_wide.csv"
        self.map_name = os.path.join(self.config_path, self.csv)
        self.waypoints = np.loadtxt(self.map_name, delimiter=';', skiprows=1) 

        x = self.waypoints[:, 1] #* 2.0
        y = self.waypoints[:, 2] #* 2.0
        v = np.ones_like(x) * 3.0
        self.track = Track.from_refline(x, y, v)
        
        self.initial_x = None
        self.initial_y = None
        

        WAYPOINTS_SUBSAMPLE_STEP = 10
        waypointx = self.track.raceline.xs
        waypointy = self.track.raceline.ys
        waypointyaw = self.track.raceline.yaws
        self.waypoints = np.column_stack((waypointx, waypointy, waypointyaw))
        self.waypoints = self.waypoints[::WAYPOINTS_SUBSAMPLE_STEP, :]

        self.marker_array = MarkerArray()
        
        ref = True
        for i, waypoint in enumerate(self.waypoints):
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
            self.marker_array.markers.append(marker)
        
        self.get_logger().info("Waypoint Node Initialized")

    def publish_waypoints_as_markers(self):
        """ Publish waypoints as visualization markers in RViz """
        self.marker_pub.publish(self.marker_array)

    def yaw_to_quaternion(self, yaw):
        """ Convert yaw angle to a quaternion (x, y, z, w) """
        return [0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)]
    
def main(args=None):
    rclpy.init(args=args)

    nmpc_node = WaypointPublisher()
    rclpy.spin(nmpc_node)

    # Clean up
    nmpc_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    