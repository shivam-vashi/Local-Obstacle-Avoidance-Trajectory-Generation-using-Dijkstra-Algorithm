#!/usr/bin/env python2
"""
The main executable for the planner module. This is the file that should be run when
starting the planner module.
"""
from csv_planner import CSVPlanner
from csv_planner import quat_from_yaw

import argparse
import numpy as np
import os
import rospy

from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class ROSPlanner:
    """Class implementation of the planner module that publishes control references.

    This planner contains two main objects: the global planner and the local planner.
    The global planner creates a static trajectory that will not change. The local
    planner generates dynamic trajectories that are collision-free, dynamically
    feasible (according to some model of the vehicle), and converges toward the global
    planner.
    """
    
    def __init__(self, global_traj_file, waypoint_spacing, max_accel, 
                 max_decel, max_vel=None, sim=False):
        # Initialize node parameters.
        self.node_name = "planner"
        rospy.init_node(self.node_name)
        rospy.loginfo("Initialized node.")
        self.loop_rate = 50. # [Hz]
        self.rate = rospy.Rate(self.loop_rate)

        # This needs to be adjusted according to which controller is being used. Some
        # control configurations will use a feedforward component based on curvature and
        # they may require the reference curvature to be extracted at a different point
        # than the reference pose and velocity.
        self.lookahead_distance = 2.6
        self.feedforward_lookahead_d = 2.6
        use_Dijkstra_planner = True

        if use_Dijkstra_planner:
            import pandas as pd
            import ast
            # Read the CSV file using pandas
            df = pd.read_csv('local_trajectory_output_Dijkstra.csv')

            # Convert string representations of lists to actual lists
            x_values_str = df['x'].tolist()
            x_values_list = [ast.literal_eval(x) for x in x_values_str]

            y_values_str = df['y'].tolist()
            y_values_list = [ast.literal_eval(y) for y in y_values_str]

            yaw_values_str = df['yaw'].tolist()
            yaw_values_list = [ast.literal_eval(yaw) for yaw in yaw_values_str]

            velocity_values_str = df['velocity'].tolist()
            velocity_values_list = [ast.literal_eval(velocity) for velocity in velocity_values_str]

            s_values_str = df['s'].tolist()
            s_values_list = [ast.literal_eval(s) for s in s_values_str]

            c_values_str = df['c'].tolist()
            c_values_list = [ast.literal_eval(c) for c in c_values_str]
            min_length = min(np.shape(x_values_list)[1], np.shape(y_values_list)[1], np.shape(yaw_values_list)[1], np.shape(velocity_values_list)[1], np.shape(s_values_list)[1], np.shape(c_values_list)[1])
            # print(np.shape(x_values_list[0][0:min_length]))
            # Convert lists to numpy arrays
            x_values = np.array(x_values_list[0][0:min_length])
            y_values = np.array(y_values_list[0][0:min_length])
            yaw_values = np.array(yaw_values_list[0][0:min_length])
            velocity_values = np.array(velocity_values_list[0][0:min_length])
            s_values = np.array(s_values_list[0][0:min_length])
            c_values = np.array(c_values_list[0][0:min_length])
            
            # print((np.shape(x_values), len(y_values), len(yaw_values), len(velocity_values), len(s_values), len(c_values)))
            # Create 2D numpy array
            self.g_traj = np.array(([x_values, y_values, yaw_values, velocity_values, s_values, c_values])).T
            # print(self.g_traj[:, 0:2])
            print(np.shape(self.g_traj))
            
        else:
            # Create global planner.
            self.global_planner = CSVPlanner(global_traj_file, 
                                            waypoint_spacing, 
                                            max_accel,
                                            max_decel,
                                            max_vel,
                                            sim)
            
            # Get global trajectory.
            self.g_traj = self.global_planner.get_trajectory()
            rospy.loginfo(
                "[" + self.node_name + "]: Global trajectory starts at [" 
                + str(self.g_traj[0,0]) + "," + str(self.g_traj[0,1]) + "]"
            )
        
        # Select local planner function.
        self.local_planner = lambda global_trajectory: global_trajectory

        # Generate a local trajectory.
        self.l_traj = self.local_planner(self.g_traj)

        # Initialize attributes to be populated by subscribers.
        self.current_position = (0.0, 0.0)

        # Create ROS publishers.
        self.pub_ctrl_ref_pose = rospy.Publisher(
            "/ctrl_ref_pose",
            PoseStamped, 
            queue_size=1, 
            tcp_nodelay=True
        )
        self.pub_ctrl_ref_twist = rospy.Publisher(
            "/ctrl_ref_twist",
            TwistStamped,
            queue_size=1, 
            tcp_nodelay=True
        )
        self.pub_ctrl_ref_curv = rospy.Publisher(
            "/ctrl_ref_curv",
            PointStamped,
            queue_size=1,
            tcp_nodelay=True
        )

        # Create subscribers.
        self.current_position = None
        rospy.Subscriber(
            '/novatel/oem7/odom', 
            Odometry, 
            self.position_callback, 
            tcp_nodelay=True
        )

        while not rospy.is_shutdown():
            self.publisher()
            self.rate.sleep()
    
    def position_callback(self, msg):
        """Get new position and publish the new plan segment."""
        odom_age = rospy.get_time() - msg.header.stamp.to_sec()
        if abs(odom_age) > 1. / self.loop_rate:
            rospy.logerr(self.node_name + ": Outdated odom message= " + str(odom_age) + " sec")
            # TODO: Determine appropriate fault response for this.
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        self.current_position = (current_x, current_y)

    def publisher(self):
        # Do not publish a reference if we have not yet received our pose.
        if not self.current_position:
            rospy.loginfo_throttle(1, "No current position.")
            return
        
        # Determine if the local planner should generate a new plan.
        # Here is where we can replan at a different frequency than what is
        # needed to generate control references.
        if False:
            self.l_traj = self.local_planner()

        # Get control references from local trajectory.
        pose, twist, curv = self.get_control_reference()

        # Publish pose and twist.
        self.pub_ctrl_ref_pose.publish(pose)
        self.pub_ctrl_ref_twist.publish(twist)
        self.pub_ctrl_ref_curv.publish(curv)

    def traverse_local_path(self, distance, i_start):
        """Traverse path for some distance to get index at that distance."""
        i_min = i_start
        dist_along_path = self.l_traj[i_min, 4]
        while distance > 0.0:
            i_min += 1
            try:
                distance -= self.l_traj[i_min, 4] - dist_along_path
                dist_along_path = self.l_traj[i_min, 4]
            except IndexError:
                break
        
        # We want the point just before the distance is traversed not after.
        i_min -= 1
        return i_min


    def get_control_reference(self):
        """Gets the reference pose, velocity, and curvature for the controller."""
        # Store variables so they are not overwritten by callbacks.
        current_position = self.current_position

        # Find closest position of local trajectory to vehicle.
        ego_x = np.ones((self.l_traj.shape[0], 1)) * current_position[0]
        ego_y = np.ones((self.l_traj.shape[0], 1)) * current_position[1]
        ego_position = np.hstack([ego_x, ego_y])

        distances = np.linalg.norm(self.l_traj[:, 0:2] - ego_position, axis=1)
        i_min = np.argmin(distances)

        # Traverse local trajectory until reached lookahead distance.
        i_min = self.traverse_local_path(self.lookahead_distance,i_min)

        # Create Pose stamped message.
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "odom"
        pose.pose.position.x = self.l_traj[i_min, 0]
        pose.pose.position.y = self.l_traj[i_min, 1]
        x, y, z, w = quat_from_yaw(self.l_traj[i_min, 2])
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        # Create Twist Stamped message.
        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = "odom"
        twist.twist.linear.x = self.l_traj[i_min, 3]

        # Traverse local trajectory again to find feedforward distance.
        i_min = np.argmin(distances)
        i_min = self.traverse_local_path(self.feedforward_lookahead_d, i_min)

        # Create curvature stamped message.
        curv = PointStamped()
        curv.header.stamp = rospy.Time.now()
        curv.header.frame_id = "odom"
        curv.point.x = self.l_traj[i_min, 5]

        return pose, twist, curv



# Initialize parser
parser = argparse.ArgumentParser(description="Local planner using csv data as waypoints.")
parser.add_argument("-s", "--sim", action="store_true", default=False,
                    help="Flag adjusts the start path to (0,0) for simulation.")
parser.add_argument("-p", "--path", required=True, help="Path to csv file to use.")
parser.add_argument("--waypoint_spacing", type=float, default=0.1,
                    help="The distance [m] to sample the waypoints from the csv at.")
parser.add_argument("--max_vel", type=float, default=None,
                    help="Maximum velocity (m/s) to navigate the maneuver.")
parser.add_argument("--max_accel", type=float, default=1.0,
                    help="Maximum acceleration (m/s^2) to reach constant velocity.")
parser.add_argument("--max_decel", type=float, default=8.0,
                    help="Maximum deceleration (m/s^2) to come to a stop.")

if __name__ == "__main__": 
    args = parser.parse_args()
    filename = args.path
    if not os.path.isfile(filename):
        raise AssertionError("%s does not exist" % filename)
    try:
        planner = ROSPlanner(filename, args.waypoint_spacing, args.max_accel, 
                             args.max_decel, args.max_vel, args.sim)
    except rospy.ROSInterruptException:
        pass
