#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32

import sys
import tf

from utilities.kdtree import kdtree
from utilities.hysteresis import hysteresis


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPTS = 200 # Number of waypoints we will publish. You can change this number
ACC_MIN = -0.8
DIST_MIN = 1


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        #rospy.logwarn("debugging: wapoint_updater.py - initialization - line42")
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.wpts_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.curr_vel_cb, queue_size=1)

        self.final_wpts_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.speed_limit = rospy.get_param('waypoint_loader/velocity', 40) / 3.6 # Convert kph to mps

        # TODO: Add other member variables you need below
        self.wpt_tree = None
        self.wpt_speeds = []
        self.tf_listener = tf.TransformListener()
        self.wpt_redlight = None
        self.next_wpt = -1
        self.speed_curr = None
        self.speed_target_traj = 0.0
        self.speed_hysteresis_traj = hysteresis(2.0, 2.1, 0.0)
        self.pose_curr = None    #Current pose
        self.wpts_base = None    #Base waypoints

        rospy.spin()


    def pose_cb(self, msg):
        # TODO: Implement
        self.pose_curr = msg
        #rospy.logwarn("waypoint_updater:pose_cb:self.pose_curr %s", self.pose_curr)

        if ((self.wpts_base == None) or (self.wpt_redlight == None) or (self.speed_curr == None)):
            #rospy.logwarn("debugging: wapoint_updater.py - pose_cb - line73")
            return # Do nothing unless all msgs received
        #rospy.logwarn("[debugging: wapoint_updater.py - pose_cb - line75]")
        # Find the nearest waypoint to the current position
        next_wpt = self.get_nearest_wpt(self.pose_curr.pose)

        # The next waypoint must be ahed of the current position
        transformed_wpt = self.trans_fromcar_tomap(self.wpts_base.waypoints[next_wpt].pose)

        if ((transformed_wpt != None) and (transformed_wpt.pose.position.x <= 0.0)):
            next_wpt += 1

        num_wpts = len(self.wpts_base.waypoints)
        #rospy.logwarn("waypoint_updater.py - pose_cb: num_wpts=%f", num_wpts)

        if next_wpt >= num_wpts:
            next_wpt -= num_wpts

        #rospy.logwarn("debugging: wapoint_updater.py - pose_cb - line88")
        self.calc_traj(next_wpt) # Calculate the trajectory
        next_wps = [None] * LOOKAHEAD_WPTS # Construct a set of following waypoints

        for _wp, wp in enumerate(range(next_wpt, next_wpt + LOOKAHEAD_WPTS)):
            wp_index = wp if (wp < num_wpts) else (wp - num_wpts)
            next_wps[_wp] = self.wpts_base.waypoints[wp_index]
            self.set_wpt_vel(next_wps, _wp, min(self.wpt_speeds[wp_index], self.get_traj_speed_at_wpt(_wp)))

        # Construct final_waypoints message
        lane = Lane()
        lane.waypoints = next_wps
        lane.header.frame_id = self.wpts_base.header.frame_id
        lane.header.stamp = rospy.Time(0)

        #rospy.loginfo("[debugging wapoint_updater.py - pose_cb - line105: final_waypoints published")
        self.final_wpts_pub.publish(lane)
        
        #self.wpts_base = msg
        #pass


    def wpts_cb(self, msg):
        # TODO: Implement
        if self.wpts_base != None:
            return

        self.wpts_base = msg

        for idx_wp in range(len(self.wpts_base.waypoints)):
            self.wpts_base.waypoints[idx_wp].pose.header.frame_id = self.wpts_base.header.frame_id
            self.wpt_speeds.append(self.speed_limit)

        self.wpt_tree = kdtree([(wpt.pose.pose.position.x, wpt.pose.pose.position.y) for wpt in self.wpts_base.waypoints], 2)
        #pass


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.wpt_redlight = msg.data
        #pass


    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass


    ### Added custom functions
    def curr_vel_cb(self, msg):# Get current velocity
        self.speed_curr = msg.twist.linear.x
    
    def get_nearest_wpt(self, pose):# Identify the nearest path waypoint to the position
        if self.wpt_tree == None:
            return None

        return self.wpt_tree.closest((pose.position.x, pose.position.y))[0]
    

    def trans_fromcar_tomap(self, pose_curr):# Transform the car position to the map coordinate
        try:
            self.tf_listener.waitForTransform("base_link", "world", rospy.Time(0), rospy.Duration(0.02))
            transformed_pose_curr = self.tf_listener.transformPose("base_link", pose_curr)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            try:
                self.tf_listener.waitForTransform("base_link", "world", rospy.Time(0), rospy.Duration(1.0))
                transformed_pose_curr = self.tf_listener.tranformPose("base_link", pose_curr)
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                transformed_pose_curr = None
                rospy.logwarn("Failed to transform pose")

        return transformed_pose_curr


    def calc_traj(self, next_wpt):# Calculate a trajectory
        speed_max = self.wpt_speeds[next_wpt]
        #rospy.logwarn("[wapoint_updater.py - calc_trajectory - line168] self.wpt_redlight = %f", self.wpt_redlight)

        if self.wpt_redlight > 0:
            #rospy.logwarn("wapoint_updater.py - calc_trajectory - line171] self.wpt_redlight")
            dist_stop = self.distance(self.wpts_base.waypoints, next_wpt, self.wpt_redlight)
            if dist_stop > DIST_MIN:
                dist_stop -= DIST_MIN
            speed_target = min(self.speed_curr, min(speed_max, math.sqrt(-2.0*ACC_MIN*dist_stop)))
        else:
            #rospy.logwarn("debugging: wapoint_updater.py - calc_trajectory - line177")
            speed_target = speed_max
        
        #rospy.logwarn("waypoint_updater.py - calc_trajectory: speed_target=%f", speed_target)

        self.speed_target_traj = self.speed_hysteresis_traj.output(speed_target)

    
    def get_traj_speed_at_wpt(self, wpt):# Get the expected speed at a waypoint
        return self.speed_target_traj


    def get_wpt_vel(self, wpt):
        return wpt.twist.twist.linear.x


    def set_wpt_vel(self, wpts, wpt, velocity):
        wpts[wpt].twist.twist.linear.x = velocity


    def distance(self, wpts, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(wpts[wp1].pose.pose.position, wpts[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')