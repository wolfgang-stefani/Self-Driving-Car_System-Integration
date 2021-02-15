#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial import KDTree # KDTree is a data structure that allows to look up the closest point in space really efficiently
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

Please note that the simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message.
'''

LOOKAHEAD_WPS = 200 # we will publish just a fixed number of waypoints currently ahead of the vehicle
MAX_DECEL = 0.5

class WaypointUpdater(object):
    
    # The purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles.
    
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # Member variables
        self.base_lane = None
        self.prev_state = self.now_state = -1
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb) # "listen" to current position of the vehicle
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb) # taking into account the basic trajectory. This list includes waypoints both before and after the vehicle.
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb) # taking into account the locations to stop for red traffic lights
        # rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb) # taking into account the locations of obstacles

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        self.loop() # this gives us control about the publishing frequency
    
    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
            
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])
        
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx
    
    def publish_waypoints(self):

        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)
        
    def generate_lane (self):
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        temp_base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = temp_base_waypoints

        else:
            lane.waypoints = self.decelerate_waypoints(temp_base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            stop_idx = max(self.stopline_wp_idx - closest_idx - 5 , 0)
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)

            if vel < 1.:
                vel = 0
                
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
            
        return temp
    
    def pose_cb(self, msg):
        # 'cb' for callback
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
