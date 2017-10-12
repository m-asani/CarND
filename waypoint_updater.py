#!/usr/bin/env python
## author Muhammad Asani
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
	self.pose = None
        self.lane = None
	self.waypoints = None
	self.header = None

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.lane_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

	
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
	self.pose = msg
        #pass

    def lane_cb(self, msg2):
        # TODO: Implement
	self.lane = msg2
	self.header = self.lane.header
	self.waypoints = self.lane.waypoints
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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

     def find_fwd_waypoints(self, waypoints, pose, LOOKAHEAD_WPS):
        dist = 0
	fwd_waypoints = []
	for i in range(0, len(waypoints)):
	
            dw = lambda a: math.sqrt((a.x)**2 + (a.y)**2  + (a.z)**2)
	    dp = lambda b: math.sqrt((b.x)**2 + (b.y)**2  + (b.z)**2)
        
            waypoint_posn = dw(waypoints[i].pose.pose.position)
	    pose_posn = dp(pose.pose.position)

            while ((waypoint_posn - pose_posn) > 0 and len(fwd_waypoints < LOOKAHEAD_WPS):
		fwd_waypoints.append(waypoints[i])
        return fwd_waypoints

     def publish(self, waypoints):
        lane = Lane()
        #lane.header.frame_id = '/world'
        #lane.header.stamp = rospy.Time(0)
	lane.header = self.header
        lane.waypoints = self.find_fwd_waypoints(self, waypoints, pose, LOOKAHEAD_WPS)
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
