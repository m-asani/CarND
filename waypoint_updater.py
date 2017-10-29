#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from styx_msgs.msg import Lane, Waypoint
import time
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

LOOKAHEAD_WPS = 400 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
	self.waypoints = []
	self.first_execute = True
	self.pose = PoseStamped()
	self.velocity = ()
	self.lane = Lane()
	self.waypoint_offset = 0
	self.prev_waypoint_index = 0
	#print("resetting prev_fwd_waypoints")
	self.prev_fwd_waypoints = []
	self.current_waypoint = []
	self.temp_angle = 0
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.lane_cb)
	rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)

	
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
	

        # TODO: Add other member variables you need below

        rospy.spin()
    def velocity_cb(self, vel):
	self.velocity = vel
	current_time = rospy.Time(0)
	print(self.pose.pose.position.x, self.pose.pose.position.y,self.pose.pose.position.z,self.pose.pose.orientation.w, self.velocity.twist.linear.x,current_time  )

    def pose_cb(self, msg):
        # TODO: Implement
	#rospy.loginfo("enter pose_call back")
	self.pose = msg
	self.publish(msg)
        #pass

    

    def lane_cb(self, data):
        # TODO: Implement
	self.waypoints = data.waypoints
	#print("pose.x, pose.y,pose.z, pose.orientation, velocity.x, time ")
	#print("waypoints_position.x,waypoints_position.y,waypoints_orientation.w,waypoints_linear.x,waypoints_linear.y, waypoints_linear.z")
	#for i in range(0,len(self.waypoints)):
		#print(self.waypoints[i].pose.pose.position.x,self.waypoints[i].pose.pose.position.y,self.waypoints[i].pose.pose.orientation.w,self.waypoints[i].twist.twist.linear.x,self.waypoints[i].twist.twist.linear.y, self.waypoints[i].twist.twist.linear.z)
	

	self.lane = data
	
           

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    

    
    def find_fwd_waypoints(self, waypoints, pose, LOOKAHEAD_WPS):
         dist = 0
	 fwd_waypoints = []
	 #closest_waypoint_index = 0
	 closest_waypoint_index = self.find_closest_waypoints(waypoints, pose)
	 #print("closest waypoint index = ", closest_waypoint_index)
	 if (self.first_execute == True):
	 	self.waypoint_offset = closest_waypoint_index
	 index_end_val = closest_waypoint_index+ LOOKAHEAD_WPS
	 for i in range(closest_waypoint_index, index_end_val):
	     	fwd_waypoints.append(waypoints[i])
	 self.prev_waypoint_index = 0
	 self.prev_fwd_waypoints = fwd_waypoints
         return fwd_waypoints




    def find_closest_waypoints(self, waypoints, pose):
	waypoint_index = self.distance_pose(waypoints, pose )
	#print("length self.prev_fwd_waypoints in find_closest_waypoints = ",len(waypoints))
	return waypoint_index	


    def passed_waypoint(self, waypoints, pose):
	waypoint_index = self.distance_pose_new(waypoints, pose )
	#print("length self.prev_fwd_waypoints in find_closest_waypoints = ",len(waypoints))
	return waypoint_index	

   

    def publish(self, data):
        final_lane = Lane()
	final_lane.header.frame_id = self.lane.header.frame_id
	#final_lane.header.frame_id = '/world'
        final_lane.header.stamp = self.lane.header.stamp
	#final_lane.header.stamp = rospy.Time(0)
	waypoints = self.waypoints
	pose = self.pose
        #pose.pose = self.pose.pose
	if (self.first_execute == True):
        	final_lane.waypoints = self.find_fwd_waypoints( waypoints, pose, LOOKAHEAD_WPS)
		self.first_execute = False
		self.prev_fwd_waypoints = final_lane.waypoints
	if (self.first_execute == False):
		prev_waypoint_index = self.passed_waypoint( self.prev_fwd_waypoints, pose)
		#print((self.prev_fwd_waypoints[0].pose.pose.position.x),(self.prev_fwd_waypoints[prev_waypoint_index].pose.pose.position.x), self.pose.pose.position.x, self.pose.pose.position.y)
		if (self.prev_fwd_waypoints[0] != self.prev_fwd_waypoints[prev_waypoint_index]):#have we used up a waypoint			
			self.add_next_point()
        	final_lane.waypoints = self.prev_fwd_waypoints
		#print(self.prev_fwd_waypoints[0].pose.pose.position.x, self.prev_fwd_waypoints[0].pose.pose.position.y, self.pose.pose.position.x,self.pose.pose.position.y)
		
		#self.prev_fwd_waypoints = final_lane.waypoints
		self.current_waypoint = self.prev_fwd_waypoints[0]
	self.final_waypoints_pub.publish(final_lane)


    def add_next_point(self):
	self.prev_fwd_waypoints.append(self.waypoints[self.prev_waypoint_index+self.waypoint_offset +LOOKAHEAD_WPS+1])
	#print("waypoint_offset",self.waypoint_offset  )
	prev_fwd_waypoints = self.prev_fwd_waypoints[1::]
	
	self.prev_fwd_waypoints = prev_fwd_waypoints
	
	self.prev_waypoint_index = self.prev_waypoint_index + 1
	
	

	return self.prev_fwd_waypoints

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

    
    def distance_pose(self, waypoints, pose):
        dist_temp = 1000000
	k = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	dwx = lambda a,b : (a.position.x- b.position.x);
	dwy = lambda a,b : (a.position.y- b.position.y);
	 

        for i in range(0,len(waypoints)):
	    waypoint_posn_x = dwx(waypoints[i].pose.pose, pose.pose)
	    waypoint_posn_y = dwy(waypoints[i].pose.pose, pose.pose)
            waypoint_heading = math.atan2((waypoint_posn_y), (waypoint_posn_x))
            #waypoint_orientation = waypoints[i].pose.pose.orientation.w
            theta = pose.pose.orientation.w 
	    #theta = 0
	    angle = abs( waypoint_heading - theta)
	    #if (angle < self.temp_angle):
	    if (angle > 1.57):  
               dist = dl(pose.pose.position, waypoints[i].pose.pose.position)
	       if (dist < dist_temp):
	             dist_temp = dist
	             k = i
	    self.temp_angle =  angle       
        return k
	
    def distance_pose_new(self, waypoints, pose):
        dist_temp = 1000000
	k = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	dwx = lambda a,b : (a.position.x- b.position.x);
	dwy = lambda a,b : (a.position.y- b.position.y);
	dwz = lambda a,b : (a.position.z- b.position.z);
	 

        for i in range(0,len(waypoints)):
	    k=0
	    waypoint_heading = 0
	    dist = dl(pose.pose.position, waypoints[i].pose.pose.position)
	    if (dist < dist_temp):
	         dist_temp = dist
	         k = i
	dist = dl(pose.pose.position, waypoints[k].pose.pose.position)
	waypoint_posn_x = dwx(waypoints[k].pose.pose, pose.pose)
	waypoint_posn_y = dwy(waypoints[k].pose.pose, pose.pose)
	waypoint_posn_z = dwz(waypoints[k].pose.pose, pose.pose)
	#if (waypoint_posn_x == 0):
		#waypoint_posn_x = 0.00001
        waypoint_heading_yx = math.atan2((waypoint_posn_y), (waypoint_posn_x))
	waypoint_heading_xz = math.atan2((waypoint_posn_z), (waypoint_posn_x))
	#waypoint_heading_yz = math.atan2((waypoint_posn_y), (waypoint_posn_z))
	y_x_vector = math.sqrt(waypoint_posn_x*waypoint_posn_x + waypoint_posn_y*waypoint_posn_y)
	z_x_vector = math.sqrt(waypoint_posn_x*waypoint_posn_x + waypoint_posn_z*waypoint_posn_z)
	y_z_vector = math.sqrt(waypoint_posn_y*waypoint_posn_y + waypoint_posn_z*waypoint_posn_z)


           
	roll, pitch, yaw = self.get_rotation_pose ()
	
	corrected_yaw = abs( waypoint_heading_yx - yaw)
	corrected_pitch = abs( waypoint_heading_xz - pitch)
	#corrected_roll = abs( waypoint_heading_yz - roll)
	#corrected_yaw = abs( waypoint_heading_yx - yaw)
	#corrected_pitch = abs( waypoint_heading_xz - roll)
	#corrected_roll = abs( waypoint_heading_yz - pitch)
	
	new_heading = self.calculate_corrected_heading(y_x_vector,z_x_vector,corrected_yaw, corrected_pitch)
	    #if (angle < self.temp_angle):
	
	if ((corrected_yaw > math.pi/2)):
            k=k+1
	    #self.temp_angle =  angle       
        return k

    def get_rotation_pose (self):
	global roll, pitch, yaw
	orientation_q = self.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] 
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	return roll,pitch,yaw

    

	
    def calculate_corrected_heading(self,y_x_vector,z_x_vector,corrected_yaw, corrected_pitch):
	b = y_x_vector*math.cos(corrected_yaw)
	b2 = z_x_vector*math.cos(corrected_pitch)
	#b3 = y_z_vector*math.cos(corrected_roll)
	o = y_x_vector*math.sin(corrected_yaw)
	o2 = z_x_vector*math.sin(corrected_pitch)
	#o3 = y_z_vector*math.sin(corrected_roll)
	new_heading = math.atan2(o+o2,b+b2)
	return new_heading
 	     



if __name__ == '__main__':
    try:
	
        WaypointUpdater()
	
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
