#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tf.transformations import euler_from_quaternion

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
ONEMPH = 0.44704

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
		#rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		#rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.pose = None
        self.max_vel = 25 * ONEMPH
        self.dt = 0.1
        self.min_dist_ahead = self.max_vel * self.dt * 2

        rospy.Timer(rospy.Duration(self.dt), self.loop)
        rospy.spin()

    def loop(self, event):
        if (self.pose is not None and self.waypoints is not None):
            car_x = self.pose.position.x
            car_y = self.pose.position.y
            car_z = self.pose.position.z
            car_o = self.pose.orientation
            rospy.loginfo("current pose (%s, %s)", car_x, car_y)

            car_q = (car_o.x, car_o.y, car_o.z, car_o.w)
            car_roll, car_pitch, car_yaw = euler_from_quaternion(car_q)

            closest_wp = (float('inf'), -1)
            for i in range(len(self.waypoints)):
                wp = self.waypoints[i]
                wp_x = wp.pose.pose.position.x
                wp_y = wp.pose.pose.position.y

                is_ahead = ((wp_x - car_x)*math.cos(car_yaw) + (wp_y - car_y)*math.sin(car_yaw)) > 0.0      
                if(not is_ahead):
                     continue
                
                dist = math.sqrt((car_x - wp_x)**2 + (car_y - wp_y)**2)
                if dist < closest_wp[0] and dist > self.min_dist_ahead:
                    closest_wp = (dist, i)
            
            idx_begin = closest_wp[1]
            idx_end = min(idx_begin + LOOKAHEAD_WPS, len(self.waypoints))
            wps = self.waypoints[idx_begin:idx_end]

            for i in range(len(wps)):
                target = self.max_vel
                start_wp_vel = wps[0].twist.twist.linear.x
                prev_wp_vel = start_wp_vel if i == 0 else prev_wp_vel
                curr_wp_vel = wps[i].twist.twist.linear.x
                
                if start_wp_vel == 0 and curr_wp_vel == 0 and prev_wp_vel ==0:
                    target = (0.25 * target + 0.75 * prev_wp_vel)
                elif prev_wp_vel < target:
                    target = (0.1 * target + 0.9 * prev_wp_vel)

                target = min(max(0, target), self.max_vel)
                prev_wp_vel = target
                wps[i].twist.twist.linear.x = target
            
            lane = Lane()
            lane.waypoints = wps

            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.waypoints = msg.waypoints

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
