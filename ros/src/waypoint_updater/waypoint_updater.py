#!/usr/bin/env python

import tf
import rospy
import math
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		#rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.red_idx = None
        self.pose = None
        self.max_vel = 25 * ONEMPH
        #self.dt = 0.1
        #self.min_dist_ahead = self.max_vel * self.dt * 2

        #rospy.Timer(rospy.Duration(self.dt), self.loop)
        self.loop()
        rospy.spin()

    def loop(self, event):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if (self.pose is not None and self.waypoints is not None):
                car_x, car_y = self.get_car_coord(self.pose)
                _,_,car_yaw = self.get_euler(self.pose)
                rospy.loginfo("current pose (%s, %s)", car_x, car_y)
                
                closest_wp = (float('inf'), -1)
                for i in range(len(self.waypoints)):
                    wp = self.waypoints[i]
                    wp_x,wp_y = self.get_waypoint_coord(wp)
                    
                    wp_is_ahead = False
                    if ((wp_x - car_x) * math.cos(car_yaw) + (wp_y - car_y) * math.sin(car_yaw)) > 0:
                         wp_is_ahead = True
                    if wp_is_ahead:
                        dist = self.get_distance((car_x,car_y), (wp_x, wp_y))
                        #if dist < closest_wp[0] and dist > self.min_dist_ahead:
                        if dist < closest_wp[0]:
                            closest_wp = (dist, i)
                
                idx_begin = closest_wp[1]
                idx_end = min(idx_begin + LOOKAHEAD_WPS, len(self.waypoints))
                wps = self.waypoints[idx_begin:idx_end]
                
                initial_wp_velocity = wps[0].twist.twist.linear.x
                target_wp_velocity = self.max_vel
                for i in range(len(wps)):
                    prev_wp_vel = initial_wp_velocity if i == 0 else prev_wp_vel
                    curr_wp_vel = wps[i].twist.twist.linear.x
                    
                    if initial_wp_velocity == 0 and prev_wp_vel == 0 and curr_wp_vel == 0:
                        target_wp_velocity = 0.25 * target_wp_velocity
                    else:
                        target_wp_velocity = (0.1 * target_wp_velocity + 0.9 * prev_wp_vel)
                    
                    if self.red_light_ahead():
                        target_wp_velocity = 0
                    
                    prev_wp_vel = target_wp_velocity
                    wps[i].twist.twist.linear.x = target_wp_velocity
                
                lane = Lane()
                lane.waypoints = wps
                
                self.final_waypoints_pub.publish(lane)
                rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, msg):
        # TODO: Implement
        self.waypoints = msg.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity
    
    def get_distance(self, xy1, xy2):
        x1, y1 = xy1
        x2, y2 = xy2
        dx, dy = x1 - x2, y1 - y2
        return math.sqrt(dx*dx + dy*dy)

    def get_distance_waypoint(self, waypoint, pose):
        wp_x, wp_y = self.get_waypoint_coord(waypoint)
        car_x, car_y = self.get_car_coord(pose)
        dx = wp_x - car_x 
        dy = wp_y - car_y
        return math.sqrt(dx*dx + dy*dy)

    def get_car_coord(self, car_pose):
        car_x = car_pose.position.x 
        car_y = car_pose.position.y
        return (car_x, car_y) 

    def get_waypoint_coord(self, waypoint):
        waypoint_x = waypoint.pose.pose.position.x 
        waypoint_y = waypoint.pose.pose.position.y
        return (waypoint_x, waypoint_y) 

    def get_euler(self, pose):
        o = pose.orientation
        q = (o.x, o.y, o.z, o.w)
        return tf.transformations.euler_from_quaternion(q)

    def red_light_ahead(self):
        if self.red_idx is None or self.waypoints is None or self.pose is None:
            return False
        else:
            wps = self.waypoints
            red_wp = wps[self.red_idx]
            d = self.get_distance_waypoint(red_wp, self.pose)
            red_x, red_y = self.get_waypoint_coord(red_wp)
            car_x, car_y = self.get_car_coord(self.pose)
            _,_,car_yaw = self.get_euler(self.pose)

            if ((red_x - car_x) * math.cos(car_yaw) + (red_y - car_y) * math.sin(car_yaw)) > 0 and d <= 50:
                return True
            else:
                return False


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
