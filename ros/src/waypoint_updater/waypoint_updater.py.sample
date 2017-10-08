#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
from operator import itemgetter

import tf
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
ONEMPH = 0.44704 # in mps

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber("/current_velocity", TwistStamped, self.velocity_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Current pose of the vehicle updated at an unknown rate
        self.pose = None

        # Part of the complete waypoints retrived from /waypoints
        self.waypoints = None

        #Velocity
        self.current_velocity = None

        # Select the base waypoints ahead of the vehicle
        self.waypoints_ahead = []

        #Id of the first waypoint ahead
        self.next_waypoint_id = None

        # id of red light waypoint
        self.red_waypoint = -1

        # Cycle time for the waypoint updater
        self.dt = 0.1

        #Maximum allowed velocity
        self.max_vel = 10.0*ONEMPH

        # Minimum distance to the waypoint ahead
        self.min_distance_ahead = self.max_vel*self.dt

        #Distance to traffic light waypoint
        self.stop_distance_to_tl = 7.0

        #Distance from the stop point where the car starts to decelerate
        self.decelerating_distance = self.max_vel*10

        rospy.Timer(rospy.Duration(self.dt), self.process_final_waypoints)
        rospy.spin()

    def process_final_waypoints(self, event):

        if not self.pose:
            rospy.logwarn('no pose has been set')
            return None

        if not self.current_velocity:
            rospy.logwarn('no velocity have been set')
            return None

        if not self.waypoints:
            rospy.logwarn('no waypoints have been set')
            return None


        # get closest waypoint
        idx_begin = self.get_closest_waypoint_ahead()
        idx_end = idx_begin + LOOKAHEAD_WPS
        idx_end = min(idx_end, len(self.waypoints))
        rospy.logdebug("begin {}, end {}, len {}".format(idx_begin, idx_end, len(self.waypoints)))

        wps = []
        epsilon = 1.0
        dist_to_red = 0
        dist_to_stop = 0
        velocity = 0
        ptv = 0
        for i in range(idx_begin, idx_end):
            target_velocity = self.max_vel

            if self.red_waypoint > 0:
                dist_to_red = self.distance(i, self.red_waypoint)
                dist_to_stop = max(0, dist_to_red - self.stop_distance_to_tl)
                velocity = self.get_waypoint_velocity(i)

                # slow down getting closer to intersection
                if dist_to_stop > 0 and dist_to_stop < self.decelerating_distance:
                    target_velocity *= dist_to_red/self.decelerating_distance

                # push down the brakes a bit harder second half
                if dist_to_stop > 0 and dist_to_stop < self.decelerating_distance * .5:
                    target_velocity *= dist_to_stop/self.decelerating_distance

                # don't slow down so it doesn't make sense
                target_velocity = max(2., target_velocity)

                # perform the brake motion
                if dist_to_stop > 0.5 and dist_to_stop <= 2.:
                    target_velocity = 1.0
                if dist_to_stop > 0 and dist_to_stop <= 0.5:
                    target_velocity = 0.33
                elif dist_to_stop == 0:
                    target_velocity = 0.

            # accelerate smoothly
            swv = self.get_waypoint_velocity(idx_begin)
            ptv = swv if i == idx_begin else ptv
            cwv = self.get_waypoint_velocity(i)

            rospy.logdebug("{} = start v {}, current v {}, prev v {}, target v {}".format(i, swv, cwv, ptv, target_velocity))
            if swv == 0 and cwv == 0 and ptv == 0:
                target_velocity = (0.25 * target_velocity + 0.75 * ptv)
            elif ptv < target_velocity:
                target_velocity = (0.1 * target_velocity + 0.9 * ptv)

            # make higher velocities just be the highest
            if target_velocity > 4.0:
                target_velocity = self.max_vel

            # clip around the velocities always greater than min and lesser than max
            target_velocity = min(max(0, target_velocity), self.max_vel)

            # save previous waypoint target velocity
            ptv = target_velocity


            rospy.logdebug("waypoint {}, decelerating_dist {}, dist_to_red {}, "
                           "dist_to_stop {}, velocity {}, target_velocity {}".format(
                               i,
                               self.decelerating_distance,
                               dist_to_red,
                               dist_to_stop,
                               velocity,
                               target_velocity))

            self.set_waypoint_velocity(i, target_velocity)
            wps.append(self.waypoints[i])

        lane = Lane()
        lane.waypoints = wps
        self.final_waypoints_pub.publish(lane)

        dr = self.distance(idx_begin, self.red_waypoint)
        ds = max(0, dr - self.stop_distance_to_tl)
        vd = self.get_waypoint_velocity(idx_begin)
        rospy.logdebug("[idx %s ] tl %s ds %s dr %s vd %s ", idx_begin, self.red_waypoint, ds, dr, vd)

    def get_closest_waypoint_ahead(self):

        # get the pose of the vehicle
        cx = self.pose.position.x
        cy = self.pose.position.y
        co = self.pose.orientation
        cq = (co.x, co.y, co.z, co.w)
        _, _, ct = tf.transformations.euler_from_quaternion(cq)

        closest_wp = (float('inf'), -1)
        for wp_i in range(len(self.waypoints)):
            wx = self.waypoints[wp_i].pose.pose.position.x
            wy = self.waypoints[wp_i].pose.pose.position.y
            wa = ((wx - cx) * math.cos(ct) +
                  (wy - cy) * math.sin(ct)) > self.min_distance_ahead

            if not wa:
                continue

            dist = math.sqrt((cx - wx)**2 + (cy - wy)**2)
            if dist < closest_wp[0]:
                closest_wp = (dist, wp_i)

        return closest_wp[1]

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.red_waypoint = msg.data

    def obstacle_cb(self, msg):
        # TODO: callback for /obstacle_waypoint message.
        pass

    def velocity_cb(self, msg):
        self.current_velocity = msg.twist

    def get_waypoint_velocity(self, waypoint):
        return self.waypoints[waypoint].twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        self.waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(self.waypoints[wp1].pose.pose.position,
                       self.waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_from_waypoint(self, waypoint):
        """
            Compute the euclidian distance of the waipoint
            from the current pose of the vehicle
        """
        xw = waypoint.pose.pose.position.x
        yw = waypoint.pose.pose.position.y
        zw = waypoint.pose.pose.position.z

        xc = self.pose.position.x
        yc = self.pose.position.y
        zc = self.pose.position.z

        return math.sqrt((xw - xc)*(xw - xc) +
                         (yw - yc)*(yw - yc) +
                         (zw - zc)*(zw - zc))



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
