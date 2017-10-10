#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        x = pose.position.x
        y = pose.position.y

        closest_wp = (float('inf'), -1)
        for i in range(len(self.waypoints)):
            wp = self.waypoints[i]
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y
            dist = math.sqrt((x - wp_x)**2 + (y - wp_y)**2)
            if dist < closest_wp[0]:
                closest_wp = (dist, i)
        return closest_wp[1]

    def get_closest_stop_waypoint(self, position):
        x = position[0]
        y = position[1]

        closest_wp = (float('inf'), -1)
        for i in range(len(self.waypoints)):
            wp = self.waypoints[i]
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y
            dist = math.sqrt((x - wp_x)**2 + (y - wp_y)**2)
            if dist < closest_wp[0]:
                closest_wp = (dist, i)
        return closest_wp[1]

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        trans_mat = slf.listner.fromTranslationRotation(trans, rot)
        pt_world = np.array([[point_in_world.x],[point_in_world.y],[point_in_world.z],[1.0]])
        cam_vec = np.dot(trans_mat, pt_world)
        x = cam_vec[0][0]
        y = cam_vec[1][0]

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        #TODO find the closest visible traffic light (if one exists)
        if not self.pose or not self.waypoints or not self.lights or not self.light_classifier:
            return -1, TrafficLight.UNKNOWN

        car_x = self.pose.position.x
        car_y = self.pose.position.y
        car_o = self.pose.orientation
        car_q = (car_o.x, car_o.y, car_o.z, car_o.w)
        car_roll,car_pitch,car_yaw = tf.transformations.euler_from_quaternion(car_q)

        light_closest = (float('inf'),-1, None)
        for i in range(len(self.lights)):
            l = self.lights[i]
            l_x = l.pose.pose.position.x
            l_y = l.pose.pose.position.y
            l_o = l.pose.pose.orientation
            l_q = (l_o.x, l_o.y, l_o.z, l_o.w)
            l_roll, l_pitch, l_yaw = tf.transformations.euler_from_quaternion(l_q)

            light_ahead = ((l_x - car_x) * math.cos(car_yaw) + (l_y - car_y) * math.sin(car_yaw)) > 0.0
            if not light_ahead:
                continue

            light_distance = math.sqrt((car_x - l_x)**2 + (car_y - l_y)**2)
            if light_distance < light_closest[1]:
                light_closest = (light_distance, self.get_closest_waypoint(l.pose.pose), i)

            stop_line_positions = self.config['stop_line_positions']
            stop_closest = (float('inf'), -1)
            for sl in stop_line_positions:
                stop_x = sl[0]
                stop_y = sl[1]
                stop_d = math.sqrt((car_x - stop_x)**2 + (car_y - stop_y)**2)
                if stop_d < stop_closest[0]:
                    stop_closest = (stop_d,self.get_closest_stop_waypoint(sl))

        if light_closest[0] > 50:
            return -1,TrafficLight.UNKNOWN

        idx = stop_closest[1]
        tl_state = self.get_light_state(light_closest[2])

        rospy.loginfo("light state: %s", tl_state)
        return idx, tl_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
