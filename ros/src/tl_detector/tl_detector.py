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

    def get_closest_waypoint(self, light):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        distances = [self.get_distance(self.get_light_coord(light), 
                                       self.get_waypoint_coord(wp)) for wp in self.waypoints]
        return distances.index(min(distances))

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

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def get_euler(self, pose):
        o = pose.orientation
        q = (o.x, o.y, o.z, o.w)
        return tf.transformations.euler_from_quaternion(q)

    def get_distance(self, xy1, xy2):
        x1, y1 = xy1
        x2, y2 = xy2
        dx, dy = x1 - x2, y1 - y2
        return math.sqrt(dx*dx + dy*dy)

    def get_car_coord(self, car_pose):
        car_x = car_pose.position.x 
        car_y = car_pose.position.y
        return (car_x, car_y) 

    def get_light_coord(self, light):
        light_x = light.pose.pose.position.x 
        light_y = light.pose.pose.position.y
        return (light_x, light_y) 

    def get_waypoint_coord(self, waypoint):
        waypoint_x = waypoint.pose.pose.position.x 
        waypoint_y = waypoint.pose.pose.position.y
        return (waypoint_x, waypoint_y) 

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        #TODO find the closest visible traffic light (if one exists)        
        light_closest = (float('inf'), -1)
        if self.pose:
            for i in range(len(self.lights)):
                l = self.lights[i]
                l_x, l_y = self.get_light_coord(l)
                car_x, car_y = self.get_car_coord(self.pose)

                if self.get_distance((l_x, l_y),(car_x, car_y)) >= 90:
                    continue

                light_state = self.get_light_state(light)
                rospy.loginfo("TrafficLight: %s", light_state)
                if light_state != TrafficLight.RED:
                    continue

                light_index = self.get_closest_waypoint(light)
                light_wp = self.waypoints[light_index]
                
                _,_,car_yaw = get_euler(self.pose)
                light_is_ahead = False
                if ((l_x - car_x) * math.cos(car_yaw) + (l_y - car_y)*math.sin(car_yaw)) > 0:
                    light_is_ahead = True
                if light_is_ahead:
                    distance = self.get_distance((l_x, l_y),(car_x, car_y))
                    if distance < light_closest:
                        light_closest = (distance, i)
        
        if light_closest[1] is not None:
            return (light_closest[1], TrafficLight.RED)
        else:
            return (-1, TrafficLight.UNKNOWN)

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
