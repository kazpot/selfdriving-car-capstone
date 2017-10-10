from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        averages = image.mean(0).mean(0)
        blue, green, red = averages
        
        prediction = TrafficLight.UNKNOWN

        if red > blue and red > green:
            prediction = TrafficLight.RED
        elif green > red and green > blue:
            prediction = TrafficLight.GREEN
        else:
            prediction = TrafficLight.UNKNOWN

        return prediction
