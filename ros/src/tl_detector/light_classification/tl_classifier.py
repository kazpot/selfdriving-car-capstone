from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        self.GREEN_CHANNEL = 1
        self.RED_CHANNEL = 2
        self.threshold = 80

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        red_img = image[:,:,self.RED_CHANNEL]
        green_img = image[:,:,self.GREEN_CHANNEL]
        red_area = np.sum(red_img == red_img.max())
        green_area = np.sum(green_img == green_img.max())
        
        prediction = TrafficLight.UNKNOWN
        
        if red_area > self.threshold:
            prediction = TrafficLight.RED
        elif green_area > self.threshold:
            prediction = TrafficLight.GREEN
        else:
            prediction = TrafficLight.UNKNOWN
        
        return prediction
