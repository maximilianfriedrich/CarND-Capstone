import cv2
import numpy as np
from styx_msgs.msg import TrafficLight

MAJOR_PIX_NUMBER = 50

class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        pass

    def classify(self, img, color_range_start, color_range_end):
        start = np.array(color_range_start, np.uint8)
        end = np.array(color_range_end, np.uint8)
        frame_threshed = cv2.inRange(img, start, end)
        return cv2.countNonZero(frame_threshed)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction

        img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        c = self.classify(img,
                          [0, 125, 125],
                          [15, 255, 255])
        if c > MAJOR_PIX_NUMBER:
            return TrafficLight.RED

        c = self.classify(img,
                          [28, 125, 125],
                          [47, 255, 255])
        if c > MAJOR_PIX_NUMBER:
            return TrafficLight.YELLOW

        c = self.classify(img,
                          [63, 125, 125],
                          [99, 255, 255])
        if c > MAJOR_PIX_NUMBER:
            return TrafficLight.GREEN
        return TrafficLight.UNKNOWN
