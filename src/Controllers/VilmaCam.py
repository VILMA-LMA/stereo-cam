#!/usr/bin/env python

#
# Author: Muhamed Avila (femuhamed@hotmail.com)
#
# This work is licensed under the terms of the GPL v3.0 license.
# For a copy, see <https://opensource.org/licenses/GPL-3.0>.
#

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# ROS import
import rospy

from vilma_stereo_cam.msg import VilmaCam


class VilmaCamera():

    """
    Base Class to Vilma Stereo Camera'
    """

    def _convertRostoCV(self, rosImage):

        try:
            vilma_img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")

            # resize for help in pre/processing (e.g. Neural Nets)
            vilma_img = cv2.resize(vilma_img, (128, 128))
            return vilma_img
        except CvBridgeError as e:
            print(e)

    def _convertImagetoMatrix(self, image):

        # TODO: convert the obtained image to a reshaped/normalized matrix
        pass

    def _saveImage(self, image):

        # TODO: save the image(to a database??)
        pass

    def _cameraCallback(self, cameraData):

        # for now we just only convert ros data to image
        imageVilma = self._convertRostoCV(cameraData.image)

    def __init__(self, node):

        self.bridge = CvBridge()

        # this topic is created from pointgrey ros launch, don't update.
        self.leftCamTopic = "topic"
        self.cameraNode = rospy.init_node(node)
        self.pointgrey_subscriber_left = rospy.Subscriber(
            self.leftCamTopic, VilmaCam, self._cameraCallback)

        rospy.loginfo(
            'listening to pointgrey left camera with topic {}'
            .format(self.leftCamTopic))

    def run(self):
        """
        main loop
        """
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """

    vilmaCamera = VilmaCamera()

    # run the node
    vilmaCamera.run()

if __name__ == '__main__':
    main()
