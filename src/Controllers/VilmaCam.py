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

# for now, we only import Image
from sensor_msgs.msg import Image

import numpy as np

from PIL import Image
import os
from datetime import datetime


class VilmaCamera():

    """
    Base Class to Vilma Stereo Camera'
    """

    def _debugCamera(self, imageVilma):
        if self.debugCam:
            cv2.imshow("Vilma cam", imageVilma)
            cv2.waitKey(3)

    # defines the region of interest
    def _roi(img, vertices):
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, vertices, 255)
        masked = cv2.bitwise_and(img, mask)

        return masked

    def _preProcess(self, image):

        # PRE-PROCESSING (maybe collect an ammount of image np array
        #  and then pre-process this array will be more performatic ??)

        # resize for help in pre/processing (e.g. Neural Nets)
        vilma_img = cv2.resize(image, (100, 100),
                               interpolation=cv2.INTER_LINEAR)

        # remove noise
        vilma_img = cv2.GaussianBlur(vilma_img, (5, 5), 0)

        # normalize the image
        cv2.normalize(vilma_img, vilma_img, 0, 255, cv2.NORM_MINMAX)

        # define the region of interest (the vertices depends directly to
        # what must be seen). This value must be updated accordingly with
        # the car which the camera is installed

        vertices = np.array([0, 0], [128, 0], [0, 128], [128, 128])

        vilma_img = self._roi(vilma_img, vertices)

        # TODO: Segmentation and Dimensionality reduction (Maybe ??)

        return vilma_img

    def _convertRostoCV(self, rosImage):

        try:
            vilma_img = self.bridge.imgmsg_to_cv2(rosImage, "bgr8")

            vilma_img = self._preProcess(vilma_img)

            # Convert to a np array and save it

            return vilma_img
        except CvBridgeError as e:
            print(e)

    def _convertImagetoArray(self, image):
        imageArray = np.asarray(image)
        self.imageList.append(imageArray)

    def _saveImage(self, saveAfter=2000):

        if len(self.imageList) >= saveAfter:
            dirName = './{}/'.format(self.dirName)

            # In future, change to a more performatic  save-image algorithm
            for i, image in enumerate(self.imageList):
                Image.fromarray(image).save(dirName /
                                            'camera_image_{}.jpeg'
                                            .format(datetime.now()))
            self.imageList.clear()

    def _cameraCallback(self, cameraData):

        # convert ROS data to image and pre-process
        imageVilma = self._convertRostoCV(cameraData)

        # convert to a np array and append to list
        self._convertImagetoArray(imageVilma)

        # save image in disk
        self._saveImage()

        # debuging camera image (if set, it will slow the performance)
        self._debugCamera(imageVilma)

    def __init__(self, node, debugCam=False):

        self.dirName = 'images'
        # Create target Directory if don't exist
        if not os.path.exists(dirName):
            os.mkdir(dirName)
            print("Directory ", dirName,  " Created ")
        else:
            print("Directory ", dirName,  " already exists")

        self.imageList = []
        self.bridge = CvBridge()
        self.debugCam = debugCam
        # this topic is created from pointgrey ros launch, don't update.
        self.leftCamTopic = "/camera/image_color"
        self.cameraNode = rospy.init_node(node)
        self.pointgrey_subscriber_left = rospy.Subscriber(
            self.leftCamTopic, Image, self._cameraCallback)

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
            rospy.loginfo("Shutting down")
        pass

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """

    vilmaCamera = VilmaCamera('vilma_stereo_cam', True)

    # run the node
    vilmaCamera.run()


if __name__ == '__main__':
    main()
