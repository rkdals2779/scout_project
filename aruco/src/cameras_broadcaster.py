#!/home/min/.pyenv/versions/pyenv_py3810/bin/python3.8
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose


class camera_br:
    def __init__(self):
        self.L515_aruco01 = Pose()
        self.L515_detect = False
        self.D455_aruco01 = Pose()
        self.D455_detect = False
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('L515_aruco01_pose', Pose, self.L515_aruco01_pose)
        rospy.Subscriber('D455_aruco01_pose', Pose, self.D455_aruco01_pose)
        print("start")

    def brs(self):
        while not rospy.is_shutdown():
            self.br.sendTransform((0.1015, 0.0, 0.085),
                                  tf.transformations.quaternion_from_euler(-np.pi * 2 / 3, 0, -np.pi / 2),
                                  rospy.Time.now(),
                                  "L515",
                                  "world")
            if self.L515_detect:
                self.br.sendTransform((self.L515_aruco01.position.x, self.L515_aruco01.position.y,
                                       self.L515_aruco01.position.z),
                                      (tf.transformations.quaternion_from_euler(self.L515_aruco01.orientation.x,
                                                                                self.L515_aruco01.orientation.z,
                                                                                -self.L515_aruco01.orientation.y,
                                                                                'rxyz')),
                                      rospy.Time.now(),
                                      "L515_aruco01",
                                      "L515")

            self.br.sendTransform((0.35, 0.0, 0.15),
                                  tf.transformations.quaternion_from_euler(-np.pi / 2, 0, -np.pi / 2),
                                  rospy.Time.now(),
                                  "D455",
                                  "world")

            if self.D455_detect:
                self.br.sendTransform((self.D455_aruco01.position.x, self.D455_aruco01.position.y,
                                       self.D455_aruco01.position.z),
                                      (tf.transformations.quaternion_from_euler(self.D455_aruco01.orientation.x,
                                                                                -self.D455_aruco01.orientation.z,
                                                                                self.D455_aruco01.orientation.y,
                                                                                'rxyz')),
                                      rospy.Time.now(),
                                      "D455_aruco01",
                                      "D455")

    def L515_aruco01_pose(self, a):
        self.L515_aruco01 = a
        self.L515_detect = True

    def D455_aruco01_pose(self, a):
        self.D455_aruco01 = a
        self.D455_detect = True


def main():
    rospy.init_node("Cameras_broadcaster")
    camera = camera_br()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.brs()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
