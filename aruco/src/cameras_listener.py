#!/home/min/.pyenv/versions/pyenv_py3810/bin/python3.8
# -*- coding: utf-8 -*-

import rospy
import tf
import numpy as np
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose


class camera_ls:
    def __init__(self):
        self.aruco_Pose_to_xArm = rospy.Publisher('camera2_aruco_xyz', Pose, queue_size=1)
        self.aruco_Pose_to_DWA = rospy.Publisher('camera1_aruco_xyz', Pose, queue_size=1)
        self.L515_aruco01_detected = False
        self.L515_aruco01_detected_pre = False
        self.D455_aruco01_detected = False
        self.D455_aruco01_detected_pre = False
        print("start")
    def lss(self):
        listener = tf.TransformListener()
        pose_to_xArm = Pose()
        pose_to_DWA = Pose()
        while not rospy.is_shutdown():
            # L515
            try:
                (trans, rot) = listener.lookupTransform("world", "L515_aruco01", rospy.Time(0))
                self.L515_aruco01_detected = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.L515_aruco01_detected = False
                continue
            # publishing
            if self.L515_aruco01_detected:
                pose_to_xArm.position.x = trans[0]
                pose_to_xArm.position.y = trans[1]
                pose_to_xArm.position.z = trans[2]
                pose_to_xArm.orientation.x = rot[0]
                pose_to_xArm.orientation.y = rot[1]
                pose_to_xArm.orientation.z = rot[2]
                pose_to_xArm.orientation.w = rot[3]
                self.aruco_Pose_to_xArm.publish(pose_to_xArm)

            # D455
            try:
                (trans, rot) = listener.lookupTransform("world", "D455_aruco01", rospy.Time(0))
                self.D455_aruco01_detected = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.D455_aruco01_detected = False
            # publishing
            if self.D455_aruco01_detected:
                pose_to_DWA.position.x = trans[0]
                pose_to_DWA.position.y = trans[1]
                pose_to_DWA.position.z = trans[2]
                pose_to_DWA.orientation.x = rot[0]
                pose_to_DWA.orientation.y = rot[1]
                pose_to_DWA.orientation.z = rot[2]
                pose_to_DWA.orientation.w = rot[3]
                self.aruco_Pose_to_xArm.publish(pose_to_DWA)

            # 마커 감지 확인
            if self.L515_aruco01_detected is not self.L515_aruco01_detected_pre:
                print(f"L515: {self.L515_aruco01_detected}")
            if self.D455_aruco01_detected is not self.D455_aruco01_detected_pre:
                print(f"D455: {self.D455_aruco01_detected}")
            self.L515_aruco01_detected_pre = self.L515_aruco01_detected
            self.D455_aruco01_detected_pre = self.D455_aruco01_detected




def main():
    rospy.init_node("Cameras_listener")
    camera = camera_ls()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera.lss()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
