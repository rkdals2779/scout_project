#!/home/won/.pyenv/versions/3.7.8/envs/camera_ros/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose


class Scout_ctr:
    def __init__(self):
        self.dwa_motion_fin = 'off'
        self.xArm_catch = 'off'
        self.xArm_motion_fin = 'off'
        self.tf_motion_fin = 'off'
        self.ctr_mode = 'patrol'
        rospy.Subscriber('dwa_motion_fin', String, self.dwa_motion_fin_f)
        rospy.Subscriber('xArm_catch', String, self.xArm_catch_f)
        rospy.Subscriber('xArm_motion_fin', String, self.xArm_motion_fin_f)

        self.dwa_motion_start = rospy.Publisher('dwa_motion_start', String, queue_size=1)
        self.xArm_motion_start = rospy.Publisher('xArm_motion_start', String, queue_size=1)
        self.pub_to_dwa = String()
        self.pub_to_xArm = String()

    def p(self):




        self.dwa_motion_start.publish(self.pub_to_dwa)
        self.xArm_motion_start.publish(self.pub_to_xArm)

    def dwa_motion_fin_f(self, a):
        self.dwa_motion_fin = a.data

    def xArm_catch_f(self, a):
        self.xArm_catch = a.data

    def xArm_motion_fin_f(self, a):
        self.xArm_motion_fin = a.data


def main():
    rospy.init_node("Scout_control")
    control = Scout_ctr()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        control.p()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
