#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose


class Scout_ctr:
    def __init__(self):
        self.dwa_motion_fin = 'off'
        self.xArm_motion_fin = 'off'
        self.mode = String()
        # self.mode.data = 'xArm_move'
        self.mode.data = 'go_to_aruco'
        self.mode_pre = 'previous'
        self.mode_num = 0
        rospy.Subscriber('dwa_motion_fin', String, self.dwa_motion_fin_f)
        rospy.Subscriber('xArm_motion_fin', String, self.xArm_motion_fin_f)

        self.dwa_motion_start = rospy.Publisher('dwa_motion_start', String, queue_size=1)
        self.xArm_motion_start = rospy.Publisher('xArm_motion_start', String, queue_size=1)
        #self.pub_to_dwa = String()
        #self.pub_to_xArm = String()

    def p(self):
        # if self.mode.data == 'xArm_move':  # Just xArm
        #     if self.xArm_motion_fin == 'xArm_move_fin':
        #         self.mode.data = 'go_to_aruco'
        #
        # if self.mode.data == 'go_to_aruco':    # Just DWA
        #     if self.dwa_motion_fin == 'dwa_fin':
        #         self.mode.data = 'go_to_home'
        #         # self.mode.data = 'xArm_move_home'
        #
        #
        # elif self.mode.data == 'go_to_home':   # Just DWA
        #     if self.dwa_motion_fin == 'dwa_fin':
        #         self.mode.data = 'xArm_move_home'
        #
        # if self.mode.data == 'xArm_move_home':   # Just xArm
        #     if self.xArm_motion_fin == 'xArm_move_home_fin':
        #         self.mode.data = 'go_to_aruco'



        
        # if self.mode.data == 'xArm_move':  # Just xArm
        #     if self.xArm_motion_fin == 'xArm_move_fin':
        #         self.mode.data = 'xArm_move_home'

        # if self.mode.data == 'xArm_move_home':   # Just xArm
        #     if self.xArm_motion_fin == 'xArm_move_home_fin':
        #         self.mode.data = 'go_to_aruco'

        if self.mode.data == 'go_to_aruco':    # Just DWA
            if self.dwa_motion_fin == 'dwa_fin':
                self.mode.data = 'none'
                # self.mode.data = 'xArm_move_home'
        
        elif self.mode.data == 'none':
            r = rospy.Rate(1)
            r.sleep()
            self.mode.data = 'go_to_home'


        elif self.mode.data == 'go_to_home':   # Just DWA
            if self.dwa_motion_fin == 'dwa_fin':
                self.mode.data = 'xArm_move_home'

        



        if self.mode.data != self.mode_pre:
            self.mode_pre = self.mode.data
            self.mode_num += 1
            print('[{}]mode: {}'.format(self.mode_num, self.mode.data))

        self.dwa_motion_start.publish(self.mode.data)
        self.xArm_motion_start.publish(self.mode.data)

    # Subscribe 받을 시 self 변수에 대입을 위한 함수
    def dwa_motion_fin_f(self, a):
        self.dwa_motion_fin = a.data

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
