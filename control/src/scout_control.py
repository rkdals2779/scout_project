#!/home/won/.pyenv/versions/3.7.8/envs/camera_ros/bin/python
# Lenovo 환경 셔뱅
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose


class Scout_ctr:
    def __init__(self):
        self.dwa_motion_fin = 'off'
        self.xArm_motion_fin = 'off'
        self.mode = 'go_to_aruco'
        self.mode_pre = 'previous'
        self.mode_num = 0
        rospy.Subscriber('dwa_motion_fin', String, self.dwa_motion_fin_f)
        rospy.Subscriber('xArm_motion_fin', String, self.xArm_motion_fin_f)

        self.dwa_motion_start = rospy.Publisher('dwa_motion_start', String, queue_size=1)
        self.xArm_motion_start = rospy.Publisher('xArm_motion_start', String, queue_size=1)
        self.pub_to_dwa = String()
        self.pub_to_xArm = String()

    def p(self):
        if self.mode == 'go_to_aruco':  # Just DWA
            if self.dwa_motion_fin == 'go_to_aruco_fin':
                self.mode = 'xArm_move'

        if self.mode == 'xArm_move':    # Just xArm
            if self.xArm_motion_fin == 'xArm_move_fin':
                self.mode = 'go_to_home'

        if self.mode == 'go_to_home':   # Just DWA
            if self.dwa_motion_fin == 'go_to_home_fin':
                self.mode = 'xArm_move_home'

        if self.mode == 'xArm_move_home':   # Just xArm
            if self.xArm_motion_fin == 'xArm_move_home_fin':
                self.mode = 'go_to_aruco'   # 처음 부터 다시

        if self.mode != self.mode_pre:
            self.mode_pre = self.mode
            self.mode_num += 1
            print(f'[{self.mode_num}]mode: {self.mode}')

        self.dwa_motion_start.publish(self.pub_to_dwa)
        self.xArm_motion_start.publish(self.pub_to_xArm)

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
