#!/home/won/.pyenv/versions/3.7.8/envs/camera_ros/bin/python

import rospy
from std_msgs.msg import Float32, Bool, String
from geometry_msgs.msg import Pose

rospy.init_node('pub')

a = rospy.Publisher('dwa_motion_fin', String, queue_size=1)
b = String()
b.data = 'start'
while not rospy.is_shutdown():
    a.publish(b)