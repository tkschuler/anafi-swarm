#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import TwistStamped
from random import random
from random import seed

def talker():
    pub = rospy.Publisher("vicon/vicon/anafi_1", TwistStamped, queue_size=10)
    rospy.init_node('anafi_1', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        data = TwistStamped()
        data.twist.linear.x = random()
        data.twist.linear.y = random()
        data.twist.linear.z = random()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    seed(1)
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
