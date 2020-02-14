#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub=rospy.Publisher('direction', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        hello="Hello alfonso"
        rospy.loginfo(hello)
        pub.publish(hello)
        rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass