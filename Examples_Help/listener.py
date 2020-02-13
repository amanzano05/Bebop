#!/usr/bin/env python
import rospy
from std_msgs.msg import String

count=0
def callback(data):
    msg=data.data
    global count
    count=count+1
    print(count)
    print ("I heard "+ msg)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber("direction", String, callback)

        rate.sleep()
    


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    listener()