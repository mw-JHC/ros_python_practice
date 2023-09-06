#!/usr/bin/env python3

import rospy
import std_msgs.msg

if __name__ == "__main__":
    
    rospy.init_node("number_publisher")
    
    pub = rospy.Publisher('number', std_msgs.msg.Int64, queue_size= 10)
    
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        msg = std_msgs.msg.Int64()
        msg.data = 2
        pub.publish(msg)
        rate.sleep()
    
    rospy.loginfo("This node shut down ")