#!/usr/bin/env python3

import rospy
import std_msgs.msg

if __name__ == "__main__":
    
    rospy.init_node("number_publisher")
    
    pub = rospy.Publisher('number', std_msgs.msg.Int64, queue_size= 10)
    
    rospy.set_param("/number_publish_frequency", 1)
    publish_frequency = rospy.get_param("/number_publish_frequency")
    rate = rospy.Rate(publish_frequency)
    
    rospy.set_param("/number_to_publish", 2)
    
    while not rospy.is_shutdown():
        msg = std_msgs.msg.Int64()
        number = rospy.get_param("/number_to_publish")
        msg.data = number
        pub.publish(msg)
        rate.sleep()
    
    rospy.loginfo("This node shut down ")