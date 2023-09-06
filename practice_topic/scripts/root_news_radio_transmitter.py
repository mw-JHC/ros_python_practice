#!/usr/bin/env python3

import rospy
import std_msgs.msg


if __name__ == "__main__":
    rospy.init_node("robot_news_radio_transmitter")
    
    pub = rospy.Publisher("/robot_news_radio", std_msgs.msg.String, queue_size= 10)

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = std_msgs.msg.String()
        msg.data = "i am ROS Expert"
        pub.publish(msg)
        rate.sleep()
        
    rospy.loginfo("This node shut down ")
    