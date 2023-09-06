#!/usr/bin/env python3

import rospy
import std_msgs.msg

def callback_receive_radio_data(msg):
    rospy.loginfo("Message recevived: ")
    rospy.loginfo(msg)
    
if __name__ == "__main__":
    rospy.init_node('smartphone')
    
    sub = rospy.Subscriber("robot_news_radio", std_msgs.msg.String, callback_receive_radio_data)
    
    rospy.spin()
    
    