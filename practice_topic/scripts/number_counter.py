#!/usr/bin/env python3
import rospy
import std_msgs.msg

counter = 0

def callback_number(msg):
    # subs
    global counter 
    counter = counter + msg.data
    print("result: ", counter)
    
    #pub
    pub.publish(counter)

if __name__ == "__main__":
    
    rospy.init_node("number_counter")
    
    sub = rospy.Subscriber("number", std_msgs.msg.Int64, callback_number)
    pub = rospy.Publisher("number_counter", std_msgs.msg.Int16, queue_size= 10)
    
    rate = rospy.Rate(10)
    
    rospy.spin()