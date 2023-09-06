#!/usr/bin/env python3

import rospy
import rospy_tutorials.srv

if __name__ == "__main__":
    rospy.init_node("add_two_ints_client")
    
    rospy.wait_for_service("/add_two_ints")
    
    try:
        add_two_ints = rospy.ServiceProxy("/add_two_ints", rospy_tutorials.srv.AddTwoInts)
        response = add_two_ints(8,6)
        rospy.loginfo("Sum is " + str(response.sum))
    except rospy.ServiceException as e:
        rospy.logwarn("Service Failed: " + str(e))
    