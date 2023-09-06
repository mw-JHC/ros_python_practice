#!/usr/bin/env python3

import rospy
import rospy_tutorials.srv

def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo("sum of " + str(req.a) + " + " + str(req.b) + " = " + str(result))
    return result
    

if __name__ == "__main__":
    rospy.init_node("add_two_ints_server")
    rospy.loginfo("add two ints server node created")
    
    service = rospy.Service("/add_two_ints", rospy_tutorials.srv.AddTwoInts, handle_add_two_ints)
    rospy.logdebug("Service server has been started.")
    
    rospy.spin()