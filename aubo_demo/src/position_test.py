#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
import random

def gen_position():
    pub = rospy.Publisher('position', Float32)
    rospy.init_node('position_generator', log_level=rospy.INFO)
    rospy.loginfo("Generating position")

    while not rospy.is_shutdown():
        pub.publish(Float32(random.normalvariate(5,1)))
        rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        gen_position()
    except Exception, e:
        print "done"