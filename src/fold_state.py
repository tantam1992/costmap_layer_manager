#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publish_state():
    rospy.init_node('test_state_publisher', anonymous=True)
    pub = rospy.Publisher('/fold_state', String, queue_size=10)

    rate = rospy.Rate(10)  # Publish at 10Hz

    while not rospy.is_shutdown():
        # Change the state as needed
        state = "OPERATIONAL/READY"  # Change this to "OPERATIONAL/LOADED" or other states to test
        # state = "OPERATIONAL/LOADED"  # Change this to "OPERATIONAL/LOADED" or other states to test
        pub.publish(state)
        rospy.loginfo(f"Published state: {state}")
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_state()
    except rospy.ROSInterruptException:
        pass
