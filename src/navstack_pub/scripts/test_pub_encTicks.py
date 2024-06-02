#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def publish_ticks():
    rospy.init_node('ticks_publisher', anonymous=True)
    pub_right_ticks = rospy.Publisher('right_ticks', Int16, queue_size=10)
    pub_left_ticks = rospy.Publisher('left_ticks', Int16, queue_size=10)
    rate = rospy.Rate(50)  # 50 Hz
    
    right_ticks = 0
    left_ticks = 0

    while not rospy.is_shutdown():
        right_ticks += 5
        left_ticks += 10
        # if right_ticks > 500: right_ticks = 0
        # if left_ticks > 500: left_ticks = 0

        # right_ticks = 100
        # left_ticks = 68

        pub_right_ticks.publish(right_ticks)
        pub_left_ticks.publish(left_ticks)

        rospy.loginfo("Published right ticks: %d, left ticks: %d", right_ticks, left_ticks)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_ticks()
    except rospy.ROSInterruptException:
        pass
