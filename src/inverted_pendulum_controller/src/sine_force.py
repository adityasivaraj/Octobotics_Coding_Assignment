#!/usr/bin/env python3

import rospy
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest

def publish_force():
    rospy.init_node('force_publisher', anonymous=True)
    force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
    rate = rospy.Rate(10)  #10 Hz

    amplitude = rospy.get_param('~amplitude', 1.0)  #Fetch amplitude from ROS parameter, default value is 1.0
    frequency = rospy.get_param('~frequency', 0.5)  #Fetch frequency from ROS parameter, default value is 0.5

    t = 0.0  #Time variable
    while not rospy.is_shutdown():
        force = amplitude * (amplitude - 0.5) * (1 - 2 * (abs(t * frequency - round(t * frequency))))

        force_msg = ControlForce()
        force_msg.force = force
        force_pub.publish(force_msg)

        t += 1.0 / 10.0  #Increment time variable
        rate.sleep()

if __name__ == '__main__':

    # rospy.init_node("SineForce", anonymous=True)
    # rospy.loginfo("[SineForce]: Node Started")
    # publish_force()

    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     rospy.loginfo("[StreamViewer]: Shutting down node")

    try:
        publish_force()
    except rospy.ROSInterruptException:
        pass