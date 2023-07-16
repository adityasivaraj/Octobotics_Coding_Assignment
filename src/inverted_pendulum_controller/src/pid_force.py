#!/usr/bin/env python3

import rospy
from inverted_pendulum_sim.msg import CurrentState
from inverted_pendulum_sim.msg import ControlForce
from inverted_pendulum_sim.srv import SetParams, SetParamsResponse, SetParamsRequest
from simple_pid import PID
from math import sin, cos, pi

current_state_sub = None

orientation_msg = pi - 0.3 #Default Orientation

def currentStateCallback(msg):
    global orientation_msg
    orientation_msg = msg.curr_theta

def publish_force():
    rospy.init_node('force_publisher', anonymous=True)
    global current_state_sub
    current_state_sub = rospy.Subscriber('/inverted_pendulum/current_state', CurrentState, currentStateCallback)
    force_pub = rospy.Publisher('/inverted_pendulum/control_force', ControlForce, queue_size=10)
    dt = 0.05
    rate = rospy.Rate(100)  #100 Hz

    max_force = 10 #Maximum force limit
    target_angle = pi

    PID_p = rospy.get_param('~PID_p', 1.0)
    PID_i = rospy.get_param('~PID_i', 0.1)
    PID_d = rospy.get_param('~PID_d', 0.01)

    pid_controller = PID(PID_p, PID_i, PID_d)
    pid_controller.sample_time = dt
    pid_controller.setpoint = 0.0

    t = 0.0  #Time variable
    while not rospy.is_shutdown():
        # force = amplitude * (amplitude - 0.5) * (1 - 2 * (abs(t * frequency - round(t * frequency))))
        # state = CurrentState()
        current_orientation = orientation_msg #state.curr_theta

        error = target_angle - current_orientation
        force = pid_controller(error)
        force = max(min(force, max_force), -max_force)

        force_msg = ControlForce()
        force_msg.force = force
        force_pub.publish(force_msg)

        t += dt  #Increment time variable
        rate.sleep()


if __name__ == '__main__':

    try:
        publish_force()
    except rospy.ROSInterruptException:
        pass