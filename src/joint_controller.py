#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import robotlimits

axis_state = []
joint_names = []

limits = robotlimits.get_joint_limits()


def state_callback(joint_msg: JointTrajectoryControllerState):
    global axis_state
    global joint_names
    axis_state = joint_msg.actual.positions
    joint_names = joint_msg.joint_names


def callback(joy_msg: Joy):
    # Read Inputs
    ## Joystick

    # Calculate new Joint Positions
    # Map RT and LT as one axis
    joy_j1_up = abs(joy_msg.axes[2] - 1)/2 # 0 -> 1
    joy_j1_down = -abs(joy_msg.axes[5] - 1)/2  # 0 -> -1
    joy_j1 = axis_state[0] + 5 * ( joy_j1_up +  joy_j1_down)
    

   
    joy_j2 = axis_state[1]
    if len(axis_state) >= 1:
        joy_j2 = joy_msg.axes[0] * 5 + axis_state[1]
    else:
        rospy.logwarn("axis has not length 2")
    joy_j3 = joy_msg.axes[1] * 5 + axis_state[2]

    # Joy 4 as set to RB and LB
    if joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 0:
        joy_j4 = axis_state[3]
    elif joy_msg.buttons[4] == 1:
        joy_j4 = axis_state[3] - 5 / 2
    else:
        joy_j4 = axis_state[3] + 5 / 2

    # Tool
    joy_j5 = joy_msg.axes[4] * 5 / 2 + axis_state[4]
    joy_j6 = joy_msg.axes[3] * 5 / 2 + axis_state[5]

    joints = [joy_j1, joy_j2, joy_j3, joy_j4, joy_j5, joy_j6]

    #check for limit

    # Publish Data
    dur = []
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = joint_names
    point = JointTrajectoryPoint()
    for i in range(6):
        cmd = joints[i]
        point.positions.append(cmd)
    point.time_from_start = rospy.Duration(2)
    traj.points.append(point)

    pubJoint.publish(traj)

if __name__ == '__main__':
    rospy.init_node('joint_controller_node')
    sub = rospy.Subscriber('/joy', Joy, callback=callback)
    subJoint = rospy.Subscriber('/joint_trajectory_controller/state', JointTrajectoryControllerState,
                                callback=state_callback, queue_size=1)
    pubJoint = rospy.Publisher('/joint_trajectory_controller/command', JointTrajectory, queue_size=1)
    rospy.spin()
