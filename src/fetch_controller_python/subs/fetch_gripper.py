#!/usr/bin/env python3
import sys, os
sys.path.insert(1, os.path.abspath("./.."))
import actionlib
import control_msgs.msg
import rospy
from lib.params import CLOSED_POS, OPENED_POS, MIN_EFFORT, MAX_EFFORT, GRIPPER_CONTROL_GROUP

class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    def __init__(self):
        self._client = actionlib.SimpleActionClient(GRIPPER_CONTROL_GROUP, control_msgs.msg.GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(10))

    def open(self):
        """Opens the gripper.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        self._client.send_goal_and_wait(goal, rospy.Duration(1))

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.
        The `goal` has type:
            <class 'control_msgs.msg._GripperCommandGoal.GripperCommandGoal'>
        with a single attribute, accessed via `goal.command`, which consists of:
            position: 0.0
            max_effort: 0.0
        by default, and is of type:
            <class 'control_msgs.msg._GripperCommand.GripperCommand'>
        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        self._client.send_goal_and_wait(goal, rospy.Duration(1))


