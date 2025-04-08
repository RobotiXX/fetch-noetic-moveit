#!/usr/bin/env python3
import sys, os
# sys.path.insert(1, os.path.abspath("."))
sys.path.insert(1, os.path.abspath("./.."))
import rospy
import actionlib
from control_msgs.msg    import FollowJointTrajectoryGoal
from control_msgs.msg    import FollowJointTrajectoryResult
from control_msgs.msg    import FollowJointTrajectoryFeedback
from control_msgs.msg    import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg	 import JointState
from lib.params import HEAD_JOINTS, ARM_AND_TORSO_JOINTS, ARM_READY_POSITION, ARM_REST_POSITION, ARM_TORSO_CONTROL_GROUP, HEAD_CONTROL_GROUP, MAX_JOINT_VEL

class FetchManipulator:
    
    def __init__(self, arm_control_group=ARM_TORSO_CONTROL_GROUP, 
                 head_control_group=HEAD_CONTROL_GROUP):
        # head
        self.head_client = actionlib.SimpleActionClient(head_control_group, FollowJointTrajectoryAction)
        self.head_joints = HEAD_JOINTS
        # arm_and_torso
        self.arm_client = actionlib.SimpleActionClient(arm_control_group, FollowJointTrajectoryAction)
        self.arm_joints = ARM_AND_TORSO_JOINTS


    def execute(self, manipulation_matrix, execution_time=6):
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions =  manipulation_matrix[0]
        arm_trajectory.points[0].velocities = manipulation_matrix[1]
        arm_trajectory.points[0].accelerations = manipulation_matrix[2]
        arm_trajectory.points[0].time_from_start = rospy.Duration(execution_time)
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
            
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
            
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0)
        
        # Send the goal to the action server
        self.arm_client.send_goal(arm_goal)

    def lookAt(self, manipulation_matrix, execution_time=1):
        # Create a single-point arm trajectory with the arm_goal as the end-point
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = self.head_joints
        head_trajectory.points.append(JointTrajectoryPoint())
        head_trajectory.points[0].positions =  manipulation_matrix[0]
        head_trajectory.points[0].velocities = manipulation_matrix[1]
        head_trajectory.points[0].accelerations = manipulation_matrix[2]
        head_trajectory.points[0].time_from_start = rospy.Duration(execution_time)
        # Create an empty trajectory goal
        head_goal = FollowJointTrajectoryGoal()
            
        # Set the trajectory component to the goal trajectory created above
        head_goal.trajectory = head_trajectory
            
        # Specify zero tolerance for the execution time
        head_goal.goal_time_tolerance = rospy.Duration(0)
        
        # Send the goal to the action server
        self.head_client.send_goal(head_goal)

    def reset(self):
        matrix = [ARM_READY_POSITION, [0.0 for _ in self.arm_joints], [0.0 for _ in self.arm_joints]]
        self.execute(matrix, execution_time=4)
        # lookAtMatrix = [[0.0, 0.0], [0.0 for _ in self.head_joints], [0.0 for _ in self.head_joints]]
        # self.lookAt(lookAtMatrix, execution_time=4)

    def rest(self):
        self.reset()
        matrix = [ARM_REST_POSITION, [0.0 for _ in self.arm_joints], [0.0 for _ in self.arm_joints]]
        self.execute(matrix, execution_time=3)


    