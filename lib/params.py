import numpy as np
import geometry_msgs
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# import rospy

### JOINT STATES
JOINT_STATES = "joint_states"

### BASE POS
BASE_POS = [0.0, 0.0, 0.0]

### GRIPPER Params
CLOSED_POS = 0.0   # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).
MIN_EFFORT = 35   # Min grasp force, in Newtons
MAX_EFFORT = 100  # Max grasp force, in Newtons
GRIPPER_CONTROL_GROUP = 'gripper_controller/gripper_action'

### MANIPULATOR Params
# ARM_AND_TORSO
ARM_REST_POSITION = [0.05, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                     1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
ARM_READY_POSITION = [0.35, -0.6074564456939697, 1.2425246238708496, 0.8049564361572266,
                      1.6225686073303223, -2.9548306465148926, -1.582684874534607, -1.0243149995803833]
MAX_JOINT_VEL = [0.1, 1.25, 1.45, 1.57, 1.52, 1.57, 2.26, 2.26]
ARM_AND_TORSO_JOINTS = ['torso_lift_joint',
                        'shoulder_pan_joint',
                        'shoulder_lift_joint',
                        'upperarm_roll_joint',
                        'elbow_flex_joint',
                        'forearm_roll_joint',
                        'wrist_flex_joint',
                        'wrist_roll_joint']
# HEAD
HEAD_JOINTS = ['head_pan_joint',
               'head_tilt_joint']

# tilt_joint ranges [-0.785 (U), 1.5708 (D) rad] = [-45, 90] 
# pan_joint ranges [-1.5708 (R), 1.5708 (L) rad] = [-90, 90] 

ARM_TORSO_CONTROL_GROUP = 'arm_with_torso_controller/follow_joint_trajectory'
HEAD_CONTROL_GROUP = 'head_controller/follow_joint_trajectory'

### MOVE Params
# /cmd_vel allows controller commandd to take over the robot
MOVE_NODE = '/cmd_vel' # /cmd_vel OR /teleop/cmd_vel OR /base_controller/command 
CONTROL_RATE = 10
MAX_LINEAR_VELOCITY = 4.8

### VISION PARAM
VISION_IMAGE_TOPIC = "/head_camera/rgb/image_raw" # "/head_camera/rgb/image_rect_color"
VISION_CAMERA_INFO_TOPIC = "/head_camera/rgb/camera_info"
REALSENSE_IMAGE_TOPIC = "/camera/color/image_raw"

# Calibration matrix for head camera
'''  
[fx  0 cx]
[ 0 fy cy]
[ 0  0  1]
'''
CAMERA_CALIBRATION_MATRIX = [574.0527954101562, 0.0, 319.5, 0.0, 574.0527954101562, 239.5, 0.0, 0.0, 1.0] # FETCH_CAM

# CAMERA_CALIBRATION_MATRIX = [602.931396484375, 0.0, 315.2467041015625, 0.0, 603.0493774414062, 234.35330200195312, 0.0, 0.0, 1.0] # REALSENSE

# Intrinsic parameters for camera (fx, fy, cx, cy)
INTRINSIC_PARAM_CAMERA = (CAMERA_CALIBRATION_MATRIX[0], CAMERA_CALIBRATION_MATRIX[4], CAMERA_CALIBRATION_MATRIX[2], CAMERA_CALIBRATION_MATRIX[5])

#APRILTAG PARAM in m
# APRIL_TAG_SIZE = 0.06 # for cabinet project tags 
APRIL_TAG_SIZE = 0.20 # for cabinet project tags 

### TRANSFORMATION MATRIX (TRANSLATION) in meters

'''
At [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] joint pos
'''

# ODOM2BASE = []

BASE2TORSO = [-0.086875, 0.000, 0.37743] 

# TORSO TO HEAD CAMEAR

TORSO2HEADPAN = [0.053125, 0.000, 0.603001417713939]

HEADPAN2HEADTILT = [0.14253, 0.000, 0.057999]

HEADTILT2HEADCAM = [0.055, 0.000, 0.022]

# TORSO TO ARM

TORSO2SHOULDERPAN = [0.119525, 0.000, 0.34858]

SHOULDERPAN2SHOULDERLIFT = [0.117, 0.000, 0.0599999999999999]

SHOULDERLIFT2UPPERARM = [0.219, 0.000, 0.000]

UPPERARM2ELBOW = [0.133, 0.000, 0.000]

ELBOW2FOREARM = [0.197, 0.000, 0.000]

FOREARM2WRISTFLEX = [0.1245, 0.000, 0.000]

WRISTFLEX2WRISTROLL = [0.1385, 0.000, 0.000]

WRISTROLL2GRIPPER = [0.16645, 0.000, 0.000]

# MATRIX TO MAP END EFFECTOR TO NEW BASE POSITION

BASE_DEST_TRANSFORM = np.array([[ 9.99999338e-01, -2.56295201e-05, -1.15025930e-03,  9.23685022e-01],
                                [ 2.63510869e-05,  9.99999803e-01,  6.27297245e-04,  1.17883454e-04],
                                [ 1.15024299e-03, -6.27327141e-04,  9.99999142e-01,  8.30216081e-01],
                                [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])




# INITIAL POSE TO SUGGEST LOCALIZATION
INITIAL_POSE_SUGGESTION = PoseWithCovarianceStamped()
INITIAL_POSE_SUGGESTION.header.frame_id = "map"
INITIAL_POSE_SUGGESTION.pose.pose.position.x= 0.024936
INITIAL_POSE_SUGGESTION.pose.pose.position.y= 0.38928
INITIAL_POSE_SUGGESTION.pose.pose.position.z=0
INITIAL_POSE_SUGGESTION.pose.covariance=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
INITIAL_POSE_SUGGESTION.pose.pose.orientation.z= -0.0739386
INITIAL_POSE_SUGGESTION.pose.pose.orientation.w= 0.997263

# WAYPOINT A (IN FRONT OF CABINET)
WAYPOINT_CABINET_POSE =  MoveBaseGoal()
WAYPOINT_CABINET_POSE.target_pose.header.frame_id = "map"
WAYPOINT_CABINET_POSE.target_pose.pose.position.x = 0.024936
WAYPOINT_CABINET_POSE.target_pose.pose.position.y = 0.38928
WAYPOINT_CABINET_POSE.target_pose.pose.orientation.z = -0.0739386
WAYPOINT_CABINET_POSE.target_pose.pose.orientation.w = 0.997263

# # IN FRONT OF SUPPORT TABLE
WAYPOINT_SIDE_POSE =  MoveBaseGoal()
WAYPOINT_SIDE_POSE.target_pose.header.frame_id = "map"
WAYPOINT_SIDE_POSE.target_pose.pose.position.x = 0.141899
WAYPOINT_SIDE_POSE.target_pose.pose.position.y = 1.17533
WAYPOINT_SIDE_POSE.target_pose.pose.orientation.z = -0.00755
WAYPOINT_SIDE_POSE.target_pose.pose.orientation.w = 0.999932


# IN FRONT OF MAIN TABLE
WAYPOINT_TABLE_POSE =  MoveBaseGoal()
WAYPOINT_TABLE_POSE.target_pose.header.frame_id = "map"
WAYPOINT_TABLE_POSE.target_pose.pose.position.x = 0.210075
WAYPOINT_TABLE_POSE.target_pose.pose.position.y = 1.95543
WAYPOINT_TABLE_POSE.target_pose.pose.orientation.z = -0.011683
WAYPOINT_TABLE_POSE.target_pose.pose.orientation.w = 0.999932

# TRANSFORMATION FROM ATTACHED REALSENSE CAMERA FRAME BACK TO ROBOT CAMERA FRAME
# REALSENSE2CAMERA = np.array(
#                             [[ 9.99440706e-01, -3.34342515e-02, -6.52692147e-04, -7.80515837e-03],
#                              [ 3.34403181e-02,  9.99324412e-01,  1.52468131e-02, -5.74749061e-02],
#                              [ 1.42485410e-04, -1.52601119e-02,  9.99883548e-01,  9.00588934e-02],
#                              [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]]
#                             )

REALSENSE2CAMERA = np.array(
                            [[ 1, 0, 0, -0.008], # x
                             [ 0, 1, 0, -0.04], # y
                             [ 0, 0, 1,  0.055], # z
                             [ 0, 0, 0,  1 ]]
                            )

# REALSENSE2CAMERA = np.array(
#                             [[ 1, 0, 0, 0], # x
#                              [ 0, 1, 0, 0], # y
#                              [ 0, 0, 1,  0], # z
#                              [ 0, 0, 0,  1 ]]
#                             )


# ESTIMATION OF TABLE POSITION IN BASEFRAME (For MoveIt PlanningScene)
TABLE_POSITION_BF = Pose()
TABLE_POSITION_BF.position.x = 0.65
TABLE_POSITION_BF.position.y = -0.35
TABLE_POSITION_BF.position.z = 0.86 #0.88
TABLE_POSITION_BF.orientation.x = 0 #q[0]
TABLE_POSITION_BF.orientation.y = 0 #q[1]
TABLE_POSITION_BF.orientation.z = 0 #q[2]
TABLE_POSITION_BF.orientation.w = 0 #q[3]


# PRESET POSES TO PLACE OBJECT BACK TO CABINET
'''
[0]|[1]
[2]|[3]   ----> Position order
'''
PRESET_VIDEO_1 = []

# POS 1
PRESET_DROP_1 = Pose()
PRESET_DROP_1.position.x = 1.2913904973755925
PRESET_DROP_1.position.y = 0.25515919616515914
PRESET_DROP_1.position.z = 0.8328768997353683

PRESET_DROP_1.orientation.x = 0 #q[0]
PRESET_DROP_1.orientation.y = 0 #q[1]
PRESET_DROP_1.orientation.z = 0 #q[2]
PRESET_DROP_1.orientation.w = 0 #q[3]

# POS 2
PRESET_DROP_2 = Pose()
PRESET_DROP_2.position.x = 1.093827803443677
PRESET_DROP_2.position.y = 0.24735415884635165
PRESET_DROP_2.position.z = 0.8339260620965698

PRESET_DROP_2.orientation.x = 0 #q[0]
PRESET_DROP_2.orientation.y = 0 #q[1]
PRESET_DROP_2.orientation.z = 0 #q[2]
PRESET_DROP_2.orientation.w = 0 #q[3]

# POS 3
PRESET_DROP_3 = Pose()
PRESET_DROP_3.position.x = 1.2797416277432574
PRESET_DROP_3.position.y = 0.040405348110622
PRESET_DROP_3.position.z = 0.8403880811277166

PRESET_DROP_3.orientation.x = 0 #q[0]
PRESET_DROP_3.orientation.y = 0 #q[1]
PRESET_DROP_3.orientation.z = 0 #q[2]
PRESET_DROP_3.orientation.w = 0 #q[3]

# POS 4
PRESET_DROP_4 = Pose()
PRESET_DROP_4.position.x = 1.1034966869796508
PRESET_DROP_4.position.y = 0.013937492670144361
PRESET_DROP_4.position.z = 0.8328504640564715

PRESET_DROP_4.orientation.x = 0 #q[0]
PRESET_DROP_4.orientation.y = 0 #q[1]
PRESET_DROP_4.orientation.z = 0 #q[2]
PRESET_DROP_4.orientation.w = 0 #q[3]

PRESET_VIDEO_1.append(PRESET_DROP_1)
PRESET_VIDEO_1.append(PRESET_DROP_2)
PRESET_VIDEO_1.append(PRESET_DROP_3)
PRESET_VIDEO_1.append(PRESET_DROP_4)