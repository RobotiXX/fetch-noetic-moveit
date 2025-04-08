# Fetch Noetic with MoveIt! repo

----------
## Last update [04/08/2025]

----------
## Installation
Fetch Robot is currently on Ubuntu 20.04 Noetic. To use Fetch for mobile manipulation task, we need certain packages (already installed):

```sh
sudo apt install ros-noetic-moveit
```

----------
## MoveIt!

For detailed documentation, check the official link for MoveIt Noetic [**[MoveIt Documentation]**](https://moveit.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)

----------
## Quick start

- First, we need to lauch these files.

```sh
roslaunch fetch_moveit_config move_group.launch
roslaunch fetch_navigation fetch_nav.launch map_file:=/home/fetch/[MAP_NAME].yaml 
```

- To activate (1) move_group for arm manipulation and (2) fetch base navigation.

### Fetch Navigation
The standard way to navigate Fetch base is to run move_base given a predefined map. To map the environment, run:

```sh
roslaunch fetch_navigation build_map.launch
```

- Drive the robot around the environment for mapping, it is good to run RVIZ on another computer to see the map being bullt. To finalize and save the map, run:

```sh
rosrun map_server map_saver -f <map_directory/map_name>
```

- When we program Fetch, it is important to give it an initial estimation of its pose for localization. We can either do it with RVIZ or through Python/C++ code. 

```sh
# Import libraries
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

# Initialize a publisher
pub = rospy.Publisher('/initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=10)

def kick_start_localization():
    rate = rospy.Rate(2)
    rate.sleep()
    pub.publish(INITIAL_POSE_SUGGESTION)

kick_start_localization()
```

- To move within the map

```sh
# Create a movebase client
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()

# Move to goal through client
def movebase_client(client, goal):
    goal.target_pose.header.stamp = rospy.Time.now()

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()
```

### Fetch manipulation with MoveIt!

To manipulate arm with MoveIt!, there are a few steps to follow:

Import moveit python libraries, packages

```sh
from moveit_msgs.msg import MoveItErrorCodes, CollisionObject, AttachedCollisionObject, ObjectColor, Grasp
from moveit_python import MoveGroupInterface, PickPlaceInterface
from moveit_commander import PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
```

**Step 1**: Building a 3D perception. 

There are two ways to give a planning scene. 

- First way is to use depth or lidar sensor to input an [**OctoMap**](http://docs.ros.org/en/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/perception_configuration.html). We haven't tested this out yet, but its possible.

- Second way, which is the standard way, is to build a PlanningScene with MoveIt!

```sh
# Create a PlanningSceneInterface Object
SCENE = PlanningSceneInterface()

# Reset the scene for re-create
SCENE.clear()

# Add a desk to the planning scene
w_x = 0.6
w_y = 2
h_z = 0.7 / 2

desk = create_collision_object(id=f"Desk", dimensions=[w_x, w_y, 0.85], pos=[0.65 + w_x/2, -0.23 + w_y/2,  0.587-0.28], frameID="base_link")

SCENE.add_object(desk)
```

**Step 2**: Use move_group to manipulate

```sh
move_group = MoveGroupInterface("arm_with_torso", "base_link")
```

With MoveIt!, you can control the robot whole body on joint level, for example

```sh
joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]

move_group.moveToJointPosition(joint_names, [0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], wait=False)
    
move_group.get_move_action().wait_for_result()
result = move_group.get_move_action().get_result()

if result:
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        rospy.loginfo("Arn-ready-position!")
    else:
        rospy.logerr("Arm goal in state: %s",
                    move_group.get_move_action().get_state())
else:
    rospy.logerr("MoveIt! failure no result returned.")
```

However, it is easier to aim for a goal end effector pose and let MoveIt! handles the rest of the arm with its FastIK (Inverse Kinematic) library. 

```sh
move_group.moveToPose(target_pose_stamped, gripper_frame)
result = move_group.get_move_action().get_result()
if result:
    # Checking the MoveItErrorCode
    if result.error_code.val == MoveItErrorCodes.SUCCESS:
        rospy.loginfo("Target pose executed!")
    else:
        rospy.logerr("Arm goal in state: %s",
                                 move_group.get_move_action().get_state())
else:
    rospy.logerr("MoveIt! failure no result returned.")
```

Again, the quality of the trajectory HEAVILY depends on how good the planning scene, or the 3D representation of the scene, is. This is where all the collision checking happens.

----------
## The Fetch API

```lib/params.py``` contains all the current parameters and transformations among different robot frames. Ideally, this is the only file we change when we do experiment.

```src/fetch_controller_python``` is a wrapper we design for convenience. It allows users to control the whole body of Fetch, and also give the ability to convert/inverse frame information. For manipulation, it is not as good as MoveIt! as it is not a planner but still worth going through. 
