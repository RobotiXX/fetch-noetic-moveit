import rospy
from geometry_msgs.msg import Twist, Vector3
import sys, os
# sys.path.insert(1, os.path.abspath("."))
sys.path.insert(1, os.path.abspath("./.."))
from lib.params import MOVE_NODE, CONTROL_RATE, MAX_LINEAR_VELOCITY
'''
This is the wrapper of Fetch's ability to move
'''
class FetchBaseMove:

    def __init__(self, publisher_node=MOVE_NODE,control_rate=CONTROL_RATE):
        # rospy.init_node("fetch_base_move")
        self.publisher = rospy.Publisher(publisher_node, Twist, queue_size=10) 
        self.rate = rospy.Rate(control_rate)
        self.max_linear_vel = MAX_LINEAR_VELOCITY

    def move(self, linear_x, angular_z):
        twist = Twist()
        twist.linear = Vector3(linear_x, 0, 0)
        twist.angular = Vector3(0, 0, angular_z)
        self.publisher.publish(twist)
        self.rate.sleep()
           
    def setRate(self, newRate):
        self.rate = rospy.Rate(newRate)

    def setMaxLinearVel(self, newMaxVel):
        self.max_linear_vel = newMaxVel