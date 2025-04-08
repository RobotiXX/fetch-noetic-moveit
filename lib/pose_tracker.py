import rospy
import sys, time, os
sys.path.insert(1, os.path.abspath("."))
from lib.params import BASE_DEST_TRANSFORM, VISION_IMAGE_TOPIC
from lib.board_tracker import BoardTracker
from lib.utils import *
from src.fetch_controller_python.fetch_robot import FetchRobot
# from src.fetch_controller_python.fetch_gripper import Gripper
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import std_msgs
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray
# geometry_msgs/PoseArray.msg
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Pose, PoseStamped
import tf


class PoseTracker:
    def __init__(self) -> None:
        self.hashTable = {}
        self.__ids = []
        self.__poses = []
        rospy.Subscriber("/tagPoses", PoseArray, self.__pose_callback)
        rospy.Subscriber("tagID", Int32MultiArray, self.__id_callback)

    def __pose_callback(self, message):
        self.__poses = message.poses

    def __id_callback(self, message):
        self.__ids = message.data

    def load_hashTable(self, target=1):
        # remove table
        self.hashTable = {}
        self.__ids = []
        self.__poses = []

        # wait
        rospy.sleep(2)

        # now reading the table again
        while len(self.__poses) != len(self.__ids) :
            print("Waiting to sync...")

        for ind in range(len(self.__ids)):
            self.hashTable[self.__ids[ind]] = self.__poses[ind]

        print("Succeeded!")
    
    def is_empty(self):
        return len(self.hashTable) == 0

    def get(self, id):
        if self.is_empty():
            print("\nThe hash table is empty\n")
            return None
        return self.hashTable[id]
    
    def get_by_index(self, index):
        if self.is_empty():
            print("\nThe hash table is empty\n")
            return None
        return list(self.hashTable.values())[index]