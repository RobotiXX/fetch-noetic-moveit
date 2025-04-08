import apriltag
from lib.params import INTRINSIC_PARAM_CAMERA, APRIL_TAG_SIZE
import numpy as np
import cv2

class BoardTracker:
    '''
    Visual tracker to track movements of the board
    '''
    def __init__(self, family="tag36h11") -> None:
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families=family))

    def detect(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
        results = self.detector.detect(gray)

        return results

    def getTransformation(self, detection):

        '''
        output matrices are already in numpy form
        '''
        pose_transform_matrix, _, _ = self.detector.detection_pose(detection=detection, camera_params=INTRINSIC_PARAM_CAMERA, tag_size=APRIL_TAG_SIZE)
        pose_rotation_matrix = pose_transform_matrix[:3,:3]
        pose_translation_matrix = pose_transform_matrix[:3,3:]

        yaw = -1 * np.arctan2(pose_rotation_matrix[0, 2], pose_rotation_matrix[1, 2])
        pitch = np.arccos(pose_rotation_matrix[2, 2])
        roll = np.arctan2(pose_rotation_matrix[2, 0], pose_rotation_matrix[2, 1])


        return pose_transform_matrix, pose_rotation_matrix, pose_translation_matrix #, (roll, pitch, yaw)
    
    def rotationMatrixToEulerAngles(R) :    
        sy = np.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    
        singular = sy < 1e-6
    
        if  not singular :
            x = np.atan2(R[2,1] , R[2,2])
            y = np.atan2(-R[2,0], sy)
            z = np.atan2(R[1,0], R[0,0])
        else :
            x = np.atan2(-R[1,2], R[1,1])
            y = np.atan2(-R[2,0], sy)
            z = 0
    
        return np.array([x, y, z])

    
    def detectPose(self, detection):
        return self.detector.detection_pose(detection, camera_params=INTRINSIC_PARAM_CAMERA, tag_size=APRIL_TAG_SIZE)
