import numpy as np
from shape_msgs.msg import MeshTriangle, Mesh, SolidPrimitive, Plane
from moveit_msgs.msg import MoveItErrorCodes, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, PoseStamped

class Algorithms:
    '''
    Optimization algorithm to dictate joints' behaviors
    '''

    def __init__(self) -> None:
        pass


class MatrixOperation:
    def __init__(self, axes=[0, 0, 0]) -> None:
        self.axes = self.vectorizeXYZ(axes)

    def vectorizeXYZ(self, axes):
        try:
            return np.array([[axes[0], axes[1], axes[2], 1]])
        except Exception as e:
            print(f"Failed to vectorize XYZ due to {e}. Return axes")
            return axes

    def setAxes(self, axes=[0, 0, 0]) -> None:
        self.axes = self.vectorizeXYZ(axes)

    def resetAxes(self) -> None:
        self.setAxes()

    def getAxes(self, M):
        return M[:3, 3]

    def trig(self, angle, unit='radian'):
        if unit != 'radian':
            r = np.radians(angle)
            return np.cos(r), np.sin(r)
        else:
            return np.cos(angle), np.sin(angle)

#     def getTransformMatrix(self, rotation=(0,0,0), translation=(0,0,0)):
#         xC, xS = self.trig(rotation[0])
#         yC, yS = self.trig(rotation[1])
#         zC, zS = self.trig(rotation[2])
#         dX = translation[0]
#         dY = translation[1]
#         dZ = translation[2]
#         return np.array([[yC*xC, -zC*xS+zS*yS*xC, zS*xS+zC*yS*xC, dX],
#             [yC*xS, zC*xC+zS*yS*xS, -zS*xC+zC*yS*xS, dY],
#             [-yS, zS*yC, zC*yC, dZ],
#             [0, 0, 0, 1]])
    def rotation_matrix_euler(self, rotation_angles):
        """
        Generate a 4x4 rotation matrix for Euler angles (a, b, c).
        """
        a, b, c = rotation_angles
        cos_a, sin_a = np.cos(a), np.sin(a)
        cos_b, sin_b = np.cos(b), np.sin(b)
        cos_c, sin_c = np.cos(c), np.sin(c)
        # Rotation matrix around x-axis
        Rx = np.array([
            [1, 0, 0, 0],
            [0, cos_a, -sin_a, 0],
            [0, sin_a, cos_a, 0],
            [0, 0, 0, 1]
        ])
        # Rotation matrix around y-axis
        Ry = np.array([
            [cos_b, 0, sin_b, 0],
            [0, 1, 0, 0],
            [-sin_b, 0, cos_b, 0],
            [0, 0, 0, 1]
        ])
        # Rotation matrix around z-axis
        Rz = np.array([
            [cos_c, -sin_c, 0, 0],
            [sin_c, cos_c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # Combine the rotation matrices in the order of z, y, x
        R = np.dot(Rz, np.dot(Ry, Rx))
        return R

    def translation_matrix(self, translation_vector):
        """
        Generate a 4x4 translation matrix for translation along the x, y, and z axes.
        """
        tx, ty, tz = translation_vector
        T = np.array([
            [1, 0, 0, tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0, 1]
        ])
        return T

    def getTransformMatrix(self, rotation=(0, 0, 0), translation=(0, 0, 0)):
        """
        Combine rotation and translation into a single transformation matrix.
        """
        rotation_matrix = self.rotation_matrix_euler(rotation)
        translation_matrix_4x4 = self.translation_matrix(translation)
        # Combine the rotation and translation matrices
        combined_matrix = np.dot(translation_matrix_4x4, rotation_matrix)
        return combined_matrix

    def getInverseMatrix(self, A):
        return np.linalg.inv(A)

    def combineTransform(self, newtransform, curr):
        # M2 * (M1 * A)
        return np.dot(newtransform, curr)

    def transform(self, old, transform_matrix):
        new = np.dot(transform_matrix, np.transpose(old))
        return np.transpose(new)


def create_collision_object(id, dimensions, pos, frameID, solidType="BOX", orientation=None):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = frameID

    solid = SolidPrimitive()
    if solidType == "BOX":
        solid.type = solid.BOX
    elif solidType == "CYLINDER":
        solid.type = solid.CYLINDER
    else:
        solid.type = solid.SPHERE
    
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pos[0]
    object_pose.position.y = pos[1]
    object_pose.position.z = pos[2]
    if orientation is not None:
        object_pose.orientation.x = orientation[0]
        object_pose.orientation.y = orientation[1]
        object_pose.orientation.z = orientation[2]
        object_pose.orientation.w = orientation[3]

    if solidType == "CYLINDER":
        object_pose.position.z = object_pose.position.z + 0.04

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object
