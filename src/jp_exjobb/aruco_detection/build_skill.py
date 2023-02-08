from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import numpy as np

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_geometry_msgs import PoseStamped, transform_to_kdl
from tf.transformations import quaternion_multiply as qm, quaternion_inverse as qi
from cv_bridge import CvBridge
import PyKDL as pk

from scipy.spatial.transform import Rotation

from .aruco_detection import aruco_detection
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy


class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.image_callback)

    def image_callback(self, data):
        if self.image is not None: return

        # TASK:
        #     The variable 'data' is a ROS message containing a RGB image.
        #     Transform it into an OpenCV image using self.bridge.

        #     The result should be stored in self.image.

        # HINTS:
        #     The resulting image should be an 8-bit image in BGR color space.

        self.image = self.bridge.imgmsg_to_cv2(data)

    def get(self):
        self.image = None
        while self.image is None:
            self.rate.sleep()
        return self.image


def make_pose_stamped(frame, position, orientation):
    pose = PoseStamped()

    pose.header.stamp = rospy.Time.now()
    pose.header.stamp.secs -= 1
    pose.header.frame_id = frame

    pose.pose.position.x = position[0]
    pose.pose.position.y = position[1]
    pose.pose.position.z = position[2]
    pose.pose.orientation.x = orientation[0]
    pose.pose.orientation.y = orientation[1]
    pose.pose.orientation.z = orientation[2]
    pose.pose.orientation.w = orientation[3]

    return pose


def unpack_pose_stamped(pose):
    position = np.zeros(3, dtype=np.float64)
    orientation = np.zeros(4, dtype=np.float64)

    position[0] = pose.pose.position.x
    position[1] = pose.pose.position.y
    position[2] = pose.pose.position.z
    orientation[0] = pose.pose.orientation.x
    orientation[1] = pose.pose.orientation.y
    orientation[2] = pose.pose.orientation.z
    orientation[3] = pose.pose.orientation.w

    return position, orientation


def get_object_pose(object):
    position = np.zeros(3, dtype=np.float64)
    orientation = np.zeros(4, dtype=np.float64)

    position[0] = object.getProperty('skiros:PositionX').value
    position[1] = object.getProperty('skiros:PositionY').value
    position[2] = object.getProperty('skiros:PositionZ').value

    orientation[0] = object.getProperty('skiros:OrientationX').value
    orientation[1] = object.getProperty('skiros:OrientationY').value
    orientation[2] = object.getProperty('skiros:OrientationZ').value
    orientation[3] = object.getProperty('skiros:OrientationW').value

    return position, orientation


def quat2rot(q):
    return Rotation.from_quat(q).as_matrix()


def rot2quat(R):
    return Rotation.from_matrix(R).as_quat()



def set_object_pose(object, position:list, orientation:list):
    object.setData(':Position', position)
    object.setData(':Orientation', orientation)
    # object.setProperty('skiros:PositionX', position[0])
    # object.setProperty('skiros:PositionY', position[1])
    # object.setProperty('skiros:PositionZ', position[2])
    # object.setProperty('skiros:OrientationX', orientation[0])
    # object.setProperty('skiros:OrientationY', orientation[1])
    # object.setProperty('skiros:OrientationZ', orientation[2])
    # object.setProperty('skiros:OrientationW', orientation[3])


class SkillBill(SkillDescription):
    def createDescription(self):
        self.addParam('Number', 0, ParamTypes.Required)
        self.addParam('Object', Element("skiros:Product"), ParamTypes.Required)

class skill_bill(PrimitiveBase):

    def createDescription(self):
        self.setDescription(SkillBill(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        print('---')

        object = self.params['Object'].value
        data = int(self.params['Number'].value)

        print(data)
        object.setProperty('skiros:BaseFrameId', str(data))
        self.wmi.update_element_properties(object)
        self.wmi.update_element(object)

        print('---')
        return self.success('Done')

    def onEnd(self):
        return True
