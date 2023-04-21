from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
from sensor_msgs.msg import Image
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from cv_bridge import CvBridge

import numpy as np
from scipy.spatial.transform import Rotation

from .aruco_detection import aruco_detection


class ArucoEstimation(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('View Frame', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Camera Parameters', Element('scalable:CalibrationParameters'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HasTransformationPose', 'skiros:hasA', 'Camera', 'View Frame', True))
        self.addPreCondition(self.getRelationCond('HasCalibrationParameters', 'skiros:hasA', 'Camera', 'Camera Parameters', True))

        self.addParam('Object', Element("sumo:Object"), ParamTypes.Required)
        
        # self.addParam('x', 0.015, ParamTypes.Required)
        # self.addParam('y', 0.025, ParamTypes.Required)
        # self.addParam('z', -0.03, ParamTypes.Required)

        self.addParam('x', 0.0, ParamTypes.Required)
        self.addParam('y', 0.0, ParamTypes.Required)
        self.addParam('z', 0.0, ParamTypes.Required)


class jp_pose_estimation(PrimitiveThreadBase):

    def createDescription(self):
        self.setDescription(ArucoEstimation(), self.__class__.__name__)

    def onInit(self):
        # self.poses = []
        self.hz = 5
        self.rate = rospy.Rate(self.hz)
        self.sub = RGBListener()
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

        self.poses = []

        return True

    def onPreempt(self):
        return True

    def preStart(self):
        self.preempt = False
        return True

    def run(self):
        # if self.poses:
        #     ts = np.array([t for t, _ in self.poses])
        #     qs = np.array([q for _, q in self.poses])
        #     qs[qs[:, -1] < 0] *= -1

        #     print('Std of %d poses:' % len(self.poses))
        #     print('Position std:   ', ts.std(axis=0), np.linalg.norm(ts.std(axis=0)))
        #     print('Orientation std:', qs.std(axis=0), np.linalg.norm(qs.std(axis=0)))

        object = self.params['Object'].value
        aruco_ids, markers = extract_object_markers(object, self.wmi)

        cam_params = self.params['Camera Parameters'].value
        fx = cam_params.getProperty('scalable:FocalLengthX').value
        fy = cam_params.getProperty('scalable:FocalLengthY').value
        cx = cam_params.getProperty('scalable:PixelCenterX').value
        cy = cam_params.getProperty('scalable:PixelCenterY').value
        k1 = cam_params.getProperty('scalable:Distortionk1').value
        k2 = cam_params.getProperty('scalable:Distortionk2').value
        p1 = cam_params.getProperty('scalable:Distortionp1').value
        p2 = cam_params.getProperty('scalable:Distortionp2').value
        k3 = cam_params.getProperty('scalable:Distortionk3').value

        if aruco_ids:
            done = False
            trials = 0
            while not done and trials < 5 * self.hz:
                img = self.sub.get()
                ids = aruco_detection(img, (fx, fy, cx, cy), (k1, k2, p1, p2, k3), aruco_ids)

                position_correction = np.array([self.params['x'].value, self.params['y'].value, self.params['z'].value], dtype=float)
                ids = {id: (pos + position_correction, quat) for id, (pos, quat) in ids.items()}

                if ids:
                    done = True
                trials += 1
                self.rate.sleep()
            if done:
                view_frame = self.params['View Frame'].value
                camera_frame = view_frame.getProperty('skiros:FrameId').value
                object_parent_frame = object.getProperty('skiros:BaseFrameId').value

                total_position = np.zeros(3, dtype=np.float64)
                total_quaternion = np.zeros(4, dtype=np.float64)
                for id_dict, (position, quaternion) in ids.items():
                    # Get pose of marker in object frame
                    marker = markers[id_dict]
                    marker_position, marker_orientation = get_object_pose(marker)
                    marker_rotation_matrix = quat2rot(marker_orientation)

                    # Get pose of marker in the parent frame of the object by using the
                    # representation of the pose in the camera frame
                    estimated_pose = make_pose_stamped(camera_frame, position, quaternion)
                    estimated_pose = self.buffer.transform(estimated_pose, object_parent_frame, rospy.Duration(1))
                    estimated_position, estimated_orientation = unpack_pose_stamped(estimated_pose)
                    estimated_rotation_matrix = quat2rot(estimated_orientation)

                    # Compute transformation between object parent frame and object such that the
                    # set of transformations is consistent
                    estimated_object_rotation_matrix = estimated_rotation_matrix @ marker_rotation_matrix.T
                    estimated_object_position = estimated_position - estimated_object_rotation_matrix @ marker_position

                    estimated_object_quaternion = rot2quat(estimated_object_rotation_matrix)

                    total_position += estimated_object_position
                    total_quaternion += estimated_object_quaternion

                total_position /= len(ids)
                total_quaternion /= np.linalg.norm(total_quaternion)

                # self.poses.append((total_position, total_quaternion))
                set_object_pose(object, list(total_position), list(total_quaternion))
                self.wmi.update_element_properties(object)

                result = "Found AruCo markers with ids: [%s]" % ", ".join([str(id) for id in ids.keys()])
                return self.success(result)
            else:
                return self.fail('Could not find AruCo marker within 5 seconds.', -1)
        else:
            return self.fail('Object does not have markers.', -1)

    def onEnd(self):
        return True

class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.image_callback)

    def image_callback(self, data):
        if self.image is not None: return
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


def extract_object_markers(object, wmi):
    relations = object.getRelations()

    aruco_ids = {}
    markers = {}
    for relation in relations:
        if relation['dst'] == '-1':
            continue
        marker = wmi.get_element(relation['dst'])
        if marker.type == 'scalable:Marker':
            id = marker.getProperty("skiros:Value").value
            dict = marker.getProperty("scalable:Dictionary").value
            size = marker.getProperty("skiros:Size").value

            if dict == "4x4":
                dict = 4
            elif dict == "5x5":
                dict = 5
            elif dict == "6x6":
                dict = 6
            elif dict == "7x7":
                dict = 7
            else:
                raise RuntimeError("Unsupported aruco dictionary: %s" % dict)

            aruco_ids[(id, dict)] = size
            markers[(id, dict)] = marker
    
    return aruco_ids, markers

