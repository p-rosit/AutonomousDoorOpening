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
import cv2 as cv
from scipy.spatial.transform import Rotation as rot

class JPPoseEstimation(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('View Frame', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Camera Parameters', Element('scalable:CalibrationParameters'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HasTransformationPose', 'skiros:hasA', 'Camera', 'View Frame', True))
        self.addPreCondition(self.getRelationCond('HasCalibrationParameters', 'skiros:hasA', 'Camera', 'Camera Parameters', True))

        self.addParam('Object', Element("sumo:Object"), ParamTypes.Required)

        self.addParam('Detection Time (s)', 1.0, ParamTypes.Required)
        self.addParam('x', -0.04, ParamTypes.Required)
        self.addParam('y', 0.0, ParamTypes.Required)
        self.addParam('z', 0.0, ParamTypes.Required)

class jp_pose_estimation_alt(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPPoseEstimation(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 20
        self.rate = rospy.Rate(self.hz)
        self.sub = RGBListener()

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

            # Dict of size and matching aruco dict to detect.
        self.ds = {4: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000),
                   5: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000),
                   6: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000),
                   7: cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)}
        self.p = cv.aruco.DetectorParameters()
        return True
    
    def onPreempt(self):
        self.preempted = True
        return self.fail('Preempted detection.', -1)

    def preStart(self):
        self.preempted = False
        return super().preempt()
    
    def run(self):
        thing = self.params['Object'].value
        aruco_markers = self.extract_object_markers()

        cam_params = self.params['Camera Parameters'].value
        calib, dist = cam_params.getData(':CameraCalibrationParameters')

        time_limit = self.params['Detection Time (s)'].value

        no_markers = True
        for _, markers in aruco_markers.items():
            if markers:
                no_markers = False
        if no_markers:
            return False, 'Object does not have markers'

        K = np.zeros((3, 3))
        # Create camera matrix
        # camera matrix (K):
        #            fx  0  cx
        #            0  fy  cy
        #            0  0   1
        calib = (585.756070948, 579.430235849, 319.5, 239.5)
        dist = (0, 0, 0, 0, 0)
        fx, fy, cx, cy = calib
        K[0, 0] = fx
        K[1, 1] = fy
        K[0, 2] = cx
        K[1, 2] = cy
        K[2, 2] = 1

        ind = 0
        total_position = np.zeros(3, dtype=np.float64)
        total_quaternion = np.zeros(4, dtype=np.float64)
        detected = set()
        while ind < self.hz * time_limit:
            ind += 1
            img = self.sub.get(time_limit=1.0)

            view_frame = self.params['View Frame'].value
            camera_frame = view_frame.getProperty('skiros:FrameId').value
            object_parent_frame =thing.getProperty('skiros:BaseFrameId').value
            ids = self.aruco_detection(img, K, dist, aruco_markers)

            if not ids:
                continue

            sub_position = np.zeros(3, dtype=np.float64)
            sub_quaternion = np.zeros(4, dtype=np.float64)
            for (id, dictionary), (marker, position, quaternion) in ids.items():
                detected.add((id, dictionary))

                # Get pose of marker in object frame
                marker_position = np.array(marker.getData(':Position'))
                marker_orientation = np.array(marker.getData(':Orientation'))
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

                sub_position += estimated_object_position
                sub_quaternion += estimated_object_quaternion
            total_position += sub_position / len(ids)
            total_quaternion += sub_quaternion / len(ids)

        total_position /= ind
        total_quaternion /= np.linalg.norm(total_quaternion)
        thing.setData(':Position', total_position)
        thing.setData(':Orientation', total_quaternion)
        print(total_position)
        print(total_quaternion)

        self.wmi.update_element_properties(thing)

        if not detected:
            return False, 'Object was not detected in %f seconds.' % time_limit

        return True, 'Detected.'

    def extract_object_markers(self):
        relations = self.params['Object'].value.getRelations()

        aruco_markers = {4: dict(), 5: dict(), 6: dict(), 7: dict()}
        for relation in relations:
            if relation['dst'] == '-1':
                continue

            marker = self.wmi.get_element(relation['dst'])
            if marker.type == 'scalable:Marker':
                id = marker.getProperty("skiros:Value").value
                dictionary = marker.getProperty("scalable:Dictionary").value
                size = marker.getProperty("skiros:Size").value

                try:
                    index = int(dictionary[0])
                    if index < 4 or 7 < index:
                        raise ValueError
                except ValueError:
                    raise ValueError('Unsupported aruco dictionary: "%s"')

                if id in aruco_markers[index]:
                    raise RuntimeError('Object has duplicate aruco markers.')
                aruco_markers[index][id] = (marker, size)
        
        return aruco_markers

    def aruco_detection(self, rgb, K, dist, aruco_markers):
        detected_ids = dict()

        for dictionary, aruco_dict in self.ds.items():
            corners, ids, _ = cv.aruco.detectMarkers(rgb, aruco_dict, parameters=self.p)

            if not corners:
                continue

            ids = ids.reshape(-1)
            for i, (c, id) in enumerate(zip(corners, ids)):
                if id not in aruco_markers[dictionary]:
                    continue

                marker, l = aruco_markers[dictionary][id]
                coords = cv.undistortPoints(c, K, dist, P=K)
                obj_points = np.array([
                    [l, 0, 0],
                    [l, l, 0],
                    [0, l, 0],
                    [0, 0, 0]
                ], dtype=float)

                _, r, t = cv.solvePnP(obj_points, coords, K, None)

                position = t.reshape(-1)
                rotation_matrix = cv.Rodrigues(r)
                quaternion = rot.from_matrix(rotation_matrix[0]).as_quat()

                detected_ids[(dictionary, id)] = (marker, position, quaternion)

        return detected_ids

class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.bridge = CvBridge()
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
        self.image = 0.0

        rospy.Subscriber(topic, Image, callback=self.image_callback)

    def image_callback(self, data):
        if self.image is not None: return
        self.image = self.bridge.imgmsg_to_cv2(data)

    def get(self, time_limit=1.0):
        self.image = None
        ind = 0

        while self.image is None and ind < time_limit * self.hz:
            ind += 1
            self.rate.sleep()
        
        if ind == time_limit * self.hz - 1:
            raise RuntimeError('Did not receive image within %f seconds.' % time_limit)

        return self.image

def extract_object_markers(thing, wmi):
    return 0, 0

def quat2rot(q):
    return rot.from_quat(q).as_matrix()

def rot2quat(R):
    return rot.from_matrix(R).as_quat()

def make_pose_stamped(frame, position, orientation):
    pose = PoseStamped()

    pose.header.stamp = rospy.Time(0)
    # pose.header.stamp.secs -= 1
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
