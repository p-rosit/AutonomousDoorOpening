from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_geometry_msgs import PoseStamped, transform_to_kdl
from tf.transformations import quaternion_multiply as qm, quaternion_inverse as qi
from cv_bridge import CvBridge
import PyKDL as pk

import numpy as np
from scipy.spatial.transform import Rotation

from .aruco_detection import aruco_detection


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
    # print(object)
    # print('position', position)
    # print('orientation', orientation)
    object.setData(':Position', position)
    object.setData(':Orientation', orientation)
    # object.setProperty('skiros:PositionX', position[0])
    # object.setProperty('skiros:PositionY', position[1])
    # object.setProperty('skiros:PositionZ', position[2])
    # object.setProperty('skiros:OrientationX', orientation[0])
    # object.setProperty('skiros:OrientationY', orientation[1])
    # object.setProperty('skiros:OrientationZ', orientation[2])
    # object.setProperty('skiros:OrientationW', orientation[3])


class ArucoEstimation(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Object', Element("skiros:Product"), ParamTypes.Required)
        # self.addParam('TestVis', Element("skiros:Product"), ParamTypes.Required)
        self.addParam('x', 0.015, ParamTypes.Required)
        self.addParam('y', 0.01, ParamTypes.Required)
        self.addParam('z', -0.04, ParamTypes.Required)
        self.addParam('ox', 0.0, ParamTypes.Required)
        self.addParam('oy', 0.0, ParamTypes.Required)
        self.addParam('oz', 0.0, ParamTypes.Required)
        self.addParam('ow', 1.0, ParamTypes.Required)


class aruco_marker(PrimitiveBase):
    def __init__(self, *args, **kwargs):
        self.hz = 5
        self.rate = rospy.Rate(self.hz)
        self.sub = RGBListener(topic='/img')
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

        super(aruco_marker, self).__init__(*args, **kwargs)

    def createDescription(self):
        self.setDescription(ArucoEstimation(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        object = self.params['Object'].value
        # test_vis = self.params['TestVis'].value
        relations = object.getRelations()

        # ob_pose = make_pose_stamped('map', np.array([0, 0, 0]), np.array([1, 0, 0, 0]))
        # ob_pose = self.buffer.transform(ob_pose, object.getProperty('skiros:BaseFrameId').value, rospy.Duration(1))

        # object.setData(':PoseStampedMsg', ob_pose)

        # ob_pos, ob_quat = unpack_pose_stamped(ob_pose)
        # print(ob_pos, ob_quat)

        aruco_ids = {}
        markers = {}
        for relation in relations:
            if relation['dst'] == '-1':
                continue
            marker = self.wmi.get_element(relation['dst'])
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

        print(aruco_ids)

        if aruco_ids:
            done = False
            trials = 0
            while not done and trials < 5 * self.hz:
                img = self.sub.get()
                ids = aruco_detection(img, (672.043304, 670.234373, 488.053352, 281.019651), (0, 0, 0, 0, 0), aruco_ids)

                print(ids)

                posss = np.array([self.params['x'].value, self.params['y'].value, self.params['z'].value], dtype=float)
                # print(posss)
                ids = {id: (pos + posss, quat) for id, (pos, quat) in ids.items()}

                # quattt = np.array([self.params['ox'].value, self.params['oy'].value, self.params['oz'].value, self.params['ow'].value])
                # quattt /= np.linalg.norm(quattt)

                # ids = {(20, 4): (np.array([self.params['x'].value, self.params['y'].value, self.params['z'].value]), quattt)}
                # ids = {(20, 4): (np.array([41.15, 4.2098, 0.0]), np.array([0, 0, 0, 1.0]))}

                # print(ids)
                if ids:
                    done = True
                trials += 1
                self.rate.sleep()
            if done:
                # remove hard-coding
                camera_frame = 'skiros:TransformationPose-49'  # self.params['Camera'].value.getProperty('skiros:FrameId').value
                object_parent_frame = object.getProperty('skiros:BaseFrameId').value

                # test_vis.setProperty('skiros:BaseFrameId', camera_frame)
                # for _, (pos, quat) in ids.items():
                #     set_object_pose(test_vis, list(pos.reshape(-1)), list(quat))
                #     break
                # # self.wmi.update_element(test_vis)
                # self.wmi.update_element_properties(test_vis)

                # for id_dict, (t, q) in ids.items():
                #     marker = markers[id_dict]
                #     marker_frame = marker.getProperty('skiros:BaseFrameId').value

                #     pose = make_pose_stamped(camera_frame, t, q)
                #     marker_estimated_pose = self.buffer.transform(pose, marker_frame, rospy.Duration(1))

                #     print("before", unpack_pose_stamped(marker_estimated_pose))
                #     break

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
                    # print(estimated_rotation_matrix)

                    # Compute transformation between object parent frame and object such that the
                    # set of transformations is consistent
                    estimated_object_rotation_matrix = estimated_rotation_matrix @ marker_rotation_matrix.T
                    estimated_object_position = estimated_position - estimated_object_rotation_matrix @ marker_position

                    estimated_object_quaternion = rot2quat(estimated_object_rotation_matrix)

                    # print(id_dict)
                    # print('pos: ', estimated_object_position)
                    # print('quat:', estimated_orientation)

                    total_position += estimated_object_position
                    total_quaternion += estimated_object_quaternion
                    # break

                total_position /= len(ids)
                total_quaternion /= np.linalg.norm(total_quaternion)

                # object.setProperty('skiros:BaseFrameId', workspace_frame)
                # print(workspace_frame)
                set_object_pose(object, list(total_position), list(total_quaternion))
                self.wmi.update_element_properties(object)

                # for id_dict, (t, q) in ids.items():
                #     marker = markers[id_dict]
                #     marker_frame = marker.getProperty('skiros:BaseFrameId').value
                #     print(marker_frame)

                #     pose = make_pose_stamped(camera_frame, t, q)
                #     marker_estimated_pose = self.buffer.transform(pose, marker_frame, rospy.Duration(1))

                #     print("estimated", unpack_pose_stamped(marker_estimated_pose))
                #     break

                result = "Found AruCo markers with ids: [%s]" % ", ".join([str(id) for id in ids.keys()])
                return self.success(result)
            else:
                return self.fail('Could not find AruCo marker within 5 seconds.', -1)
        else:
            return self.fail('Object does not have markers.', -1)

    def onEnd(self):
        return True


""" class aruco_marker_backwards(PrimitiveBase):
    def __init__(self, *args, **kwargs):
        self.hz = 5
        self.rate = rospy.Rate(self.hz)
        self.sub = RGBListener()
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

        super(aruco_marker_backwards, self).__init__(*args, **kwargs)

    def createDescription(self):
        self.setDescription(ArucoEstimation(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        object = self.params['Object'].value
        test_vis = self.params['TestVis'].value
        relations = object.getRelations()
       
        # ob_pose = make_pose_stamped('map', np.array([0, 0, 0]), np.array([1, 0, 0, 0]))
        # ob_pose = self.buffer.transform(ob_pose, object.getProperty('skiros:BaseFrameId').value, rospy.Duration(1))

        # object.setData(':PoseStampedMsg', ob_pose)

        # ob_pos, ob_quat = unpack_pose_stamped(ob_pose)
        # print(ob_pos, ob_quat)

        aruco_ids = {}
        markers = {}
        for relation in relations:
            if relation['type'] == "skiros:hasA" and "scalable:Marker" in relation['dst']:
                marker = self.wmi.get_element(relation['dst'])
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

        print(aruco_ids)

        if aruco_ids:
            done = False
            trials = 0
            while not done and trials < 5 * self.hz:
                img = self.sub.get()
                ids = aruco_detection(img, (672.043304, 670.234373, 488.053352, 281.019651), (0, 0, 0, 0, 0), aruco_ids)

                # ids = {(20, 4): (np.array([0.0, 0.0, 0.5]), np.array([0, 0, 0, 1]))}
                # ids = {(20, 4): (np.array([41.15, 4.2098, 0.0]), np.array([0, 1, 0, 1]) / np.sqrt(2))}
                # ids = {(20, 4): (np.array([41.15, 4.2098, 0.0]), np.array([0, 0, 0, 1.0]))}

                print(ids)
                if ids:
                    done = True
                trials += 1
                self.rate.sleep()
            if done:
                # remove hard-coding
                camera_frame = 'skiros:TransformationPose-49'  # self.params['Camera'].value.getProperty('skiros:FrameId').value
                workspace_frame = object.getProperty('skiros:BaseFrameId').value
                object_frame = object.getProperty('skiros:FrameId').value


                test_vis.setProperty('skiros:BaseFrameId', camera_frame)
                for _, (pos, quat) in ids.items():
                    set_object_pose(test_vis, list(pos.reshape(-1)), list(quat))
                    break
                # self.wmi.update_element(test_vis)
                self.wmi.update_element_properties(test_vis)

                for id_dict, (t, q) in ids.items():
                    marker = markers[id_dict]
                    marker_frame = marker.getProperty('skiros:BaseFrameId').value

                    pose = make_pose_stamped(camera_frame, t, q)
                    marker_estimated_pose = self.buffer.transform(pose, marker_frame, rospy.Duration(1))

                    print("before", unpack_pose_stamped(marker_estimated_pose))
                    break

                total_position = np.zeros(3, dtype=np.float64)
                total_quaternion = np.zeros(4, dtype=np.float64)
                for id_dict, (position, quaternion) in ids.items():
                    # Get pose of marker in object frame
                    marker = markers[id_dict]
                    marker_position, marker_orientation = get_object_pose(marker)
                    marker_rotation_matrix = quat2rot(marker_orientation)

                    # Reverse dependency of object and marker
                    reverse_rotation_matrix = marker_rotation_matrix.T
                    reverse_position = (-reverse_rotation_matrix @ marker_position)
                    reverse_orientation = rot2quat(reverse_rotation_matrix)

                    marker.setProperty('skiros:BaseFrameId', camera_frame)
                    # print(position)
                    set_object_pose(marker, list(position), list(quaternion))
                    self.wmi.update_element_properties(marker)
                    # print(quaternion)

                    object.setProperty('skiros:BaseFrameId', marker.getProperty('skiros:FrameId').value)
                    # print(reverse_position)
                    set_object_pose(object, list(reverse_position), list(reverse_orientation))
                    self.wmi.update_element_properties(object)
                    # print(reverse_orientation)

                    # Get pose of marker in the parent frame of the object by using the
                    # representation of the pose in the camera frame
                    estimated_pose = make_pose_stamped(object_frame, np.array([0, 0, 0]), np.array([0, 0, 0, 1]))
                    estimated_pose = self.buffer.transform(estimated_pose, workspace_frame, rospy.Duration(1))
                    estimated_position, estimated_quaternion = unpack_pose_stamped(estimated_pose)

                    total_position += estimated_position
                    total_quaternion += estimated_quaternion

                    marker.setProperty('skiros:BaseFrameId', object_frame)
                    set_object_pose(marker, list(marker_position), list(marker_orientation))
                    self.wmi.update_element_properties(marker)
                    break

                # total_position /= len(ids)
                # total_quaternion /= np.linalg.norm(total_quaternion)

                object.setProperty('skiros:BaseFrameId', workspace_frame)
                print(workspace_frame)
                set_object_pose(object, list(total_position), list(total_quaternion))
                self.wmi.update_element_properties(object)

                for id_dict, (t, q) in ids.items():
                    marker = markers[id_dict]
                    marker_frame = marker.getProperty('skiros:BaseFrameId').value
                    print(marker_frame)

                    pose = make_pose_stamped(camera_frame, t, q)
                    marker_estimated_pose = self.buffer.transform(pose, marker_frame, rospy.Duration(1))

                    print("estimated", unpack_pose_stamped(marker_estimated_pose))
                    break

                result = "Found AruCo markers with ids: [%s]" % ", ".join([str(id) for id in ids.keys()])
                return self.success(result)
            else:
                return self.fail('Could not find AruCo marker within 5 seconds.', -1)
        else:
            return self.fail('Object does not have markers.', -1)

    def onEnd(self):
        return True

"""