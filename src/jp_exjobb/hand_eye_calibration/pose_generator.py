from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np
from scipy.spatial.transform import Rotation as rot

rng = np.random.default_rng()

class JPGeneratePose(SkillDescription):
    def createDescription(self):
        self.addParam('Base', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Origin', Element('sumo:Object'), ParamTypes.Optional)
        self.addParam('Prev', Element('sumo:Object'), ParamTypes.Optional)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Required)
        self.addParam('Direction Up', [0.0, 0.0, -1.0], ParamTypes.Optional)
        self.addParam('Direction Frame', '', ParamTypes.Optional)
        
        self.addParam('Radius', 0.7, ParamTypes.Required)
        self.addParam('Origin Max Radius', 0.8, ParamTypes.Optional)
        self.addParam('Origin Min Radius', 0.5, ParamTypes.Optional)
        self.addParam('Max Angle (deg)', 50.0, ParamTypes.Required)
        self.addParam('Angle Interval (deg)', 10.0, ParamTypes.Optional)

class jp_generate_pose(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPGeneratePose(), self.__class__.__name__)
    
    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True

    def preStart(self):
        # Unpack variables
        self.base = self.params['Base'].value
        self.origin = self.params['Origin'].value
        self.prev = self.params['Prev'].value
        self.pose = self.params['Pose'].value
        self.direction_up = np.array(self.params['Direction Up'].values, dtype=float)
        self.direction_frame = self.params['Direction Frame'].value
        
        self.rad = self.params['Radius'].value
        self.origin_max_rad = self.params['Origin Max Radius'].value
        self.origin_min_rad = self.params['Origin Min Radius'].value
        self.ang = self.params['Max Angle (deg)'].value
        self.interval = self.params['Angle Interval (deg)'].value

        # Short hand for writing that base frame of direction up is the base frame id
        if self.direction_frame == '':
            self.direction_frame = self.base.getProperty('skiros:FrameId').value

        # Check that base has a pose
        if not self.base.hasData(':Pose'):
            return self.startError('Base does not have a pose.', -1)
        
        # Check if origin exists and if it has a pose
        self.has_origin = self.origin.label != '' and self.origin.id != ''
        if not self.has_origin:
            self.origin_max_rad = np.inf
            self.origin_min_rad = 0

        if self.has_origin and not self.origin.hasData(':Pose'):
            return self.startError('Origin does not have a pose.')

        if self.has_origin:
            origin, _ = self.convert_pose(
                self.origin.getProperty('skiros:BaseFrameId').value,
                self.base.getProperty('skiros:FrameId').value,
                pos=np.array(self.origin.getData(':Position'))
            )

            # Check if any point can be generated (at the moment underestimates correct radius)
            if np.linalg.norm(origin) > self.rad + self.origin_max_rad + 0.1:
                return self.startError('No valid point can be generated, increase some radius.', -1)
        
        # Check if previous pose exists and has a pose
        self.has_prev = self.prev.label != '' and self.prev.id != ''

        if self.has_prev and not self.prev.hasData(':Pose'):
            return self.startError('Previous point does not have a pose.', -1)
        
        return True

    def run(self):
        # Prepare variables
        self.ang = self.ang if 0.0 <= self.ang <= 180.0 else 180.0
        self.interval = self.interval if 0.0 <= self.interval <= 180 else 0.0
        self.ang *= np.pi / 180.0
        self.interval *= np.pi / 180.0
        self.direction_up /= np.linalg.norm(self.direction_up)

        # Compute the radius within which newly generate points should not exist
        if self.ang < np.pi / 2:
            self.prev_rad = self.rad * np.sin(self.ang) / 3
        else:
            self.prev_rad = self.rad / 3

        # Extract the relevant positions
        self.base_pos = np.array(self.base.getData(':Position'))
        if self.has_origin:
            self.origin_pos = np.array(self.origin.getData(':Position'))
        if self.has_prev:
            self.prev_pos = np.array(self.prev.getData(':Position'))

        # Generate the random pose
        self.pos = self.generate_position()
        self.quat = self.generate_orientation()

        # Convert the random pose to the right frame
        self.pos, self.quat = self.convert_pose(
            self.base.getProperty('skiros:FrameId').value,
            self.pose.getProperty('skiros:BaseFrameId').value,
            pos=self.pos,
            quat=self.quat
        )

        # Write the generated pose to the transformation pose
        self.pose.setData(':Position', self.pos)
        self.pose.setData(':Orientation', self.quat)
        self.wmi.update_element_properties(self.pose)

        return self.success('Pose generated.')

    def generate_position(self):
        """
        A position is generated which has a maximal angle to the chosen 'up' direction.
        We also take care to stay close enough to the origin, if one is specified, and
        not to generate a pose which is too close the the most recent one, if one has
        been specified.
        """
        if self.has_origin:
            origin, _ = self.convert_pose(
                self.origin.getProperty('skiros:BaseFrameId').value,
                self.base.getProperty('skiros:FrameId').value,
                pos=self.origin_pos
            )
        if self.has_prev:
            prev, _ = self.convert_pose(
                self.prev.getProperty('skiros:BaseFrameId').value,
                self.base.getProperty('skiros:FrameId').value,
                pos=self.prev_pos
            )
        direction_up, _ = self.convert_pose(
            self.direction_frame,
            self.base.getProperty('skiros:FrameId').value,
            pos=self.direction_up
        )

        pos = np.full(3, np.inf)
        angle = np.inf
        orig_mag = 0
        prev_mag = np.inf

        while (
            angle > self.ang or
            self.origin_min_rad**2 > orig_mag or
            orig_mag > self.origin_max_rad**2 or
            prev_mag < self.prev_rad**2
        ):
            pos = rng.normal(size=3)
            pos *= self.rad / np.linalg.norm(pos)

            up_mag = np.dot(direction_up, pos)
            perp_mag = np.linalg.norm(
                pos - up_mag * direction_up
            )

            angle = np.arctan2(perp_mag, up_mag)
            if self.has_origin:
                orig_mag = np.square(pos - origin).sum()
            if self.has_prev:
                prev_mag = np.square(pos - prev).sum()

        return pos

    def generate_orientation(self):
        """
        An orientation is generated given the generated position. The orientation is
        dependent on how the arm would have to move to be pointed towards the target,
        if there is a camera on the gripper it is assumed to be on the top of the gripper
        although in principle it doesn't really matter.
        """
        pos, _ = self.convert_pose(
            self.base.getProperty('skiros:FrameId').value,
            'map',
            pos=self.pos
        )
        base, _ = self.convert_pose(
            self.base.getProperty('skiros:BaseFrameId').value,
            'map',
            pos=self.base_pos
        )
        if self.has_origin:
            origin, _ = self.convert_pose(
                self.origin.getProperty('skiros:BaseFrameId').value,
                'map',
                pos=self.origin_pos
            )

        axis_z = base - pos
        axis_z /= np.linalg.norm(axis_z)

        axis_x = np.array([0.0, 0.0, -1.0])
        axis_x -= -axis_z[2] * axis_z
        axis_x /= np.linalg.norm(axis_x)

        axis_y = np.cross(axis_z, axis_x)
        axis_y /= np.linalg.norm(axis_y)

        axis_x = axis_x.reshape((-1, 1))
        axis_y = axis_y.reshape((-1, 1))
        axis_z = axis_z.reshape((-1, 1))
        r = np.concatenate((axis_x, axis_y, axis_z), axis=1)

        if self.has_origin:
            pos_vec = pos[:2] - base[:2]
            orig_vec = origin[:2] - base[:2]
            ang = np.arccos(
                np.dot(pos_vec, orig_vec) / (
                    np.linalg.norm(pos_vec) *
                    np.linalg.norm(orig_vec)
                )
            )

            orthogonal_vec = np.array([
                -orig_vec[1],
                orig_vec[0]
            ])

            if np.dot(orthogonal_vec, pos_vec) < 0:
                ang = -ang

            ang += rng.uniform(-self.interval, self.interval)
            dr = np.array([
                [np.cos(ang), -np.sin(ang), 0],
                [np.sin(ang),  np.cos(ang), 0],
                [          0,            0, 1]
            ])

            r = r @ dr

        quat = rot.from_matrix(r).as_quat()

        _, quat = self.convert_pose(
            'map',
            self.base.getProperty('skiros:FrameId').value,
            quat=quat
        )

        return quat

    def convert_pose(self, from_frame, to_frame, pos=None, quat=None):
        """
        Convert a position and an orientation from some frame to a target frame
        """
        pose = PoseStamped()
        pose.header.frame_id = from_frame
        pose.header.seq = rospy.Time(0)

        if pos is not None:
            pos = pos.copy()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]

        if quat is not None:
            quat = quat.copy()
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

        pose = self.buffer.transform(pose, to_frame, rospy.Duration(1))
        
        if pos is not None:
            pos[0] = pose.pose.position.x
            pos[1] = pose.pose.position.y
            pos[2] = pose.pose.position.z

        if quat is not None:
            quat[0] = pose.pose.orientation.x
            quat[1] = pose.pose.orientation.y
            quat[2] = pose.pose.orientation.z
            quat[3] = pose.pose.orientation.w

        return pos, quat