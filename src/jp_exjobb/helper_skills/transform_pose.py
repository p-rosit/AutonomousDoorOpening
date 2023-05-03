from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
import tf2_ros

class TransformPose(SkillDescription):
    def createDescription(self):
        self.addParam('Thing', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('To Frame', '', ParamTypes.Required)

class transform_pose(PrimitiveBase):
    def createDescription(self):
        self.setDescription(TransformPose(), self.__class__.__name__)
    
    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True

    def execute(self):
        thing = self.params['Thing'].value
        frame = self.params['To Frame'].value

        pose = thing.getData(':PoseStampedMsg')

        new_pose = self.buffer.transform(pose, frame, rospy.Duration(1))

        msg = "".join([
            'Position:\n',
            '\tx: %.16f\n' % new_pose.pose.position.x,
            '\ty: %.16f\n' % new_pose.pose.position.y,
            '\tz: %.16f\n' % new_pose.pose.position.z,
            'Orientation:\n',
            '\tw: %.16f\n' % new_pose.pose.orientation.w,
            '\tx: %.16f\n' % new_pose.pose.orientation.x,
            '\ty: %.16f\n' % new_pose.pose.orientation.y,
            '\tz: %.16f' % new_pose.pose.orientation.z
        ])
        print("%s transformed to %s:" % (thing.label, frame))
        print(msg)

        return self.success(msg)
