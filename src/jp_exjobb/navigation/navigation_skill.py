#!/usr/bin/env python3

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
from tf2_geometry_msgs import PoseStamped

import threading

class Navigation:
    def __init__(self):
        self.PENDING = 0
        self.ACTIVE = 1
        self.PREEMPTED = 2
        self.SUCCEEDED = 3
        self.ERROR = 4

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        
        self.status = 0
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.rate = rospy.Rate(1)

        self.min_distance = 5
        self.recovery_pose = None

    def preempt(self):
        self.preempt_navigation = True

    def feedback(self, feedback):
        pose = feedback.base_position.pose

        if (self.recovery_pose is None
            or distance(pose, self.recovery_pose) > self.min_distance ** 2):
            self.recovery_pose = pose

    def navigate_to(self, goal, max_trials, time_between_trials):
        trials = 0

        while trials < max_trials:
            rospy.loginfo('Sending goal')
            self.client.send_goal(goal, feedback_cb=self.feedback)
            
            while self.client.get_state() in (self.ACTIVE, self.PENDING):
                print (self.client.get_state())
                if self.preempt_navigation:
                    self.client.cancel_goal()
                    return True
                self.rate.sleep()

            print(self.client.get_state())

            self.status = self.client.get_state()
            print(self.done)
            if self.status == self.SUCCEEDED or self.status == self.PREEMPTED:
                self.done = True
                return True

            rospy.loginfo('Trying again')

            trials += 1  # trials lol :)
            rospy.sleep(time_between_trials)
        
        return False

    def navigate(self, pose_stamped, max_trials=5, max_recovery=2, time_between_trials=5.0):
        self.done = False
        self.preempt_navigation = False
        target_pose_stamped = self.buffer.transform(pose_stamped, 'map', rospy.Duration(1))
        # target_pose = target_pose_stamped.pose
        goal = MoveBaseGoal()
        recovery_goal = MoveBaseGoal()
        goal.target_pose = target_pose_stamped


        self.client.wait_for_server()

        for _ in range(max_recovery):
            print('entering navto', pose_stamped.pose.position.x)
            status = self.navigate_to(goal, max_trials, time_between_trials)
            print(self.done)
            if self.done:
                break
            
            # Recovery behaviour
            rospy.sleep(time_between_trials)
            
            if self.recovery_pose is not None:
                recovery_pose_stamped = PoseStamped()
                recovery_pose_stamped.header.frame_id = 'map'
                recovery_pose_stamped.header.stamp = rospy.Time.now()
                recovery_pose_stamped.pose = self.recovery_pose

                recovery_goal.target_pose = recovery_pose_stamped
                self.navigate_to(recovery_goal, max_trials, time_between_trials)
        
        return status
        
def distance(pose1, pose2):
    diffX = pose1.position.x - pose2.position.x
    diffY = pose1.position.y - pose2.position.y
    return diffX ** 2 + diffY ** 2

class JPDrive(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('SourceLocation', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('TargetLocation', Element('scalable:Location'), ParamTypes.Required)
        
        self.addPreCondition(self.getRelationCond('HeronAtLocation', 'skiros:at', 'Heron', 'SourceLocation', True))

class jp_drive(PrimitiveBase):
    def createDescription(self):
        self.setDescription(JPDrive(), self.__class__.__name__)

    def onInit(self):
        self.navigator = Navigation()
        return True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def onPreempt(self):
        # preempt navigation class
        self.navigator.preempt()
        self.thread.join()
        return self.fail('Canceled', -1)

    def run(self):
        target = self.params['TargetLocation'].value

        pose = target.getData(':PoseMsg')
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = target.getProperty('skiros:BaseFrameId').value
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = pose

        self.result = self.navigator.navigate(pose_stamped)
        self.done = True

    def execute(self):
        #print('Executing skill')
        if not self.done:
            return self.step('Running...')
        
        self.thread.join()

        if self.result:
            heron = self.params['Heron'].value
            source = self.params['SourceLocation'].value
            target = self.params['TargetLocation'].value
            
            # heron.removeRelation({'src': '-1', 'type': 'skiros:at', 'dst': source.id, 'state': True, 'abstract': False})
            for relation in heron.getRelations(subj='-1', pred='skiros:at'):
                if self.wmi.get_element(relation['dst']).type == 'skiros:Location':
                    heron.removeRelation(relation)
            heron.addRelation('-1', 'skiros:at', target.id)
            self.wmi.update_element(heron)
            return self.success('Done')
        else:
            return self.fail('Failed', -1)

class jp_move_heron(PrimitiveBase):
    def createDescription(self):
        self.setDescription(JPDrive(), self.__class__.__name__)
    
    def execute(self):
        heron = self.params['Heron'].value
        source = self.params['SourceLocation'].value
        target = self.params['TargetLocation'].value
        
        # heron.removeRelation({'src': '-1', 'type': 'skiros:at', 'dst': source.id, 'state': True, 'abstract': False})
        for relation in heron.getRelations(subj='-1', pred='skiros:at'):
            if self.wmi.get_element(relation['dst']).type == 'skiros:Location':
                heron.removeRelation(relation)
        heron.addRelation('-1', 'skiros:at', target.id)
        self.wmi.update_element(heron)
        return self.success(
            'Moved Heron from %s to %s in wm.' % (source.label, target.label)
        )
