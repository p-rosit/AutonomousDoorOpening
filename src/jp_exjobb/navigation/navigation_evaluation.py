from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, Loop
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase

import rospy
from std_msgs.msg import Empty

import numpy as np


class GoToPosition(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Object', Element('skiros:Product'), ParamTypes.Required)
        self.addParam('InitialLocation', Element('scalable:Workstation'), ParamTypes.Required)
        self.addParam('TargetLocation', Element('scalable:Workstation'), ParamTypes.Required)

class loop_heron(SkillBase):
    def createDescription(self):
        self.setDescription(GoToPosition(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill(Loop(end=20))(
                self.skill('JPDrive', 'jp_drive', remap={'TargetLocation': 'TargetLocation'}),
                self.skill('DetectAndSave', 'detect_and_save', remap={'Object': 'Object', 'Camera': 'Camera'}),
                self.skill('CalculateNavigation', 'calculate_mean'),
                self.skill('JPDrive', 'jp_drive', remap={'TargetLocation': 'InitialLocation'})
            ),
            self.skill('CalculateNavigation', 'evaluate_navigation')
        )

class DetectAndSave(SkillDescription):
    
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Object', Element('skiros:Product'), ParamTypes.Required)
    

class detect_and_save(SkillBase):

    def createDescription(self):
        self.setDescription(DetectAndSave(), self.__class__.__name__)
    
    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill('ArucoEstimation', 'aruco_marker', remap={'Object': 'Object', 'Camera': 'Camera'}),
            self.skill('SaveCoordinates', 'save_coordinates', remap={'Object': 'Object'}),
        )

class SaveCoordinates(SkillDescription):
    def createDescription(self):
        self.addParam('Object', Element('skiros:Product'), ParamTypes.Required)

class save_coordinates(PrimitiveBase):

    def createDescription(self):
        self.setDescription(SaveCoordinates(), self.__class__.__name__)

    def onInit(self):
        self.mean_sub = rospy.Subscriber('/NavigationEvaluation/mean', Empty, callback=self.mean_callback)
        self.eval_sub = rospy.Subscriber('/NavigationEvaluation/evaluate', Empty, callback=self.eval_callback)
        self.currr = []
        self.badname = []
        return True
    
    def mean_callback(self, msg):
        #nparray in tuple in list
        plist = np.zeros(3)
        qlist = np.zeros(4)
        for p, q in self.currr:
            plist += p
            if q[3] < 0:
                q *= -1
            qlist += q
        
        plist /= len(self.currr)
        qlist /= np.linalg.norm(qlist)

        self.badname.append((plist, qlist)) 

    def eval_callback(self, msg):
        # zip me upp bby
        # ps i love you
        # daddy cool
        ps, qs = list(zip(*self.badname))
        # pray the gay away(conversion therapy)
        ps = np.array(ps)
        qs = np.array(qs)
        # np.sexually_transmitted_disease
        penis_std = ps.std(axis=0)
        qvinna_std = qs.std(axis=0)
        
        print(np.linalg.norm(penis_std))
        print(np.linalg.norm(qvinna_std))

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        obj = self.params['Object'].value
        pose = obj.getData(':PoseMsg')
        p = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.currr.append((p, q))
        return self.success('Penis')

    def onEnd(self):
        return True

class CalculateNavigation(SkillDescription):
    def createDescription(self):
        pass

class calculate_mean(PrimitiveBase):

    def createDescription(self):
        self.setDescription(CalculateNavigation(), self.__class__.__name__)

    def onInit(self):
        self.pub = rospy.Publisher('/NavigationEvaluation/mean', Empty, queue_size=1)
        return True

    def execute(self):
        self.pub.publish(Empty())
        return self.success('in')

class evaluate_navigation(PrimitiveBase):

    def createDescription(self):
        self.setDescription(CalculateNavigation(), self.__class__.__name__)
    
    def onInit(self):
        self.pub = rospy.Publisher('/NavigationEvaluation/evaluate', Empty, queue_size=1)

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        self.pub.publish(Empty())
        return self.success('Vagina')

    def onEnd(self):
        return True
