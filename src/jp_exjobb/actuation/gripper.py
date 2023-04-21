from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from wsg_50_common.msg import Status
from wsg_50_common.srv import Move, MoveRequest
import skiros2_common.tools.logger as log

import rospy


class JPGripper(SkillDescription):
    def createDescription(self):
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Open', False, ParamTypes.Required)


class jp_gripper(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPGripper(), self.__class__.__name__)
    
    def onInit(self):
        return True
    
    def preStart(self):
        driver_address = self.params['Gripper'].value.getProperty('skiros:DriverAddress').value
        self.grasp_srv = rospy.ServiceProxy(driver_address+'/grasp', Move)
        self.release_srv = rospy.ServiceProxy(driver_address+'/release', Move)

        return True
    
    def _call(self, service, msg):
        try:
            resp1 = service(msg)
            if resp1.error == 0:
                return True, 'Changed gripper state'
            return False, "Error {}".format(resp1.error)
        except rospy.ServiceException as e:
            log.error("[actuate_wsg_gripper]", "Service call failed: %s"%e)
            return (False, "Service call failed: %s"%e)

    def run(self):
        if self.params["Open"].value:
            self._resp = self._call(self.release_srv, MoveRequest(100.0, 400.0))
        else:
            self._resp = self._call(self.grasp_srv, MoveRequest(0.0, 400.0))
        
        skill_succeded, msg = self._resp

        if skill_succeded:
            return self.success(msg)
        else:
            return self.fail(msg, -1)
        
