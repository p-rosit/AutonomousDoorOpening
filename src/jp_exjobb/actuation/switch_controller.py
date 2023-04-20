from skiros2_skill.core.skill import (SkillDescription,
                                      ParamOptions,
                                      SkillBase,
                                      Sequential)
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import controller_manager_msgs.srv
import rospy

class JPSwitchController(SkillDescription):   
    def createDescription(self):
        # =======Params=========
        self.addParam("Arm", Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam("Controller", Element('scalable:ControllerState'), ParamTypes.Required)

class jp_switch_controller(PrimitiveBase):
    def createDescription(self):
        self.setDescription(JPSwitchController(), self.__class__.__name__)

    def onStart(self):
        ns = ""
        self.controllers = {}
        # Fetch information from the world model
        if self.params["Arm"].value.hasProperty("skiros:JointConfigurationController"):
            self.controllers["joint_config"] = self.params["Arm"].value.getProperty("skiros:JointConfigurationController").value
        if self.params["Arm"].value.hasProperty("skiros:CompliantController"):
            self.controllers["compliant"] = self.params["Arm"].value.getProperty("skiros:CompliantController").value
        if len(self.controllers) == 0:
            rospy.logwarn("No controllers specified for this arm. Switching will not work.")
        if self.params["Arm"].value.hasProperty("scalable:ArmId"):
            ns = self.params["Arm"].value.getProperty("scalable:ArmId").value

        ns = ""
        # Connect
        self.c_manager_l = rospy.ServiceProxy("/" + ns + '/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        self.c_manager_s = rospy.ServiceProxy("/" + ns + '/controller_manager/switch_controller', controller_manager_msgs.srv.SwitchController)
        return True

    def execute(self):
        controller = self.params['Controller'].value
        controller_type = controller.getProperty('skiros:Value').value
        if controller_type not in self.controllers:
            return self.fail("Unregistered controller type: {}. Known types are: {}".format(controller_type, list(self.controllers.keys())), -1)

        # Check which one is running
        c_list = self.c_manager_l(controller_manager_msgs.srv.ListControllersRequest())
        c_states = {}
        for c in c_list.controller:
            if c.name in self.controllers.values():
                c_states[c.name] = c.state
        if self.controllers[controller_type] not in c_states:
            return self.fail("Desired controller '{}' not known to controller manager".format(self.controllers[controller_type]), -2)
        if c_states[self.controllers[controller_type]] == "running":
            return self.success("'{}' controller '{}' is already running.".format(controller_type, self.controllers[controller_type]))

        # Switch controllers
        req = controller_manager_msgs.srv.SwitchControllerRequest()
        req.start_controllers.append(self.controllers[controller_type])
        if "running" in c_states.values():
            running_c = list(c_states.keys())[list(c_states.values()).index("running")]
            req.stop_controllers.append(running_c)
            rospy.logwarn("No controller to stop when activating '{}' with '{}'. Is the robot running?".format(controller_type, self.controllers[controller_type]))
        req.strictness = 2  # Strict
        req.timeout = 1.    # Seconds 

        if self.c_manager_s(req):
            return self.success("Controller changed to '{}' with '{}'".format(controller_type, self.controllers[controller_type]))
        else:
            return self.fail("Failed switching controllers.", -3)