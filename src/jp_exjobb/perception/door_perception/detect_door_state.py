from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy

from jp_exjobb.perception.door_perception.door_state_classification import DoorStateClassifier

class DetectDoorState(SkillDescription):
    def createDescription(self):
        self.addParam('Region Transition', Element('scalable:RegionTransition'), ParamTypes.Required)
        self.addParam('Region Bounding Box', Element('scalable:RegionBB'), ParamTypes.Inferred)
        self.addParam('Time Limit (s)', 3600.0, ParamTypes.Required)
        self.addParam('Door Open Threshold', 0.3, ParamTypes.Required)
        self.addParam('Door Closed Threshold', 0.8, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('DoorHasBB', 'skiros:hasA', 'Region Transition', 'Region Bounding Box', True))

class detect_door_state(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(DetectDoorState(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        self.running = False
        return True

    def preStart(self):
        door_bb = self.params['Region Bounding Box'].value
        open_threshold = self.params['Door Open Threshold'].value
        closed_threshold = self.params['Door Closed Threshold'].value
        self.door_state = DoorStateClassifier(door_bb, open_threshold, closed_threshold)

        self.preempted = False
        return True

    def onPreempt(self):
        self.preempted = True
        return self.fail('Watching preempted.', -1)

    def run(self):
        time_limit = self.params['Time Limit (s)'].value

        ind = 0
        while ind < self.hz * time_limit and not self.preempted:
            self.door_state_known, self.door_open = self.door_state.get_door_state()
            self.door_fill = self.door_state.get_filled_value()

            if self.door_state_known:
                if self.door_open:
                    self.status = 'Door open %.2f.' % self.door_fill
                else:
                    self.status = 'Door closed %.2f.' % self.door_fill
            else:
                self.status = 'Door sexuality unknown, %.2f.' % self.door_fill
            ind +=1
            self.rate.sleep()
        
        return self.success('Door peeping concluded.')

    def onEnd(self):
        self.door_state.unregister()
        self.door_state = None
        return True

# class watchwatchwatch(PrimitiveThreadBase):
#     def createDescription(self):
#         self.setDescription(DetectDoorState(), self.__class__.__name__)

#     def onInit(self):
#         self.f_sub = rospy.Subscriber('/f_scan', LaserScan, callback=self.f_scan)
#         self.b_sub = rospy.Subscriber('/b_scan', LaserScan, callback=self.b_scan)

#         self.buffer = tf2_ros.Buffer()  # type: any
#         self.tf_listener = tf2_ros.TransformListener(self.buffer)

#         return True

#     def preStart(self):
#         self.f = None
#         self.b = None
#         return True

#     def f_scan(self, msg):
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         lengths = np.array(msg.ranges)
#         angles = np.linspace(angle_min, angle_max, num=lengths.shape[0])
#         x = lengths * np.cos(angles)
#         y = lengths * np.sin(angles)

#         frame = msg.header.frame_id
        
#         for i in range(x.shape[0]):
#             x[i], y[i] = self.transform(frame, x[i], y[i])

#         self.f = (x, y)

#     def b_scan(self, msg):
#         angle_min = msg.angle_min
#         angle_max = msg.angle_max
#         lengths = np.array(msg.ranges)
#         angles = np.linspace(angle_min, angle_max, num=lengths.shape[0])
#         x = lengths * np.cos(angles)
#         y = lengths * np.sin(angles)

#         frame = msg.header.frame_id
        
#         for i in range(x.shape[0]):
#             x[i], y[i] = self.transform(frame, x[i], y[i])

#         self.b = (x, y)
    
#     def transform(self, frame, x, y):
#         pose = PoseStamped()
#         pose.header.frame_id = frame
#         pose.header.stamp = rospy.Time(0)
#         pose.pose.position.x = x
#         pose.pose.position.y = y
        
#         pose = self.buffer.transform(pose, 'map', rospy.Duration(1))
        
#         x = pose.pose.position.x
#         y = pose.pose.position.y
#         return x,y
                

#     def run(self):
#         while self.f is None or self.b is None:
#             rospy.sleep(0.5)

#         f_x, f_y = self.f
#         b_x, b_y = self.b

#         plt.cla()

#         plt.axis('equal')

#         plt.plot(f_x, f_y, '*')
#         plt.plot(b_x, b_y, '*')
#         plt.xlim(35, 50)
#         plt.ylim(0, 10)
        
#         plt.savefig('/home/duploproject/penis.png')
        
#         return self.success(':)')
