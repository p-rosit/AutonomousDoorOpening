from skiros2_common.core.primitive_thread import PrimitiveThreadBase

import rospy

from jp_exjobb.perception.door_perception.detect_door_state import DetectDoorState
from jp_exjobb.perception.door_perception.door_state_classification import DoorStateClassifier

class watch_for_door(PrimitiveThreadBase):
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
        count = 0
        amount = self.hz * self.params['Time'].value
        time_limit = self.params['Time Limit (s)'].value

        ind = 0
        while ind < self.hz * time_limit and not self.preempted:
            self.door_state_known, self.door_open = self.door_state.get_door_state()
            self.door_fill = self.door_state.get_filled_value()

            if self.door_state_known:
                if self.door_open:
                    count = 0
                    self.status = 'Door open %.2f.' % self.door_fill
                else:
                    count += 1
            else:
                self.status = 'Door sexuality unknown, %.2f.' % self.door_fill
            
            if amount < count:
                return self.fail('Door closed.', -1)
        
            ind +=1
            self.rate.sleep()

        return self.success('Door peeping concluded, door did not close.')

    def onEnd(self):
        self.door_state.unregister()
        self.door_state = None
        return True