from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive_thread import PrimitiveThreadBase

import rospy
from geometry_msgs.msg import WrenchStamped

import numpy as np
import matplotlib.pyplot as plt

class ListenToWrench(SkillDescription):
    def createDescription(self):
        self.addParam('Time (s)', 5.0, ParamTypes.Required)
        self.addParam('Name', '', ParamTypes.Required)

class listen_to_wrench(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ListenToWrench(), self.__class__.__name__)

    def onInit(self):
        self.running = False
        self.sub = rospy.Subscriber('/wrench', WrenchStamped, callback=self.listen)
        self.rate = rospy.Rate(10)

    def preStart(self):
        self.running = True
        self.preempt_requested = False
        self.force = []
        self.force_coords = ([], [], [])
        self.torque = []
        self.torque_coords = ([], [], [])
        self.status = 'Listening to "/wrench" topic.'
        return True
    
    def onPreempt(self):
        self.preempt_requested = True
        return self.fail('Listening preempted.', -1)

    def run(self):
        time_limit = rospy.Duration(self.params['Time (s)'].value)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < time_limit:
            if self.preempt_requested:
                return False, ''
            self.rate.sleep()
        
        self.running = False

        force_mag = np.array(self.force)
        torque_mag = np.array(self.torque)

        plt.plot(force_mag)
        plt.plot(torque_mag)
        plt.legend(['Force', 'Torque'])
        plt.savefig('/home/duploproject/' + self.params['Name'].value + '_wrench.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        axs[0].plot(self.force_coords[0])
        axs[0].plot(self.force_coords[1])
        axs[0].plot(self.force_coords[2])
        axs[0].legend(['force x', 'force y', 'force z'])
        axs[1].plot(self.torque_coords[0])
        axs[1].plot(self.torque_coords[1])
        axs[1].plot(self.torque_coords[2])
        axs[1].legend(['torque x', 'torque y', 'torque z'])
        plt.savefig('/home/duploproject/' + self.params['Name'].value + '_coords.png')
        plt.cla()

        return True, 'Done listening to "/wrench" topic.'

    def listen(self, msg):
        if self.running:
            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z

            force_mag = np.sqrt(fx ** 2 + fy ** 2 + fz ** 2)
            torque_mag = np.sqrt(tx ** 2 + ty ** 2 + tz ** 2)

            self.force.append(force_mag)
            self.torque.append(torque_mag)

            [force.append(coord) for force, coord in zip(self.force_coords, [fx, fy, fz])]
            [torque.append(coord) for torque, coord in zip(self.torque_coords, [tx, ty, tz])]
