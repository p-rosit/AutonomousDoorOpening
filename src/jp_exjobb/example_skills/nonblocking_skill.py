from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import threading

import rospy

class NonBlockingBase(PrimitiveBase):
    """
        SkiROS skill class which runs the skill in a new thread. This means that
        the skiros gui does not get blocked by the skill. The skill should be
        implemented in self.run() and the end messages should be written by
        implementing self.onRunning() and self.onComplete()
    """

    def onStart(self):
        """
            This function starts the thread which actually executes the code
            for the skill. It also resets the flags which keep track of if
            the skill has been preempted or completed.

            This function can be extended but super().onStart() needs to be
            on of the final things in the subclass you implement. Otherwise
            the thread is not started :)
        """
        # Reset relevant flags
        self.complete = False
        self.skill_preempted = False

        # Start new thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        return True

    def execute(self):
        if not self.complete or self.skill_preempted:
            # While the skill is not complete return the running message
            return self.onRunning()
        else:
            # When the skill has finished kill the thread
            self.thread.join()
            # Return a complete message
            return self.onComplete()

    def run(self):
        """
            This function is run in a separate thread to the skiros gui and
            should implement the computation of the skill which takes a
            while to run.
            
            The flag self.complete should be set to True at the end of the
            implementation of the skill. The flag tells this superclass that
            the execution is complete and that the skill can complete.
        """
        self.complete = True
    
    def onRunning(self):
        """
            This function should return

                self.step('MSG')
            
            to ensure that the skill does prematurely tell skiros that it has
            concluded. Some small computations can be done but nothing major
            should be done.
        """
        raise NotImplementedError

    def onComplete(self):
        """
            This function should return either

                self.success('MSG'),

                or 

                self.fail('MSG', code).
            
            No intensive computations should be done in this function
            but small computations can be made. Like checking a flag
            if the skill failed or not.
        """
        raise NotImplementedError

class NonBlockingExample(SkillDescription):
    def createDescription(self):
        self.addParam('Time (s)', 10, ParamTypes.Required)

class non_blocking_example(NonBlockingBase):
    def createDescription(self):
        self.setDescription(NonBlockingExample(), self.__class__.__name__)
    
    def onStart(self):
        # Do relevant pre-computation things here
        self.succeeded = False
        self.ind = 0

        # Super onStart must be called, this starts the thread which runs self.run
        super().onStart()

        return True

    def onPreempt(self):
        # This flag tells the skill that it has been preempted
        self.skill_preempted = True
        return self.fail('Count preempted.', -1)

    def run(self):
        # Do the actual skill here
        while self.ind < self.params['Time (s)'].value and not self.skill_preempted:
            print(self.ind)
            self.ind += 1
            rospy.sleep(1)

        if self.ind < 5:
            self.succeeded = True

        # Super run should be called, this tells the skill the computation is complete
        super().run()

    def onRunning(self):
        # This is called while the skill is not complete
        return self.step('Running counter.')
    
    def onComplete(self):
        # This is called when the skill is complete
        if self.succeeded:
            return self.success('Skill counted.')
        else:
            return self.fail('Count too large.', -1)
