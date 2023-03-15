from skiros2_common.core.primitive import PrimitiveBase
import threading

class NonBlockingBase(PrimitiveBase):
    """
    SkiROS skill class which runs the skill in a new thread. This means that
    the skiros gui does not get blocked by the skill. The skill should be
    implemented in self.run() and the end messages should be written by
    implementing self.onRunning() and self.onComplete()
    """

    def onPreStart(self):
        """
        This function runs before the thread is started and should do any
        precomputation things that you need to do before starting the skill.
        As long as these pre-computations do not take too long, then they
        should happen in self.run which is in a new thread.
        """
        raise NotImplementedError

    def onStart(self):
        """
        This function starts the thread which actually executes the code
        for the skill. It also resets the flags which keep track of if
        the skill has been preempted or completed.
        """
        # Perform any precomputations
        self.onPreStart()

        # Reset relevant flags
        self.complete = False
        self.skill_preempted = False

        # Start new thread
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        return True

    def execute(self):
        """
        Here we regularly check if the separate thread is done yet. You
        should NOT extend this function in the subclass.
        """
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

        Flags can be checked to return different running messages, for example.
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
