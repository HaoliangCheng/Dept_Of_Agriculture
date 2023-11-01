import schedule as sched

class BehavioralLayer:

    def __init__(self, sensors, actuators, behaviors):
        self.behaviors = behaviors
        for behavior in behaviors:
            behavior.setSensors(sensors)
            behavior.setActuators(actuators)
        self.enabled = []
        # Initialize any extra variables here

    def getBehavior(self, name):
        for b in self.behaviors:
            if (b.name == name): return b
        return None

    def isEnabled(self, behavior):
        return behavior in self.enabled

    def startBehavior(self,name):
        # BEGIN STUDENT CODE
        behavior=self.getBehavior(name)
        if(behavior is not None and not self.isEnabled(behavior)):
            behavior.start()
            self.enabled.append(behavior)
        # END STUDENT CODE
        pass

    def pauseBehavior(self,name):
        # BEGIN STUDENT CODE
        behavior=self.getBehavior(name)
        if(behavior is not None and self.isEnabled(behavior)):
            behavior.pause()
            self.enabled.remove(behavior)
        # END STUDENT CODE
        pass

    def doStep(self):
        for behavior in self.enabled:
            behavior.doStep()

    def startAll(self):
        for behavior in self.behaviors:
            self.startBehavior(behavior.name)

    #more functions? write them here!

class ExecutiveLayer:

    def __init__(self):
        self.schedule = {}
        self.laststep = -1
        self.monitors = []
        # Initialize any extra variables here

    def setPlanningLayer(self, planning):
        self.planning = planning

    def setBehavioralLayer(self, behavioral):
        self.behavioral = behavioral

    def setSchedule(self, schedule):
        self.schedule = schedule

    def requestNewSchedule(self):
        self.planning.requestNewSchedule()

    def getMonitor(self, name):
        for m in self.monitors:
            if (m.name == name): return m
        return None

    def setMonitors(self, sensors, actuator_state, monitorsList):
        self.monitors = monitorsList
        now = sensors.getTime()
        for monitor in self.monitors:
            monitor.setSensors(sensors)
            monitor.setActuatorState(actuator_state)
            monitor.setExecutive(self)
            monitor.last_time = now
            monitor.dt = 0
            monitor.activate()

    def doStep(self, t): #t time in seconds since midnight
        # NOTE: Disable any behaviors that need to be disabled
        #   before enabling any new behaviors
        # BEGIN STUDENT CODE
        #for behavior_name in self.schedule:
        #   for runing_time in self.schedule[behavior_name]:
        #       # Check if the current time is within the specified time interval
        #       if runing_time[0] <= t/60 < runing_time[1]:
        #           # Start the behavior if it's not running
        #           if behavior_name not in self.behaviors:
        #               self.behaviors.append(behavior_name)
        #               self.start.append(behavior_name)
        #       else:
        #           # Stop the behavior if it's running
        #           if behavior_name in self.behaviors:
        #               self.behaviors.remove(behavior_name)
        #               self.pause.append(behavior_name)
                        
        #for behavior_name in self.pause:
        #    behavior=self.behavioral.getBehavior(behavior_name)
        #    behavior.pause()
        #self.pause.clear()
        #for behavior_name in self.start:
        #    behavior=self.behavioral.getBehavior(behavior_name)
        #    behavior.start()
        #self.start.clear()
        for behavior_name in self.schedule:
            pause = True
            for (start, end) in self.schedule[behavior_name]:
                if start <= t/60 < end:
                    pause = False
            if pause:
            	self.behavioral.pauseBehavior(behavior_name)
              
        for behavior_name in self.schedule:
            for (start, end) in self.schedule[behavior_name]:
                if start <= t/60 < end:
                    self.behavioral.startBehavior(behavior_name)
        # END STUDENT CODE
        for monitor in self.monitors:
            monitor.doMonitor()


class PlanningLayer:

    def __init__(self, schedulefile):
        self.schedulefile = schedulefile
        self.usetestfile = False
        self.schedulerequested = True
        self.schedule = {}
        self.laststep = 0

    def setTestingSchedule(self, testschedule):
        self.testschedule = testschedule

    def setExecutive(self, executive):
        self.executive = executive

    def switch_to_test_sched(self):
        self.usetestfile = True
        self.requestNewSchedule()

    def getNewSchedule(self):
        if self.usetestfile:
            self.schedule = self.scheduleFromFile(self.testschedule)
        else:
            self.schedule = self.scheduleFromFile(self.schedulefile)
        self.executive.setSchedule(self.schedule)
        self.schedulerequested = False

    def requestNewSchedule(self):
        self.schedulerequested = True

    def doStep(self, t):
        if self.schedulerequested or self.checkEnded(t):
            self.getNewSchedule()
        self.laststep = (t//60)%(24*60)

    def checkEnded(self, t):
        mins = (t//60)%(24*60)
        if mins < self.laststep: #looped back around
            return True
        return False

    def scheduleFromFile(self, schedulefile):
        schedule = sched.readSchedule(schedulefile)
        return schedule
