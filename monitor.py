
'''
Defines a general monitor that tracks some aspect(s) of the system/environment
  and takes some action(s), when necessary.
Each monitor must implement:
     perceive - to take in sensor data and time and output percepts
     monitor - to take in the percepts
Each monitor performs one perceive, monitor sequence each time it is invoked.
Note that the period at which a monitor executes can be varied.  The
  variable self.last_time is the last (unix) time the monitor was invoked, and
  self.dt is the amount of time that has passed since it was last invoked
'''
class Monitor(object):
    dt = 0
    last_time = 0

    def __init__(self, name, period=1):
        self.name = name
        self.period = period

    def setSensors(self, sensors):
        self.sensors = sensors

    def setActuatorState(self, actuator_state):
        self.actuator_state = actuator_state

    def setExecutive(self, executive):
        self.executive = executive

    def activate(self):
        pass

    def perceive(self):
        pass

    def monitor(self):
        pass

    def doMonitor(self):
        now = self.sensors.getTime()
        dt = now - self.last_time
        if (dt >= self.period):
            self.dt = dt
            self.sensordata = self.sensors.doSense()
            self.perceive()
            self.monitor()
            self.last_time = now
