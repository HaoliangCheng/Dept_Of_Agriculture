'''
Defines a general behavior.
Each behavior needs to be able to take in sensor and actuators
Each behavior must implement:
     perceive - to take in sensor data and time and output percepts
     plan - to take in percepts, determine new state
     act - to take in the state and output actions for each actuator
     start - to start up after running
     pause - to shut down before stopping
Each behavior performs one perceive, plan, act loop and returns the desired actions
doStep sends commands to actuators
'''
from std_msgs.msg import String
import rospy

class Behavior(object):
    enablePub = None
    disablePub = None

    def __init__(self, name):
        self.name = name
        if not Behavior.enablePub:
            Behavior.enablePub = rospy.Publisher('enable', String,
                                                 latch=True, queue_size=10)
            Behavior.disablePub = rospy.Publisher('disable', String,
                                                  latch=True, queue_size=10)

    def setSensors(self, sensors):
        self.sensors = sensors

    def setActuators(self, actuators):
        self.actuators = actuators

    def start(self):
        self.sensordata = self.sensors.doSense()
        self.perceive()
        self.enable()

    def pause(self):
        self.disable()

    def enable(self):
        print("Enable: %s" %self.name)
        # Let the world know this behavior has begun
        try: Behavior.enablePub.publish(self.name)
        except: pass
        # Use 'enable' trigger to transition the FSM out of the 'initial' state
        self.trigger("enable")

    def disable(self):
        print("Disable: %s" %self.name)
        # Let the world know this behavior has stopped
        try: Behavior.disablePub.publish(self.name)
        except: pass
        # Use 'diable' trigger to transition the FSM into the 'initial' state
        self.trigger("disable") 

    def perceive(self):
        pass

    def act(self):
        pass

    def doStep(self):
        self.sensordata = self.sensors.doSense()
        self.perceive()
        self.act()
