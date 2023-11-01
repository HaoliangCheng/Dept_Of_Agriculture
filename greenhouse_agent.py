import rospy, ros_hardware, layers
import sys, os, select, time
from terrabot_utils import time_since_midnight

import greenhouse_behaviors as gb
import ping_behavior as ping
import camera_behavior 
import email_behavior as em
##import light_monitor as lm
import logging_monitor

def init_ros(sim, name):
    if sim: rospy.set_param('use_sim_time', True)
    rospy.init_node(name, anonymous = True)
    # Wait for clock to start up correctly
    while rospy.get_time() == 0: rospy.sleep(0.1)

def check_for_input():
    if sys.stdin in select.select([sys.stdin],[],[],0)[0]:
        input = sys.stdin.readline()
        if input[0] == 'q':
            quit()
        else:
            print("Usage: q (quit)")

# Often the agent starts running before sensor data is received
# This waits for some data to have been received, assuming the real
# sensor values are not zero
def wait_for_sensors(sensors):
    while sensors.weight == 0 or sensors.moisture == 0:
        #print("Wait")
        rospy.sleep(.25)

class BehavioralGreenhouseAgent:

    def __init__(self, sim):
        init_ros(sim, 'greenhouseagent_behavioral')

        # Initialize ROSSensors, ROSActuators, and behaviors,
        #  save each of them as instance variables, and pass them all
        #  to instantiate a BehavioralLayer
        self.sensors = ros_hardware.ROSSensors()
        # BEGIN STUDENT CODE
        self.actuators = ros_hardware.ROSActuators() 

        self.Light = gb.Light()
        self.RaiseTemp = gb.RaiseTemp()
        self.LowerTemp =gb.LowerTemp()
        self.LowerHumid = gb.LowerHumid()
        self.RaiseSMoist =gb.RaiseSMoist()
        self.LowerSMoist =gb.LowerSMoist()
        self.Ping =ping.Ping()
        self.behaviors=[self.Light,self.RaiseTemp,self.LowerTemp,self.LowerHumid,self.RaiseSMoist,self.LowerSMoist,self.Ping]

        self.behavioral=layers.BehavioralLayer(self.sensors,self.actuators,self.behaviors)
        
        # END STUDENT CODE

    def main(self):
        wait_for_sensors(self.sensors)
        
        
        self.behavioral.startAll()
        while not rospy.core.is_shutdown():
            # Run a step of the behavioral architecture
            self.behavioral.doStep()
            rospy.sleep(1)
            check_for_input()

class LayeredGreenhouseAgent:

    def __init__(self,sim,schedulefile):
        init_ros(sim, 'greenhouseagent_layered')
        # Initialize the architecture:
        # As with the behavioral agent, initialize ROSSensors, ROSActuators,
        #  and all behaviors, save each of them as instance variables, and
        #  pass them all to instantiate a BehavioralLayer.
        # In addition, create executive and planning layers, and set the
        #  connections between the layers, using the appropriate functions
        #  defined in the executive and planning classes.  In particular,
        #  connect the behavioral and planning layers to the executive, and
        #  the executive to the planning layer.
        # Don't forget to have the planning layer invoke getNewSchedule
        
        self.sensors = ros_hardware.ROSSensors()
        # BEGIN STUDENT CODE
        self.actuators = ros_hardware.ROSActuators() 
        self.Light = gb.Light()
        self.RaiseTemp = gb.RaiseTemp()
        self.LowerTemp =gb.LowerTemp()
        self.LowerHumid = gb.LowerHumid()
        self.RaiseSMoist =gb.RaiseSMoist()
        self.LowerSMoist =gb.LowerSMoist()
        self.Ping =ping.Ping()
        self.Camera=camera_behavior.TakeImage()
        self.Email=em.Email(self.Camera, self.RaiseSMoist)
        
        self.behaviors=[self.Light,self.RaiseTemp,self.LowerTemp,self.LowerHumid,self.RaiseSMoist,self.LowerSMoist,self.Ping,self.Camera,self.Email]
        
        self.Behavioral=layers.BehavioralLayer(self.sensors,self.actuators,self.behaviors)
        self.Planning=layers.PlanningLayer(schedulefile)
        self.Executive=layers.ExecutiveLayer()


        self.Planning.setExecutive(self.Executive)
        self.Executive.setBehavioralLayer(self.Behavioral)
        self.Executive.setPlanningLayer(self.Planning)
        self.Planning.getNewSchedule()
        self.Executive.setMonitors(self.sensors, self.actuators.actuator_state, [logging_monitor.LoggingMonitor()])
        # END STUDENT CODE

    def main(self):
        wait_for_sensors(self.sensors)
        while not rospy.core.is_shutdown():
            t = time_since_midnight(rospy.get_time())
            # Run a step of each layer of the architecture
            # BEGIN STUDENT CODE
            self.Behavioral.doStep()
            self.Planning.doStep(t)
            self.Executive.doStep(t)
            # END STUDENT CODE
            rospy.sleep(1)
            check_for_input()

if __name__ == '__main__':
    sim = False
    if "-m" in sys.argv and "sim" in sys.argv:
        sim = True
    if "-B" in sys.argv:
        print("Starting Behavioral Agent")
        agent = BehavioralGreenhouseAgent(sim)
        agent.main()
    elif "-L" in sys.argv:
        print("Starting Layered Agent")
        agent = LayeredGreenhouseAgent(sim,"greenhouse_schedule.txt")
        agent.main()
