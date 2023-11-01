from behavior import *
from transitions import Machine
import sys, os.path as op
import os
from terrabot_utils import clock_time
import time

'''
The behavior should adjust the lights to a reasonable level (say 400-600),
wait a bit for the light to stabilize, and then request an image.
It should check to be sure the image has been recorded and, if so, process
the image; if not, try again for up to 3 times before giving up
'''
class TakeImage(Behavior):
    def __init__(self):
        super(TakeImage, self).__init__("TakeImageBehavior")
        # Your code here
	# Initialize the FSM and add transitions
        # BEGIN STUDENT CODE
        self.trytime = 0
        self.phototime = 0
        self.lasttime = 86400
        self.initial = 'halt'
        self.pathname=""
        self.states = [self.initial,"start","wait4light","wait4file","filenotshow","fileshow","warning","setnewday","notsetnewday"]
        self.fsm = Machine(self, states=self.states, initial=self.initial, ignore_invalid_triggers=True)
        
        self.fsm.add_transition("enable", self.initial, "setnewday", conditions=["newday"], after="setday")
        self.fsm.add_transition("enable", self.initial, "notsetnewday", unless=["newday"],after="printnotnewday")
        self.fsm.add_transition("doStep", "setnewday", "start", after="setInitial")
        self.fsm.add_transition("doStep", "notsetnewday", "start", conditions=["not3photostoday"], after="setInitial")
        self.fsm.add_transition("disable", "notsetnewday", self.initial, unless=["not3photostoday"], after="printreminder")
               
        self.fsm.add_transition("doStep", "start", "wait4light", conditions=["lightfit"],after="takephoto")
        self.fsm.add_transition("doStep", "start" , "start", conditions=["lower"], after="increaselight")
        self.fsm.add_transition("doStep", "start" , "start", conditions=["higher"], after="decreaselight")
        self.fsm.add_transition("doStep", "wait4light", "wait4file", conditions=["timeup"],after="waitfile")
        
        self.fsm.add_transition("doStep", "wait4file", "filenotshow", conditions=["notfile"],after="wait20")
        self.fsm.add_transition("doStep", "filenotshow", "warning", conditions=["try3times"], after="printwarning")
        self.fsm.add_transition("doStep", "filenotshow", "start", conditions=["timeup","nottry3times"],after="tryagain")
        self.fsm.add_transition("disable", "warning", self.initial)
        
        self.fsm.add_transition("doStep", "wait4file", "fileshow", unless=["notfile"], after="setimagetimes")
        self.fsm.add_transition("disable", "fileshow", self.initial)
        # END STUDENT CODE

    # Add the condition and action functions
    #  Remember: if statements only in the condition functions;
    #            modify state information only in the action functions
    # BEGIN STUDENT CODE

    #  conditions
    def lightfit(self):
        return self.light >= 400 and self.light <= 600
    def lower(self):
        return self.light < 400
        
    def higher(self):
        return self.light >= 600
    def notfile(self):
        return not op.exists(self.pathname)
    def try3times(self):
        return self.trytime > 2  
    def nottry3times(self):
        return self.trytime <= 2  
    def not3photostoday(self):
        return self.phototime < 3 
    def newday(self):
        return self.lasttime + 1200 > self.mtime 
    def timeup(self):
        return self.time >= self.waittime
 
    # actions
    def tryagain(self):
        self.pathname="/home/robotanist/Desktop/TerraBot/imagefolder/photo"+str(self.time)+".png"
    
    def waitfile(self):
        print("waitfile\n")  
  
    def printnotnewday(self): 
        print("not new day\n")  
        
    def setday(self):
        print("new day\n")   
        self.phototime=0
        self.lasttime=self.mtime
        
    def takephoto(self):
        print("take one\n")   
        self.setcamera(self.pathname)
        self.waittime = self.time + 15  
     
    def printreminder(self):
        print("Already take 3 images today!")    
        
    def setimagetimes(self):        
        self.phototime=self.phototime+1
        print("Today:"+str(self.phototime)+"\n")

        
    def wait20(self):
        self.trytime=self.trytime+1   
        print("file not show, try again")   
        print(str(self.trytime)+"\n")  
        self.waittime = self.time + 20 
        
    def increaselight(self):
        self.setLED(self.led+20)
       
    def decreaselight(self):
        self.setLED(self.led-20)
               
    def printwarning(self):
        self.trytime=0
        print("Images don't show up!")    
        
    def setInitial(self):
        print("start to take photo\n")   
        self.trytime=0
        self.pathname="/home/robotanist/Desktop/TerraBot/imagefolder/photo"+str(self.time)+".png"
        self.led = 500
        self.setLED(self.led)       
            
    def setcamera(self, path_name):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"camera": path_name}))                            
    def setLED(self, level):
        self.led = max(0, min(255, level))
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": self.led }))
    # END STUDENT CODE

    def perceive(self):
        self.time = self.sensordata['unix_time']
        # Add any sensor data variables you need for the behavior
        # BEGIN STUDENT CODE
        self.mtime = self.sensordata["midnight_time"]
        self.light = self.sensordata["light"]
        # END STUDENT CODE
    
    def act(self):
        self.trigger("doStep")
