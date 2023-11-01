from behavior import *
from transitions import Machine
import send_email
import os
'''
The behavior should send an email that includes the team name and TerraBot
number, the date and time, the current sensor and actuator readings, and
the most recent image taken
'''
class Email(Behavior):
    def __init__(self, camera, raisemoist):
        super(Email, self).__init__("EmailBehavior")
        # Your code here
	# Initialize the FSM and add transitions
        # BEGIN STUDENT CODE
        self.camera = camera
        self.raisemoist = raisemoist
        self.initial = 'halt'
        # STUDENT CODE: Modify these lines to add all your FSM states
        self.states = [self.initial, 'active']
        # print("Created email object!")
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', 'halt', 'active', after=["sendEmail","setInitial"])
        self.fsm.add_transition('disable', 'active', 'halt')
        # END STUDENT CODE

    # Add the condition and action functions
    #  Remember: if statements only in the condition functions;
    #            modify state information only in the action functions
    # BEGIN STUDENT CODE
  
    # END STUDENT CODE

    def perceive(self):
        self.time = self.sensordata['unix_time']
        # Add any sensor data variables you need for the behavior
        # BEGIN STUDENT CODE
        self.light = self.sensordata['light']
        self.temp = self.sensordata['temp']
        self.humid = self.sensordata['humid']
        self.weight = self.sensordata['weight']
        self.smoist = self.sensordata['smoist']
        self.level = self.sensordata['level']
        # END STUDENT CODE

    def setInitial(self):
        self.last_time = self.time

    # TODO: update with latest image after camera_behavior is implemented
    def sendEmail(self):
        from_address = "TerraBot8@outlook.com"
        password = "ODonnell459"
        to_address = "hongsng@andrew.cmu.edu, haolian2@andrew.cmu.edu, yichenzh@andrew.cmu.edu"
        subject = f"Department of Agriculture's daily report: {self.time}"
        text = f''' 
        Light: {self.light},
        Temp: {self.temp}, 
        Humidity: {self.humid},
        Weight: {self.weight},
        Soil Moisture: {self.smoist},
        Water Level: {self.level}    
        Time of last watering: {self.raisemoist.last_watered_time}      
'''     
        if self.camera.pathname == "":
            pathname = "../imagefolder/"
            maxTime = 0
            maxName = ""
            for file in os.listdir(pathname):
                time = int(file.split("photo")[1].split(".png")[0].split(".")[0])
                if time > maxTime:
                    maxTime = time
                    maxName = "/" + str(file)
            pathname += maxName 
        else:
            pathname = self.camera.pathname
        f = open(pathname, 'rb')
        photo = f.read()
        send_email.send(from_address, password, to_address, subject, text, images = [photo], instructors = True)

    def act(self):
        self.trigger("doStep")

