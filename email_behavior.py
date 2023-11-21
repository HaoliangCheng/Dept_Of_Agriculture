from behavior import *
from transitions import Machine
import send_email
import os
from PIL import Image
from collections import defaultdict
import terrabot_utils
import cv_utils
import vision
'''
The behavior should send an email that includes the team name and TerraBot
number, the date and time, the current sensor and actuator readings, and
the most recent image taken
'''
class Email(Behavior):
    def __init__(self, camera):
        super(Email, self).__init__("EmailBehavior")
        # Your code here
	# Initialize the FSM and add transitions
        # BEGIN STUDENT CODE
        self.camera = camera
        self.initial = 'halt'
        # STUDENT CODE: Modify these lines to add all your FSM states
        self.states = [self.initial, 'active']
        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition('enable', 'halt', 'active', after=["sendEmail","setInitial"])
        self.fsm.add_transition('disable', 'active', 'halt')
        self.email_index = 1
        self.categories = defaultdict(list)
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

    # hook used by other behaviors to update state of email
    def update(self, category, text):
        self.categories[category].append(text)

    def sendEmail(self):
        from_address = "TerraBot8@outlook.com"
        password = "ODonnell459"
        to_address = "hongsng@andrew.cmu.edu, haolian2@andrew.cmu.edu, yichenzh@andrew.cmu.edu"
        subject = f"Department of Agriculture's daily report (Day {self.email_index}): {terrabot_utils.clock_time(self.time)}"
        self.email_index += 1
        
        if self.camera.pathname == "":
            pathname = "../imagefolder/grow_period_b"
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
        print(f"last image pathname is {pathname}")
        imageName = pathname.split(".png")[0].split(".jpeg")[0]
        print(f"image name is {imageName}")
        lastImage = cv_utils.readImage(pathname) # np array
        lastImage = vision.colorCorrect(lastImage, [150,75,75], [75,150,75], [75,75,150])
        foliage, original, height = vision.foliageImages(lastImage)
        
        cv_utils.writeImage(imageName + ".jpeg", lastImage)
        cv_utils.writeImage(imageName + "foliage.jpeg", foliage)
        f = open(f"{imageName}foliage.jpeg", 'rb')
        photo1 = f.read()
        f2 = open(f"{imageName}.jpeg", 'rb')
        photo2 = f2.read()
        images = [photo1, photo2]
        greenery, health_msg = vision.plantHealth(lastImage)
        text = f''' 
        Light: {self.light},
        Temp: {self.temp}, 
        Humidity: {self.humid:.2f},
        Weight: {self.weight:.2f} grams,
        Soil Moisture: {self.smoist:.2f},
        Water Level: {(self.level * 18):.2f} ml left,    
        Time of last watering: {self.get_last_watering()},
        Watering in the last 24 hours: {self.get_watering_summary()},
        Previous day's total insolation: {self.get_insolation_summary()}    ,
        Proportion of greenery in last image: {greenery:.2f}
        Plant health compared to yesterday: {health_msg}
        Plant height: {height:.2f}

'''     
        print("sending email!!")
        # TODO: make instructors = True
        send_email.send(from_address, password, to_address, subject, text, images = images, instructors = True)
        # reset state once a day
        self.categories = defaultdict(list)

    def get_last_watering(self):
        if self.categories["last_watered_time"] != []:
            return terrabot_utils.clock_time(int(self.categories["last_watered_time"][-1]))
        
        return "We just watered at 12.30pm today"
    def get_insolation_summary(self):
        if "insolation" in self.categories and self.categories["insolation"] != []:
            return self.categories["insolation"][0]
        
        return "No summary - just initialized"
    
    def get_watering_summary(self):
        volumes = self.categories['water_added']
        explanations = self.categories['explanation']
        assert(len(volumes) == len(explanations)), "unequal amount of volumes and explanations"
        if len(volumes) == 0:
            return "NO water was added because moisture level is good and we recently watered"
        
        text = ""
        for i in range(len(volumes)):
            text += f"Watered {volumes[i]} of water because of {explanations[i]} \n"

        return text
    def act(self):
        self.trigger("doStep")
