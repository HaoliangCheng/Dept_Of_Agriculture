from behavior import *
from limits import *
from transitions import Machine

#sensor data passed into greenhouse behaviors:
#  [time, lightlevel, temperature, humidity, soilmoisture, waterlevel]
#actuators are looking for a dictionary with any/all of these keywords:
#  {"led":val, "fan":True/False, "pump": True/False}


'''
The combined ambient and LED light level between 8am and 10pm should be 
in the optimal['light_level'] range;
Between 10pm and 8am, the LEDs should be off (set to 0).
'''
class Light(Behavior):

    def __init__(self):
        super(Light, self).__init__("LightBehavior")
        self.optimal_level = optimal['light_level']

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'halt'
        self.states = [self.initial,"start","wait4night","wait4morning"]

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition("enable", self.initial, "start", after="setInitial")
        
        self.fsm.add_transition("disable", "wait4night"   , self.initial,  after="setInitial")
        self.fsm.add_transition("disable", "wait4morning" , self.initial,  after="setInitial")

        
        self.fsm.add_transition("doStep",  "start", "wait4night", unless=["morning"], after="setdark")
        self.fsm.add_transition("doStep",  "start", "wait4morning", conditions=["morning"],after="increaselight")
        
        self.fsm.add_transition("doStep", "wait4night", "wait4morning" , conditions=["morning"], after="increaselight")
        self.fsm.add_transition("doStep", "wait4morning" , "wait4night", unless=["morning"], after="setdark")
        self.fsm.add_transition("doStep", "wait4morning" , "wait4morning", conditions=["lower"], after="increaselight")
        self.fsm.add_transition("doStep", "wait4morning" , "wait4morning", conditions=["higher"], after="decreaselight")
     
        # END STUDENT CODE
        
    def setInitial(self):
        self.led = 0
        self.setLED(self.led)
        
    def perceive(self):
        self.mtime = self.sensordata["midnight_time"]
        self.time = self.sensordata["unix_time"]
        self.light = self.sensordata["light"]
    
    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")
        
    # Add all your condition functions here
    # BEGIN STUDENT CODE
        
    def morning(self):
        hour = (self.mtime//3600)%24
        return hour >= 8 and hour < 22
        
    def lower(self):
        return self.light < self.optimal_level[0]
        
    def higher(self):
        return self.light >= self.optimal_level[1]

    # END STUDENT CODE
        
    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def setdark(self):
       self.setLED(0)
       
    def increaselight(self):
       self.setLED(self.led+20)
       
    def decreaselight(self):
       self.setLED(self.led-20)
       
    def setOptimal(self,level):
        self.optimal_level=[level*0.97,level*1.03]
       
    # END STUDENT CODE
    def setLED(self, level):
        self.led = max(0, min(255, level))
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": self.led}))
                                  

"""
The temperature should be greater than the lower limit
"""
class RaiseTemp(Behavior):

    def __init__(self):
        super(RaiseTemp, self).__init__("RaiseTempBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'halt'
        self.states = [self.initial,"start","tooLowT","PerfectT","increase"]

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition("enable", self.initial, "start", after="setInitial")
        self.fsm.add_transition("doStep", "start", "PerfectT", conditions=["perfect"],after="close")
        self.fsm.add_transition("doStep", "start", "tooLowT", conditions=["toolow"],after="open")
        
        self.fsm.add_transition("doStep", "tooLowT", "PerfectT", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "tooLowT", "tooLowT", conditions=["toolow"], after="open")
        self.fsm.add_transition("doStep", "PerfectT", "tooLowT", conditions=["toolow"], after="open")
        
        self.fsm.add_transition("disable", "PerfectT", self.initial, after="setInitial")
        self.fsm.add_transition("disable", "tooLowT", self.initial, after="setInitial")
        # END STUDENT CODE

    def setInitial(self):
        self.setLED(0)
        
    def perceive(self):
        self.temp = self.sensordata["temp"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def toolow(self):
        return self.temp < limits['temperature'][0]
        
    def perfect(self):
        return self.temp >= optimal['temperature'][0]
        
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def open(self):
       self.setLED(200)
       print("Turning up the lights to raise the temperature")
    def close(self):
       self.setLED(0)
       print("Temperature is now perfect!")
    # END STUDENT CODE
            
    def setLED(self, level):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"led": level}))
        
"""
The temperature should be less than the upper limit
"""
class LowerTemp(Behavior):

    def __init__(self):
        super(LowerTemp, self).__init__("LowerTempBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'halt'
        self.states = [self.initial,"start","tooHigh","Perfect"]

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition("enable", self.initial, "start", after="setInitial")
        self.fsm.add_transition("doStep", "start", "Perfect", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "start", "tooHigh", conditions=["toohigh"], after="open")
        self.fsm.add_transition("doStep", "tooHigh", "Perfect", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "Perfect", "tooHigh", conditions=["toohigh"], after="open")
        
        self.fsm.add_transition("disable", "tooHigh", self.initial, after="setInitial")
        self.fsm.add_transition("disable", "Perfect", self.initial, after="setInitial")
        # END STUDENT CODE

    def setInitial(self):
        self.setFan(0)
        
    def perceive(self):
        self.temp = self.sensordata["temp"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def toohigh(self):
        return self.temp >= limits['temperature'][1]
    def perfect(self):
        return self.temp <= optimal['temperature'][1]
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def open(self):
       print("Temperature is too high")
       self.setFan(True)
    def close(self):
       print("Temperature not high now")
       self.setFan(False)
    # END STUDENT CODE
            
    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))
    
"""
Humidity should be less than the limit
"""
class LowerHumid(Behavior):

    def __init__(self):
        super(LowerHumid, self).__init__("LowerHumidBehavior")

        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'halt'
        self.states = [self.initial,"start","tooHumid","Perfect"]

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition("enable", self.initial, "start", after="setInitial")
        
        self.fsm.add_transition("doStep", "start", "Perfect", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "start", "tooHumid", conditions=["toohumid"], after="open")
        self.fsm.add_transition("doStep", "tooHumid", "Perfect", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "Perfect", "tooHumid", conditions=["toohumid"], after="open")
        
        self.fsm.add_transition("disable", "Perfect", self.initial, after="setInitial")
        self.fsm.add_transition("disable", "tooHumid", self.initial, after="setInitial")
        # END STUDENT CODE

    def setInitial(self):
        self.setFan(0)
        
    def perceive(self):
        self.humid = self.sensordata["humid"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def toohumid(self):
        return self.humid >= limits["humidity"][1]
    def perfect(self):
        return self.humid <= optimal["humidity"][1]
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def open(self):
       self.setFan(True)
    def close(self):
       self.setFan(False)
    # END STUDENT CODE

    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))
            
"""
Soil moisture should be greater than the lower limit
"""
class RaiseSMoist(Behavior):

    def __init__(self):
        super(RaiseSMoist, self).__init__("RaiseMoistBehavior") #Behavior sets reasonable defaults
        self.state = "halt"
        self.weight = 0
        self.weight_window = []
        self.smoist_window = []
        self.total_water = 0
        self.waterlevel = 0
        self.last_time = 24*60*60 # Start with the prior day
        self.daily_limit = 50 #100
        self.moisture_opt = optimal["moisture"][0]
        
        
    def setInitial(self, state):
        self.state = state
        
    def enable(self):
        self.setInitial("init")
        self.setPump(False)
        self.setTimer(10) # Give the sliding windows time to average out

    def disable(self):
        self.setInitial("halt")
        self.setPump(False)
        self.setLastTime()
        
    def sliding_window(self, window, item, length=4):
        if (len(window) == 0):
            window = [item]*length
        else:
            window = window[1:]
            window.append(item)
        return window, sum(window)/float(length)
    
    def perceive(self):
        self.time = self.sensordata["unix_time"]
        self.mtime = self.sensordata["midnight_time"]
        self.waterLevel = self.sensordata["level"]
        self.weight = self.sensordata["weight"]
        self.weight_window, self.weight_est = self.sliding_window(self.weight_window, self.weight)
        self.smoist = self.sensordata["smoist"]
        self.smoist_window, self.smoist_est = self.sliding_window(self.smoist_window, self.smoist)

    def act(self):
        if self.state == "init":
            if self.last_time > self.mtime: #it's the next day
                print("Next day!")
                self.resetTotalWater()
            elif self.time >= self.waittime: #time is up
                self.state = "waiting"
        elif self.state == "waiting":
            if self.total_water >= self.daily_limit: #watered enough
                self.printWateredEnough()
                self.state = "done"
            elif self.waterLevel < 30: #not enough water in reservoir
                print("NOT ENOUGH WATER IN RESERVOIR")
            elif self.smoist_est >= self.moisture_opt: #soil is moist enough
                print("Soil is moist enough (%s)" %self.smoist_est)
                self.state = "done"
            elif self.smoist_est < self.moisture_opt: #soil is too dry
                print("Soil too dry (%s) - need to water" %self.smoist_est)
                self.start_weight = self.weight_est
                self.setTimer10()
                self.setPump(True)
                self.state = "watering"
        elif self.state == "watering":
            if self.time >= self.waittime: #time is up
               self.setPump(False)
               self.setTimer20()
               self.state = "measuring"
        elif self.state == "measuring":
            if self.time >= self.waittime: #time is up
                self.calcWaterAdded()
                self.state = "waiting"
                
    def setTimer(self, wait):
        self.waittime = self.time + wait
        #print("setTimer: %d (%d)" %(self.waittime, wait))
    def setTimer10(self): self.setTimer(10)
    def setTimer20(self): self.setTimer(20)

    def setLastTime(self): self.last_time = self.mtime
    def resetTotalWater(self): # Reset total water each day
        print("Resetting total water")
        self.total_water = 0
        self.setLastTime()

    def calcWaterAdded(self):
        dwater = self.weight_est - self.start_weight # ml of water weighs a gram
        # Sometimes scales are off - cannot lose weight after watering
        dwater = max(0, dwater)

        self.total_water += dwater
        print("calcWaterAdded: %.1f (%.1f = %.1f - %.1f)"
              %(self.total_water, dwater, self.weight_est, self.start_weight))
        
    def printWateredEnough(self):
        print("Watered Enough: %.1f" %self.total_water)

    def emailNoWater(self): # Email TBD
        print("NOT ENOUGH WATER IN RESERVOIR")

    def setPump(self,state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"wpump": state}))
        

"""
Soil moisture below the upper limit
"""
class LowerSMoist(Behavior):

    def __init__(self):
        super(LowerSMoist, self).__init__("LowerMoistBehavior")


        # STUDENT CODE: Modify these lines to use your own initial state name
        #               and add all your FSM states
        self.initial = 'halt'
        self.states = [self.initial,"start","tooMoist","MoisEnough"]

        self.fsm = Machine(self, states=self.states, initial=self.initial,
                           ignore_invalid_triggers=True)

        # Add FSM transitions and actions
        # BEGIN STUDENT CODE
        self.fsm.add_transition("enable", self.initial, "start",  after="setInitial")
        
        self.fsm.add_transition("doStep", "start", "tooMoist", conditions=["toomoist"], after="open")
        self.fsm.add_transition("doStep", "start", "MoisEnough", conditions=["perfect"])
        
        self.fsm.add_transition("doStep", "tooMoist", "MoisEnough", conditions=["perfect"], after="close")
        self.fsm.add_transition("doStep", "MoisEnough", "tooMoist", conditions=["toomoist"], after="open")
        
        self.fsm.add_transition("disable", "MoisEnough", self.initial, after="setInitial")
        self.fsm.add_transition("disable", "tooMoist", self.initial, after="setInitial")
        # END STUDENT CODE

    def setInitial(self):
        self.setFan(0)
        
    def perceive(self):
        self.smoist = self.sensordata["smoist"]

    def act(self):
        # Use 'doStep' trigger for all other transitions
        self.trigger("doStep")

    # Add all your condition functions here
    # BEGIN STUDENT CODE
    def toomoist(self):
        return self.smoist >= limits["moisture"][1]
        
    def perfect(self):
        return self.smoist <= optimal["moisture"][1]
    # END STUDENT CODE

    # Add all your before / after action functions here
    # BEGIN STUDENT CODE
    def open(self):
       self.setFan(True)
       print("Turning on the fan to lower soil moisture")
       
    def close(self):
       self.setFan(False)
       print("Soil moisture is now perfect!")
       
    # END STUDENT CODE
            
    def setFan(self, act_state):
        self.actuators.doActions((self.name, self.sensors.getTime(),
                                  {"fan": act_state}))

