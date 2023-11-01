from monitor import *

class LoggingMonitor(Monitor):

    def __init__(self, period=100):
        super(LoggingMonitor, self).__init__("LoggingMonitor", period)
        # Put any iniitialization code here
        # BEGIN STUDENT CODE
        #self.filename = f"/home/robotanist/Desktop/TerraBot/agents/Mon_HW/log_output/log1.txt"
        self.filename = f"/home/robotanist/Desktop/TerraBot/agents/logs/log.txt"
        with open(self.filename, 'w') as file:
            pass
        # END STUDENT CODE

    def perceive(self):
        # BEGIN STUDENT CODE
        #self.sensordata = self.sensors.doSense()
        self.time = self.sensordata["unix_time"]
        # END STUDENT CODE
        pass
    
    def unix_to_hms(self, unix_timestamp):
        seconds_since_midnight = unix_timestamp % 86400
        hour = seconds_since_midnight // 3600
        minute = (seconds_since_midnight % 3600) // 60
        second = seconds_since_midnight % 60
        return hour, minute, second

    def monitor(self):
        """
        if (not self.filename):
            self.filename = f"/home/robotanist/15_482/Mon_HW/log{int(self.time)}.txt"
        """
        with open(self.filename, 'a') as f:
            #f.write(f"{clock_time(self.time)}")
            #f.write(str(self.time))
            hour, minute, second =  self.unix_to_hms(self.time)
            f.write(f"TIME: {int(hour):02}:{int(minute):02}:{int(second):02}")
            
            for key in self.sensordata:
                f.write(f", {key}: {self.sensordata[key]}")
            for key in self.actuator_state:
                f.write(f", {key}: {self.actuator_state[key]}")
            f.write("\n")

