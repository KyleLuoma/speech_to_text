import pyttsx3

class RobotVoice:

    def __init__(self):
        self.engine = pyttsx3.init(driverName="espeak")
        voices = self.engine.getProperty('voices') 
        #self.engine.setProperty('voice', voices[1].id)



    def say_something(self, message):
        self.engine.say(message)
        self.engine.runAndWait()


