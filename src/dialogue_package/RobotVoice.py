import pyttsx3

class RobotVoice:

    def __init__(self):
        self.engine = pyttsx3.init(driverName="espeak")
        voices = self.engine.getProperty('voices') 
        self.engine.setProperty('voice', 'english_rp+f3')
        self.engine.setProperty('rate', 130)
        self.talking = False



    def say_something(self, message):
        print("RobotVoice is Saying: ", message)
        self.talking = True
        self.engine.say(message)
        self.engine.runAndWait()
        print("RobotVoice is done talking")
        self.talking = False


    
    def is_talking(self):
        return self.talking


