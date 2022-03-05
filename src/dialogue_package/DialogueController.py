
import MicrophoneListener as ml
import RobotVoice as rv
from pyphonetics import Metaphone

def main():
    mic_listener = ml.MicrophoneListener()
    voice = rv.RobotVoice()
    
    metaphone = Metaphone()
    

    phrase = "nothing"

    while not metaphone.sounds_like("stop program", phrase):
        voice.say_something("I'm listening.")
        phrase = mic_listener.listen()

        if metaphone.sounds_like("send a text message", phrase):
            voice.say_something("Ok, let's send the celebrant a text message.")

        elif metaphone.sounds_like("take a picture", phrase):
            voice.say_something("Ok, let's take a picture of your smiling faces!")

        elif metaphone.sounds_like("usher me to table", phrase):
            voice.say_something("Ok, follow me.")

        elif metaphone.sounds_like("social bot", phrase):
            voice.say_something("Hello, you called? What can I do for you?")

        else:
            voice.say_something("you said " + phrase + ". but I don't know what that means.")

    voice.say_something("goodbye.")

if (__name__ == "__main__"): main()