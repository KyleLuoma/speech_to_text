from json import JSONEncoder
import json
import speech_recognition as sr
import pyaudio


class MicrophoneListener:

    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.paudio = pyaudio.PyAudio()
        with open('/home/ubuntu/catkin_ws/src/speech_to_text/src/dialogue_package/local/my-speech-service-343100-9df89039471a.json', 'r') as f:
            self.google_credentials = json.load(f)

        
    
    def list_microphones(self):
        print("Microphone count", self.paudio.get_device_count())
        for microphone in sr.Microphone().list_microphone_names():
            print(microphone)



    def listen(self):
        stop_listening = False
        self.recognizer.pause_threshold = 1.25
        self.recognizer.energy_threshold = 4000
        with self.microphone as source:
            print("Listening...")
            speech = self.recognizer.listen(source)
            print("Ok, I heard you. Sending sound file to Google Cloud for transcription.")
        try:
            transcript = self.recognizer.recognize_google_cloud(
                    audio_data=speech,
                    preferred_phrases=[
                        'party bot', 'send a message', 'host', 'stop program', "social bot"
                        ], #NOTE: Error in speech_recognition library __init__.py must change ["speechContext"] to ["speechContexts"] on line 924
                    credentials_json = JSONEncoder().encode(self.google_credentials)
                )
            print("Here's what I think you said:", transcript)
            return transcript
        except sr.UnknownValueError:
            print("Didn't understand what you said.")
            return "unrecognized input"
        


