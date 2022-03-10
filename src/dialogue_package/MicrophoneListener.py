from json import JSONEncoder
import json
import speech_recognition as sr
import pyaudio


class MicrophoneListener:

    def __init__(self):
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.paudio = pyaudio.PyAudio()
        with open('/home/ubuntu/catkin_ws/src/speech_to_text/src/dialogue_package/local/my-speech-service-343100-162a57910e55.json', 'r') as f:
            self.google_credentials = json.load(f)
        self.preferred_phrases = [
            'send a message', 'host', 'social bot', 'click a picture', 'click picture',
                'take a picture', 'take picture', 'take a pic', 'take a pick',
                'take pic', 'take pick', 'click a photograph', 'click photograph',
                'take a photograph', 'take photograph', 'click a photo',
                'click photo', 'take a photo', 'take photo'
        ]
        self.default_energy_threshold = 4000
        self.energy_threshold = self.default_energy_threshold
        self.use_dymanmic_energy_threshold = False
        self.recognizer.dynamic_energy_threshold = self.use_dymanmic_energy_threshold

        
    
    def list_microphones(self):
        print("Microphone count", self.paudio.get_device_count())
        for microphone in sr.Microphone().list_microphone_names():
            print(microphone)



    def mute_microphone(self):
        print("Muting microphone")
        self.recognizer.dynamic_energy_threshold = False
        self.recognizer.energy_threshold = 100000
        
        
        
    def unmute_microphone(self):
        print("Unmuting microphone")
        self.recognizer.energy_threshold = self.default_energy_threshold



    def listen(self):
        stop_listening = False
        self.recognizer.pause_threshold = 0.75
        with self.microphone as source:
            print("Listening...")
            speech = self.recognizer.listen(
                source,
                phrase_time_limit = 59
                )
            print("Ok, I heard you. Sending sound file to Google Cloud for transcription.")
        try:
            transcript = self.recognizer.recognize_google_cloud(
                    audio_data=speech,
                    preferred_phrases = self.preferred_phrases, #NOTE: Error in speech_recognition library __init__.py must change ["speechContext"] to ["speechContexts"] on line 924
                    credentials_json = JSONEncoder().encode(self.google_credentials)
                )
            print("Here's what I think you said:", transcript)
            return transcript
        except sr.UnknownValueError:
            print("Didn't understand what you said.")
            return "unrecognized input"
        


