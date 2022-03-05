
from google.cloud import speech

class GsrConnector:

    def __init__(self):
        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding = speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code="en-US",
        )
        pass

    def sendAudioToGsr(self, audio_file):
        content = audio_file.read()
        audio = speech.RecognitionAudio(content = content)
        return self.client.recognize(
            config = self.config,
            audio = audio
        )


