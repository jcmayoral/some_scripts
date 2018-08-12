#!/usr/bin/env python3
# Requires PyAudio and PySpeech.

import speech_recognition as sr
from espeak import espeak

# Record Audio
m = sr.Microphone()
r = sr.Recognizer()
espeak.set_voice("es")

# Speech recognition using Google Speech Recognition
while True:
    try:
        # for testing purposes, we're just using the default API key
        # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
        # instead of `r.recognize_google(audio)`
        with m as source: r.adjust_for_ambient_noise(source)
        print("Say something!")
        with m as source: audio = r.listen(source)
        print("Got it! Now to recognize it...")
        print("You said: " + r.recognize_google(audio))
        espeak.synth(r.recognize_google(audio)) 
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
