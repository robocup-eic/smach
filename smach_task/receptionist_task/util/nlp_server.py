#!/usr/bin/env python

"""
sudo netstat -nlp | grep 5000
kill -9 <pid_id>
"""

from flask import Flask, request
import threading
import time
import json
import requests

class SpeechToText():

    def __init__(self, name):
        self.app = Flask(name)
        self.app.add_url_rule("/", "index", self.greet)
        self.app.add_url_rule("/ros", "intent", self.intent, methods=["POST"])
        self.body = None

    def run(self):
        self.app.run(host = "0.0.0.0", port = 5000, threaded = True)

    def greet(self):
        return "<h1>Hello World</h1>"

    def intent(self):
        body = request.get_json()
        self.body = json.loads(body)
        print(self.body)
        return {"success" : True}
    
    def clear(self):
        self.body = None

def speak(text) :
    try :
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text':text})
        return x
    except :
        print("error to connect speak api.")

if __name__=="__main__":
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()
    while True:
        print(stt.body)
        time.sleep(0.1)
    