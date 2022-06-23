from flask import Flask, request
import threading

class SpeechToText():


    def __init__(self, name):
        self.app = Flask(name)
        self.app.add_url_rule("/", "index", self.greet)
        self.app.add_url_rule("/intent", "intent", self.intent, methods=["POST"])
        self.body = None

    def run(self):
        self.app.run(host = "127.0.0.1", port = 3000,threaded = True)

    def greet(self):
        return "<h1>Hello World</h1>"

    def intent(self):
        body = request.get_json()
        self.body = body
        print(self.body)
        return {"success" : True}

stt = SpeechToText("nlp")
stt.run()