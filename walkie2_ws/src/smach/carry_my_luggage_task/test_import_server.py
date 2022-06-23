from nlp_server import SpeechToText
import threading

stt = SpeechToText("nlp")
t = threading.Thread(target = stt.run, name="nlp")
t.start()
print('123')