import requests
import json

def speak(text) :
    try :
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text':text})
        # printclr(e,"red")
        return x.json()
        # return "Syntizied"

    except Exception as e:
        pass
        # printclr(e,"red")


def ww_listen(): # go to Wakeword server
    try:
        response = requests.get("http://localhost:5100/").json() # wakeword get
        while response["confidence"] < 0.62:
            speak("Sorry I didn't get that, could you speak louder or rephrase the sentence?")
            response = requests.get("http://localhost:5101/").json() # asr get
        return response
    except Exception as e:
        pass
        # printclr(e,"red")

def listen(return_json=False): # go to ASR server, By-pass wakeword
    try:
        response = requests.get("http://localhost:5101/").json() # asr get
        while response["confidence"] < 0.62:
            speak("Sorry I didn't get that, could you speak louder or rephrase the sentence?")
            response = requests.get("http://localhost:5101/").json() # asr get
        return response
    except Exception as e:
        # printclr(e,"red")
        pass

def get_intent(predicted_text):
        response = {
            "recipient_id": "bot",
            "body": predicted_text
        }

        #TODO try and except UGLY.......
        # try:
        r = requests.post(url= "http://localhost:5005/webhooks/rest/webhook", 
                    json={"sender": "bot", "message": predicted_text}
                    )
        # printclr(r.json(),"cyan")
        if r.json() == []: #TODO FIX THIS BULLSHIT
            print("Low confidence level")
            response.update({"confidence": 0})
        else:
            rasa_json = r.json()[0]['text']
            rasa_json = json.loads(rasa_json)
            # printclr(rasa_json,"red")
            # printclr(response,"red")
            response.update(rasa_json)
            # printclr(json.dumps(response, indent=4),"blue")

            #* get confidence
            r = requests.post(url= "http://localhost:5005/model/parse", 
                        json={"text": predicted_text}
                        )
            if r.json() == []:
                print("Low confidence level")
                response.update({"confidence": 0})
            else:
                confidence = r.json()['intent_ranking'][0]['confidence']
                indent_name = r.json()['intent_ranking'][0]['name']
                # printclr(f"\t{confidence=}","blue")
                # printclr(f"\t{indent_name=}","blue")
                # printclr(f"\t{json.dumps(response, indent=4)}","blue")

                
                # printclr(dict(json.dumps(r.json()[0]['text'])),"red")
                if indent_name == str(response["intent"]):
                    response.update({"confidence": confidence})
                else:
                    pass
                    # printclr("its not the same","red")
        # print(response)
        # printclr(f"\t{json.dumps(response, indent=4)}","blue")
        # printclr(f"\tlisten() sending back...","green")
        return response

# import time
# import json 
# start = time.time()
# time.sleep(1)

# print(time.time() - start)
def main():
    # clearterm() 
    pass

    # print("this is speak")
    # print(speak("say stop motherfucker"))
    # # print(speak("say "))
    # # print(listen())

    # print(json.dumps(listen(), indent=4))
    # # print(json.dumps(ww_listen(), indent=4))
    
    # print(get_intent(predicted_text="Can I have a coke?"))
    # while True:
        # x = dict(ww_listen())
    # print(ww_listen())
        # if x['intent'] == "stop":
        #     printclr("STOPPINGGGG........","red")
        #     speak("stop")

main()
    # print(ww_listen())
# print(speak("hi there this is a test for speak"))
