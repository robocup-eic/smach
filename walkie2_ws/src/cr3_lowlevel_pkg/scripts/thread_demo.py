import threading
import time

def a():
    global enable
    while enable:
        print(threading.currentThread().getName())

def b():
    global enable
    while True:
        print(threading.currentThread().getName())



if __name__ == "__main__":
    enable = True

    t1 = threading.Thread(name ="thread1", target = a)
    t2 = threading.Thread(name ="thread2", target = b)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            enable = False
            print("Killing thread")
