#!/usr/bin/python

import math
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time
import numpy as np


# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
client_feedback = dobot_api_feedback('192.168.5.6', 30003)

client_dashboard.AccL(10)
client_dashboard.SpeedL(10)
client_feedback.RelMovL(+100, 0, 0)
time.sleep(5)

