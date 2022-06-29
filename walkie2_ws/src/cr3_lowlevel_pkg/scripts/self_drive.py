# cr3 library
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time

# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
client_feedback = dobot_api_feedback('192.168.5.6', 30003)


client_dashboard.DisableRobot()
time.sleep(1)

# Remove alarm
client_dashboard.ClearError()
time.sleep(0.5)

# Description The upper function was enabled successfully
client_dashboard.EnableRobot()
time.sleep(0.5)
# Select user and Tool coordinate system 0
client_dashboard.User(0)
client_dashboard.Tool(0)

while True:
    # Remove alarm
    client_dashboard.ClearError()
    time.sleep(2)
    # Description The upper function was enabled successfully
    client_dashboard.ResetRobot()
    client_dashboard.BrakeControl(6,0)
    client_dashboard.BrakeControl(5,0)
    client_dashboard.StartDrag()
    # client_dashboard.BrakeControl(6,0)
    # client_dashboard.SetCollideDrag(0)