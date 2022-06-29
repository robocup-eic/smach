# Readme

## 1. The following steps are required to run Demo
1. The computer can be connected to the network port of the controller with a network cable, and then set a fixed IP, which is in the same network segment as the controller IP. The controller can also be connected wirelessly.

    - Four-axis robots (such as MG400, etc.) When wired connection is connected to LAN1: ip is 192.168.1.6, when wired connection is connected to LAN2: ip is 192.168.2.6, wireless connection: ip is 192.168.9.1 .
    - Six-axis robot (such as CR series, etc.) Wired connection: ip is 192.168.5.1 ,Wireless connection: ip is 192.168.1.6 .

2. Try to ping the controller IP to make sure it is in the same network segment.


## 2. File description
1. demo.py: The entry point of the program.  
   
2. dobot_api.py: According to the robot TCP/IP remote control scheme (https://github.com/Dobot-Arm/TCP-IP-Protocol), modify it by yourself.

## 3. Run Demo
Method 1: If you need to detect and search the dynamic library, you need to open the entire directory in **VsCode**, and then run demo.py directly.  

Method 2: To detect and search the dynamic library, you need to open the entire directory in **PyCharm**, and then run demo.py directly.

## 4. Test environment
- language: Python 3.7 64-bit
- os: Windows 10 64-bit

## 5. Controller version
The controller versions that can use the TCP/IP protocol are as follows:

- MG400: 1.5.4.0 and above
- CR: 3.5.1.9 and above

## visual servo

visual_servo.launch :  launch simple object detection and run visual servo as node
visual_servo_service.launch : launch visual servo as service. Open another terminal and use rosservice call