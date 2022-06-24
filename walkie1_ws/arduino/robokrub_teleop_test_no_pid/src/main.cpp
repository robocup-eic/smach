#include <Arduino.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

//Setting the timer used for interrupts
#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#define CONTROL_INTERVAL_MS 50L
#define PUB_PERIOD 100L

#include <TimerInterrupt.h>

//Import encoder library and servo for motor control
#include <Encoder.h>
#include <Servo.h>

//Import ros-related libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 goal_left_msg;
std_msgs::Float32 current_vel_msg;
std_msgs::Float32 pwr_msg;

ros::Publisher goal_left_pub("goal_left", &goal_left_msg);
ros::Publisher current_vel_pub("current_vel", &current_vel_msg);
ros::Publisher pwr_pub("pwr", &pwr_msg);

//Define Wheel Constant
#define WHEEL_NUM 2
#define WHEEL_RAD 0.125
#define WHEEL_DIST 0.547

//Define Encoders pin (need to be interrupt pins)
#define ENCA1 2
#define ENCA2 3

#define ENCB1 18
#define ENCB2 19

//Define ticks per revolution for converting ticks to rev
#define TICKS_PER_REV 2000

//Declare pins for motors
#define mobileBaseLPin 5
#define mobileBaseRPin 8

Servo mobileBaseL, mobileBaseR;
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);

long posPrev[WHEEL_NUM] = {0, 0};
long pos[WHEEL_NUM] = {0, 0}; 

float velocity[WHEEL_NUM] = {0.0, 0.0};
float goal[WHEEL_NUM] = {0.0, 0.0};
long pwr[WHEEL_NUM] = {0, 0};

//Declare Function
void GoalCb(const geometry_msgs::Twist&);
void publishState();
void controlMotor();
void setMotor(int ,Servo);

//ros publisher and transform broadcaster
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", GoalCb );


void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(goal_left_pub);
  nh.advertise(current_vel_pub);
  nh.advertise(pwr_pub);

  mobileBaseL.attach(mobileBaseLPin);
  mobileBaseR.attach(mobileBaseRPin);

  //Initiate Timer interrupt
  ITimer2.init();
  // ITimer1.init();

  //Attach interrupt to control motor function and odom calculate function
  if (ITimer2.attachInterruptInterval(CONTROL_INTERVAL_MS, controlMotor))
  {
    nh.loginfo("Starting  ITimer2 OK");
  }
  else
    nh.loginfo("Can't set interrupt on ITimer2");


}

void loop() {
  // put your main code here, to run repeatedly:
  pos[0] = encLeft.read();
  pos[1] = encRight.read();

  goal_left_msg.data = goal[0];
  current_vel_msg.data = velocity[0];
  pwr_msg.data = pwr[0];

  goal_left_pub.publish(&goal_left_msg);
  current_vel_pub.publish(&current_vel_msg);
  pwr_pub.publish(&pwr_msg);

  nh.spinOnce();
}

void GoalCb(const geometry_msgs::Twist& twist_msg){

  
  
  float x = twist_msg.linear.x;
  float z = twist_msg.angular.z;

  float goal_left = x-z*WHEEL_DIST/2;
  float goal_right = x+z*WHEEL_DIST/2;
  // if(goal_left != goal[0] || goal_right != goal[1]){
  //   nh.loginfo("RECEIVE NEW GOAL!");
  // }
  goal[0] = goal_left;
  goal[1] = goal_right;
}

void controlMotor()
{
  //Calculate velocity from position of motor
  velocity[0] = (pos[0]-posPrev[0])*1e3/((float) CONTROL_INTERVAL_MS);
  velocity[1] = (pos[1]-posPrev[1])*1e3/((float) CONTROL_INTERVAL_MS);
  //Not sure about this
  velocity[0] = (velocity[0] / TICKS_PER_REV)*2*M_PI*WHEEL_RAD;
  velocity[1] = (velocity[1] / TICKS_PER_REV)*2*M_PI*WHEEL_RAD;
  posPrev[0] = pos[0];
  posPrev[1] = pos[1];

  if((goal[0] == 0.0) && (goal[1] == 0.0)){
    setMotor(0,mobileBaseL);
    setMotor(0,mobileBaseR);
    return;
  }

  pwr[0] = 0; //something from the experiment
  pwr[1] = 0; //something from the experiment
  //Set motor power
  setMotor(pwr[0], mobileBaseL);
  setMotor(pwr[1], mobileBaseR);

 



}

void setMotor(int pwr, Servo servo)
{
  servo.write(pwr);

}