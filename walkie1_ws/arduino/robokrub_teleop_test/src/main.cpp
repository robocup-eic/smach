#include <Arduino.h>
// #define ENCODER_OPTIMIZE_INTERRUPTS

// //Setting the timer used for interrupts
// #define USE_TIMER_1     false
// #define USE_TIMER_2     true
// #define USE_TIMER_3     false
// #define USE_TIMER_4     false
// #define USE_TIMER_5     false

// #define CONTROL_INTERVAL_MS 100L
#define PUB_PERIOD 50L

// #include <TimerInterrupt.h>

//Declare PID variables
extern "C"
{
#include <PID.h>
}

#define PID_KP 0.4f
#define PID_KI 1.0f
#define PID_KD 0.0f

#define PID_TAU 0.05f

#define PID_LIM_MIN -80.0f
#define PID_LIM_MAX 80.0f

#define PID_LIM_MIN_INT -40.0f
#define PID_LIM_MAX_INT 40.0f

#define SAMPLE_TIME_S 0.05f

//Import encoder library and servo for motor control
#include <Encoder.h>
#include <Servo.h>

//Import ros-related libraries
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <std_msgs/Float32.h>

std_msgs::Float32 goal_left_msg;
std_msgs::Float32 linear_vel_msg;
std_msgs::Float32 angular_vel_msg;
std_msgs::Float32 pwr_left_msg;
std_msgs::Float32 pwr_right_msg;

ros::Publisher goal_left_pub("goal_left", &goal_left_msg);
ros::Publisher linear_vel_pub("linear_vel", &linear_vel_msg);
ros::Publisher angular_vel_pub("angular_vel", &angular_vel_msg);
ros::Publisher pwr_left_pub("pwr_left", &pwr_left_msg);
ros::Publisher pwr_right_pub("pwr_right", &pwr_right_msg);

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
#define TICKS_PER_REV 1600

//Declare pins for motors
#define mobileBaseLPin 5
#define mobileBaseRPin 8

Servo mobileBaseL, mobileBaseR;
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);
PIDController pidLeft = {PID_KP,
                     PID_KI, PID_KD,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
PIDController pidRight = {PID_KP,
                     PID_KI, PID_KD,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};

long posPrev[WHEEL_NUM] = {0, 0};
long pos[WHEEL_NUM] = {0, 0}; 

float velocity[WHEEL_NUM] = {0.0, 0.0};
float goal[WHEEL_NUM] = {0.0, 0.0};
float pwr[WHEEL_NUM] = {0, 0};

//Declare Function
void GoalCb(const geometry_msgs::Twist&);
void publishState();
void controlMotor();
void setMotor(int ,Servo);

//ros publisher and transform broadcaster
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", GoalCb );

unsigned long currT = 0;
unsigned long prevT = 0;


void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.advertise(goal_left_pub);
  nh.advertise(linear_vel_pub);
  nh.advertise(angular_vel_pub);
  nh.advertise(pwr_left_pub);
  nh.advertise(pwr_right_pub);

  mobileBaseL.attach(mobileBaseLPin);
  mobileBaseR.attach(mobileBaseRPin);

  //Initiate PID controller
  PIDController_Init(&pidLeft);
  PIDController_Init(&pidRight);

  

  //Initiate Timer interrupt
  // ITimer2.init();
  // ITimer1.init();

  //Attach interrupt to control motor function and odom calculate function
  // if (ITimer2.attachInterruptInterval(CONTROL_INTERVAL_MS, controlMotor))
  // {
  //   nh.loginfo("Starting  ITimer2 OK");
  // }
  // else
  //   nh.loginfo("Can't set interrupt on ITimer2");


}

void loop() {
  // put your main code here, to run repeatedly:
  setMotor(pwr[0], mobileBaseL);
  setMotor(pwr[1], mobileBaseR);

  currT = millis();
  pos[0] = encLeft.read();
  pos[1] = encRight.read();

  // goal_left_msg.data = goal[0]*(2*M_PI*WHEEL_RAD)/60.0;
  // linear_vel_msg.data = ((velocity[0]+velocity[1])/2)*2*M_PI*WHEEL_RAD/60.0;
  // angular_vel_msg.data = ((velocity[1]-velocity[0])/WHEEL_DIST)*2*M_PI*WHEEL_RAD/60.0;
  
  // linear_vel_pub.publish(&linear_vel_msg);
  // angular_vel_pub.publish(&angular_vel_msg);
   //Calculate velocity from position of motor
  if(currT-prevT>50){
    velocity[0] = (posPrev[0]-pos[0])*1e3/((float) (currT-prevT));
    velocity[1] = (pos[1]-posPrev[1])*1e3/((float) (currT-prevT));
    //Not sure about this
    velocity[0] = (velocity[0] / TICKS_PER_REV)*60;
    velocity[1] = (velocity[1] / TICKS_PER_REV)*60;
    posPrev[0] = pos[0];
    posPrev[1] = pos[1];

    //Calculate PID of each wheel
    pwr[0] = PIDController_Update(&pidLeft, goal[0], velocity[0]);
    pwr[1] = PIDController_Update(&pidRight, goal[1], velocity[1]);

    if((goal[0] == 0.0) && (goal[1] == 0.0)){
      pwr[0] = 0.0;
      pwr[1] = 0.0;
    }

    setMotor(pwr[0], mobileBaseL);
    setMotor(pwr[1], mobileBaseR);
    prevT = currT;
  }

  nh.spinOnce();

}

void GoalCb(const geometry_msgs::Twist& twist_msg){

  setMotor(pwr[0], mobileBaseL);
  setMotor(pwr[1], mobileBaseR);
  
  float x = twist_msg.linear.x;
  float z = twist_msg.angular.z;

  float goal_left = x+z*WHEEL_DIST/2;
  float goal_right = x-z*WHEEL_DIST/2;
  // if(goal_left != goal[0] || goal_right != goal[1]){
  //   nh.loginfo("RECEIVE NEW GOAL!");
  // }

  goal_left = goal_left*60/(2*M_PI*WHEEL_RAD);
  goal_right = goal_right*60/(2*M_PI*WHEEL_RAD);
  goal[0] = goal_left;
  goal[1] = goal_right;

    //Set motor power
}

// void controlMotor()
// {
//   //Calculate velocity from position of motor
//   velocity[0] = (posPrev[0]-pos[0])*1e3/((float) CONTROL_INTERVAL_MS);
//   velocity[1] = (posPrev[1]-pos[1])*1e3/((float) CONTROL_INTERVAL_MS);
//   //Not sure about this
//   velocity[0] = (velocity[0] / TICKS_PER_REV)*60;
//   velocity[1] = (velocity[1] / TICKS_PER_REV)*60;
//   posPrev[0] = pos[0];
//   posPrev[1] = pos[1];

//   //Calculate PID of each wheel
//   pwr[0] = PIDController_Update(&pidLeft, goal[0], velocity[0]);
//   pwr[1] = PIDController_Update(&pidRight, goal[1], velocity[1]);

//   if((goal[0] == 0.0) && (goal[1] == 0.0)){
//     pwr[0] = 0.0;
//     pwr[1] = 0.0;
//   }

//   setMotor(pwr[0], mobileBaseL);
//   setMotor(pwr[1], mobileBaseR);

//   // pwr_left_msg.data = pwr[0];
//   // pwr_right_msg.data = pwr[1];

//   // pwr_left_pub.publish(&pwr_left_msg);
//   // pwr_right_pub.publish(&pwr_right_msg);

  

// }

void setMotor(int pwr, Servo servo)
{
  servo.write(90+pwr);

}