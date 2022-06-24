#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>

//Setting the timer used for interrupts
#define USE_TIMER_1     false
#define USE_TIMER_2     true
#define USE_TIMER_3     false
#define USE_TIMER_4     false
#define USE_TIMER_5     false

#define CONTROL_INTERVAL_MS 50L
#define PUB_PERIOD 100L

//Import encoder library and servo for motor control
#include <Encoder.h>
#include <Servo.h>

//Import ros-related libraries
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <SoftwareSerial.h>
#include <ros/time.h>
#include <math.h>

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



//Declare Servo objects for each wheels
Servo mobileBaseL, mobileBaseR;
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);


//Declare variables for odom calculation
double Vx = 0 ;
double Vy = 0 ;
double W = 0 ;
double x = 0.0;
double y = 0.0;
double th = 0.0;

int posPrev[WHEEL_NUM] = {0, 0};
int pos[WHEEL_NUM] = {0, 0}; 
int prevPosOdom[WHEEL_NUM] = {0, 0};
int deltaPos[WHEEL_NUM] = {0, 0};
float rad[WHEEL_NUM] = {0.0, 0.0}; 

float velocity[WHEEL_NUM] = {0.0, 0.0};
float goal[WHEEL_NUM] = {0.0, 0.0};
int pwr[WHEEL_NUM] = {0, 0};
float odom_pose[3] = {0.0, 0.0, 0.0};
float last_theta = 0.0;

//Time variables
unsigned long prevT = 0;
unsigned long currT = 0;


sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);
static char *joint_states_name[4] = {"left_back_wheel_joint", "right_back_wheel_joint", "realsense_yaw_joint", "realsense_roll_joint"};
float eff[WHEEL_NUM];

//Declare Function
void GoalCb(const geometry_msgs::Twist&);
void calOdom();
void calJointStates();
void publishState();
void controlMotor();
void setMotor(int ,Servo, Servo);

//ros publisher and transform broadcaster
ros::NodeHandle  nh;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", GoalCb );
