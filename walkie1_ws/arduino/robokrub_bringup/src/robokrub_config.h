#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>

// //Setting the timer used for interrupts
// #define USE_TIMER_1     false
// #define USE_TIMER_2     true
// #define USE_TIMER_3     false
// #define USE_TIMER_4     false
// #define USE_TIMER_5     false

#define CONTROL_INTERVAL_MS 50L
#define PUB_PERIOD 50L

//Declare PID variables
extern "C"
{
#include <PID.h>
}

#define PID_KP_LEFT 3.0f
#define PID_KI_LEFT 2.0f
#define PID_KD_LEFT 0.0f

#define PID_KP_RIGHT 3.0f
#define PID_KI_RIGHT 2.0f
#define PID_KD_RIGHT 0.0f

#define PID_TAU 0.05f

#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX 255.0f

#define PID_LIM_MIN_INT -125.0f
#define PID_LIM_MAX_INT 125.0f

#define SAMPLE_TIME_S 0.05f

//Import encoder library and servo for motor control
#include <Encoder.h>
#include <Servo.h>

//Import ros-related libraries
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
// #include <SoftwareSerial.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

//Define Wheel Constant
#define WHEEL_NUM 2
#define WHEEL_RAD 0.125
#define WHEEL_DIST 0.547

//Define Encoders pin (need to be interrupt pins)
#define ENCA1 2 
#define ENCA2 3

#define ENCB1 17
#define ENCB2 18

//Define ticks per revolution for converting ticks to rev
#define TICKS_PER_REV 1600

//Declare pin for motors
#define PWM_L 8
#define INA_L 12
#define INB_L 13

#define PWM_R 5
#define INA_R 6
#define INB_R 7

//Define pin of the servos
#define PITCH_PIN 9
#define YAW_PIN 10

//Limit of servo
#define YAW_UPPER_LIMIT 180
#define YAW_LOWER_LIMIT 0
#define PITCH_UPPER_LIMIT 180
#define PITCH_LOWER_LIMIT 0
#define YAW_INIT_STATE 10
#define PITCH_INIT_STATE 150



//Declare Servo objects for each wheels
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);
PIDController pidLeft = {PID_KP_LEFT,
                     PID_KI_LEFT, PID_KD_LEFT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
PIDController pidRight = {PID_KP_RIGHT,
                     PID_KI_RIGHT, PID_KD_RIGHT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
//Declare Servo objects for the camera
//Setup for 2 servos
Servo camera_pitch_servo, camera_yaw_servo;
int current_pitch, current_yaw;

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

unsigned long prevT_state = 0;
unsigned long currT_state = 0;


sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("/walkie/joint_states", &joint_states);
// ros::Subscriber<geometry_msgs::Vector3> servo_direction_sub("servo_dir",CameraAdjustCb);



float eff[WHEEL_NUM];

//Declare Function
void GoalCb(const geometry_msgs::Twist&);
void calOdom();
void calJointStates();
void publishState();
void setMotor(int ,int, int, int);
void CameraAdjustCb(const geometry_msgs::Vector3&);
double deg_to_rad(double);

//ros publisher and transform broadcaster
ros::NodeHandle  nh;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("/walkie/odom", &odom);
tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/walkie/cmd_vel", GoalCb );


std_msgs::Float32 vl;
ros::Publisher vl_pub("/walkie/vel_left", &vl);
std_msgs::Float32 vr;
ros::Publisher vr_pub("/walkie/vel_right", &vr);

float vleft = 0.0;
float vright = 0.0;

//debug msg
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug",&debug_msg);

//String name
char odom_child_frame[30] = "base_footprint";
char odom_header_frame[5] = "odom";

char base_link_name[10] = "base_link";

static float joint_states_pos[4] = {0.0, 0.0, 0.0, 0.0};
static float joint_states_vel[4] = {0.0, 0.0, 0.0, 0.0};

unsigned long callback_time = 0;
