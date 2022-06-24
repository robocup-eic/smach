//Subscribe to topics that return the direction the camera needs to move.
#include <Arduino.h>

//Import Servo library
#include <Servo.h>

//Define pin of the servos
#define PITCH_PIN 5
#define YAW_PIN 8

//Limit of servo
#define YAW_UPPER_LIMIT 180
#define YAW_LOWER_LIMIT 0
#define PITCH_UPPER_LIMIT 180
#define PITCH_LOWER_LIMIT 0

//Import ros-related libraries
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

void CameraAdjustCb(const geometry_msgs::Vector3&);
void updateJointStates();
double deg_to_rad(double);

//Setup subscriber and publisher for joint states
sensor_msgs::JointState joint_states_msg;
ros::Publisher joint_states_pub("joint_states",&joint_states_msg);
ros::Subscriber<geometry_msgs::Vector3> servo_direction_sub("servo_dir",CameraAdjustCb);
ros::NodeHandle nh;
std_msgs::Float32 angle_msg;
ros::Publisher angle_pub("angle",&angle_msg);

static char *joint_states_name[2] = {(char*) "camera_pitch_joint", (char*) "camera_yaw_joint"};

//Setup for 2 servos
Servo camera_pitch_servo, camera_yaw_servo;
int current_pitch, current_yaw;

void setup() {
  // put your setup code here, to run once:
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(servo_direction_sub);
  nh.advertise(joint_states_pub);
  nh.advertise(angle_pub);

  joint_states_msg.name_length     = 2;
  joint_states_msg.position_length = 2;
  // joint_states_msg.velocity_length = 2;
  // joint_states_msg.effort_length   = 2;

  joint_states_msg.name = joint_states_name;
  joint_states_msg.header.frame_id = "base_link";

  camera_pitch_servo.attach(PITCH_PIN);
  camera_yaw_servo.attach(YAW_PIN);

  camera_pitch_servo.write(150);
  camera_yaw_servo.write(10);

}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}

double deg_to_rad(double deg){
  return deg*(M_PI/180.0);
}

void CameraAdjustCb(const geometry_msgs::Vector3& servo_dir){
  int y = (int) servo_dir.y;
  int z = (int) servo_dir.z;

  nh.loginfo("RECEIVING MSG");

  current_pitch = camera_pitch_servo.read();
  current_yaw = camera_yaw_servo.read();

  angle_msg.data = (float) current_pitch;

  current_pitch += y;
  current_yaw += z;

  if(current_pitch>PITCH_UPPER_LIMIT){
    current_pitch = PITCH_UPPER_LIMIT;
  }
  else if(current_pitch<PITCH_LOWER_LIMIT){
    current_pitch = PITCH_LOWER_LIMIT;
  }

  if(current_yaw>YAW_UPPER_LIMIT){
    current_yaw = YAW_UPPER_LIMIT;
  }
  else if(current_yaw<YAW_LOWER_LIMIT){
    current_yaw = YAW_LOWER_LIMIT;
  }

  camera_yaw_servo.write(current_yaw);
  camera_pitch_servo.write(current_pitch);

  angle_pub.publish(&angle_msg);

  updateJointStates();
}

void updateJointStates(){
  float pos[2] = {deg_to_rad((double) current_pitch), deg_to_rad((double) current_yaw)};
  
  joint_states_msg.position = pos;
  joint_states_msg.header.stamp = nh.now();
  joint_states_pub.publish(&joint_states_msg);
}