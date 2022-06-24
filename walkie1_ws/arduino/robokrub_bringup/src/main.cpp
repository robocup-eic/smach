#include <Arduino.h>
#include "robokrub_config.h"
// #include <TimerInterrupt.h>
// #include <ISR_Timer.h>

void setup() {  
  
  //Set up publisher, subscriber and broadcaster for ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.subscribe(cmd_vel_sub);
  odom_broadcaster.init(nh);
  static char *joint_states_name[] = {(char*)"realsense_yaw_joint", (char*)"realsense_pitch_joint",(char*)"left_back_wheel_joint", (char*)"right_back_wheel_joint"};

  joint_states.header.frame_id = base_link_name;
  joint_states.name = joint_states_name;

  joint_states.name_length     = 4;
  joint_states.position_length = 4;
  joint_states.velocity_length = 4;
  joint_states.effort_length   = 4;

  pinMode(INA_L, OUTPUT);
  pinMode(INA_R, OUTPUT);
  pinMode(INB_L, OUTPUT);
  pinMode(INB_R, OUTPUT);

  pinMode(PWM_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);


  //Initiate PID controller
  PIDController_Init(&pidLeft);
  PIDController_Init(&pidRight);
 
}
void loop() {

 // put your main code here, to run repeatedly:
  setMotor(pwr[0], INA_L, INB_L, PWM_L);
  setMotor(pwr[1], INA_R, INB_R, PWM_R);

  currT = millis();

  pos[0] = encLeft.read();
  pos[1] = encRight.read();

  if(currT-prevT>50){

    velocity[0] = (pos[0]-posPrev[0])*1e3/((float) (currT-prevT));
    velocity[1] = (posPrev[1]-pos[1])*1e3/((float) (currT-prevT));

    velocity[0] = (velocity[0] / TICKS_PER_REV)*60;
    velocity[1] = (velocity[1] / TICKS_PER_REV)*60;
    
    posPrev[0] = pos[0];
    posPrev[1] = pos[1];

    //Calculate PID of each wheel
    pwr[0] = PIDController_Update(&pidLeft, goal[0], velocity[0]);
    pwr[1] = PIDController_Update(&pidRight, goal[1], velocity[1]);

    //Convert Velocity to m/s
    velocity[0] = velocity[0]*2*M_PI*WHEEL_RAD/60.0;
    velocity[1] = velocity[1]*2*M_PI*WHEEL_RAD/60.0;

    // //Publish vel left and vel right
    

    if((goal[0] == 0.0) && (goal[1]==0)){
      pwr[0] = 0.0;
      pwr[1] = 0.0;
    }
    prevT = currT;

    
  }

  //Calculate velocity for the robot based on velocity of left and right wheel
  Vx=((velocity[0]+velocity[1])/2);
  W=((velocity[1]-velocity[0])/WHEEL_DIST);

  // if(currT-prevT_state>50L){
  //   publishState();
  //   prevT_state = currT;
  // }
  if(currT-callback_time>5000){
    goal[0] = 0.0;
    goal[1] = 0.0;
  }

  nh.spinOnce();
}

//Function for commanding motor
void setMotor(int pwr, int digiPin1, int digiPin2, int analogPin)
{
  if(pwr<0){
    digitalWrite(digiPin1,HIGH);
    digitalWrite(digiPin2,LOW);
    analogWrite(analogPin,-pwr);
  }
  else{
    digitalWrite(digiPin1,LOW);
    digitalWrite(digiPin2,HIGH);
    analogWrite(analogPin,pwr);
  }
}

//Callback for cmd_vel subscription
void GoalCb(const geometry_msgs::Twist& twist_msg){

  setMotor(pwr[0], INA_L, INB_L, PWM_L);
  setMotor(pwr[1], INA_R, INB_R, PWM_R);

  callback_time = millis();
  
  float x = twist_msg.linear.x;
  float z = twist_msg.angular.z;

  //Calculate goal_veclocity(rpm) fro both left and right wheel
  float goal_left = x-z*WHEEL_DIST/2;
  float goal_right = x+z*WHEEL_DIST/2;

  goal_left = goal_left*60.0/(2*M_PI*WHEEL_RAD);
  goal_right = goal_right*60.0/(2*M_PI*WHEEL_RAD);

  // if(goal_right>0){
  //   goal_right -= 0.9;
  // }
  // else{
  //   goal_right += 0.9;
  // }


  //Limit the maximum speed of the robot
  if(goal_left>80.0){
    goal_left = 80.0;
  }
  else if(goal_left<-80.0){
    goal_left = -80.0;
  }

  if(goal_right>80.0){
    goal_right=80.0;
  }
  else if(goal_right<-80.0){
    goal_right=-80.0;
  }

  //Assign it to the array variable
  goal[0] = goal_left;
  goal[1] = goal_right;
}

//Function for calculating odometry
void calOdom(){

    setMotor(pwr[0], INA_L, INB_L, PWM_L);
    setMotor(pwr[1], INA_R, INB_R, PWM_R);


    odom.header.frame_id = odom_header_frame;
    odom.child_frame_id = odom_child_frame;   

      
    odom_trans.header.frame_id = odom_header_frame;    
    odom_trans.child_frame_id = odom_child_frame;   
  
    float delta_s = WHEEL_RAD * (rad[0]+rad[1])/2.0;
    float theta = WHEEL_RAD * (rad[1]-rad[0])/WHEEL_DIST;

    float delta_theta = theta-last_theta;
    
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;
    
    
    
    odom_trans.transform.translation.x = odom_pose[0];
    odom_trans.transform.translation.y = odom_pose[1];
    odom_trans.transform.translation.z = 0.4021;
    odom_trans.transform.rotation = tf::createQuaternionFromYaw(odom_pose[2]); 
    odom_trans.header.stamp = nh.now();

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0.4021;
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.header.stamp = nh.now();

    //odom.twist.twist.linear.x = velocity[0];
    //odom.twist.twist.angular.z = velocity[1];

    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.angular.z = W;

    
   
}

//Function for calculating joint states
void calJointStates(){

    setMotor(pwr[0], INA_L, INB_L, PWM_L);
    setMotor(pwr[1], INA_R, INB_R, PWM_R);
    rad[0] = deltaPos[0]*2*M_PI/TICKS_PER_REV;
    rad[1] = deltaPos[1]*2*M_PI/TICKS_PER_REV;

    joint_states_pos[2] += rad[0];
    joint_states_pos[3] += rad[1];

    joint_states_vel[2] = velocity[0];
    joint_states_vel[3] = velocity[1];

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;

    joint_states.header.stamp=nh.now();

    
}

//Function for publishing odometry and joint states
void publishState(){

  setMotor(pwr[0], INA_L, INB_L, PWM_L);
  setMotor(pwr[1], INA_R, INB_R, PWM_R);

  deltaPos[0] = pos[0]-prevPosOdom[0];
  deltaPos[1] = pos[1]-prevPosOdom[1];
  prevPosOdom[0] = pos[0];
  prevPosOdom[1] = pos[1];

  calJointStates();
  calOdom();

  odom_broadcaster.sendTransform(odom_trans);
  odom_pub.publish(&odom);

  joint_states_pub.publish(&joint_states);
  
}