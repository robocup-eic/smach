#include <Arduino.h>
#include "no_odom_cfg.h"

void setup() {

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  // Encoder publisher
  nh.advertise(raw_vel_pub);
  nh.advertise(raw_pos_pub);

  nh.subscribe(cmd_vel_sub);

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

void loop() {

  setMotor(pwr[0], INA_L, INB_L, PWM_L);
  setMotor(pwr[1], INA_R, INB_R, PWM_R);
    currT = millis();
  pos[0] = encLeft.read();
  pos[1] = encRight.read();

  // Publish velocity and position
  if(currT-prevT > 50){
    velocity[0] = (pos[0]-posPrev[0])*1e3/((float) (currT-prevT));
    velocity[1] = (posPrev[1]-pos[1])*1e3/((float) (currT-prevT));

    velocity[0] = (velocity[0] / TICKS_PER_REV)*60;
    velocity[1] = (velocity[1] / TICKS_PER_REV)*60;

    posPrev[0] = pos[0];
    posPrev[1] = pos[1];

    //Calculate PID of each wheel
    pwr[0] = PIDController_Update(&pidLeft, goal[0], velocity[0]);
    pwr[1] = PIDController_Update(&pidRight, goal[1], velocity[1]);

    //Convert from rpm to m/s
    velocity[0] = velocity[0]*2*M_PI*WHEEL_RAD/60.0;
    velocity[1] = velocity[1]*2*M_PI*WHEEL_RAD/60.0;

    //Publish raw position from encoder
    raw_pos.data_length = 2;
    raw_pos.data = pos;
    raw_pos_pub.publish(&raw_pos);
   
    //Publish velocity
    raw_vel.data_length =2;
    raw_vel.data = velocity;
    raw_vel_pub.publish(&raw_vel);

    if((goal[0] == 0.0) && (goal[1]==0)){
      pwr[0] = 0.0;
      pwr[1] = 0.0;
    }

    prevT = currT;
  }

  if(currT-callback_time>5000){
    goal[0] = 0.0;
    goal[1] = 0.0;

  }
  nh.spinOnce();
}