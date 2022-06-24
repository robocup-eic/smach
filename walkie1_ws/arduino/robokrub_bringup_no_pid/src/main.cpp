#include <Arduino.h>
#include "robokrub_config.h"
#include <TimerInterrupt.h>
#include <ISR_Timer.h>

void setup() {  
  
  //Set up publisher, subscriber and broadcaster for ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.subscribe(cmd_vel_sub);
  odom_broadcaster.init(nh);
  joint_states.header.frame_id = "base_link";
  joint_states.name = joint_states_name;

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

  

  // if (ITimer2.attachInterruptInterval(PUB_PERIOD, publishState))
  // {
  //   nh.loginfo("Starting ITimer3 OK");
  // }
  // else
  //    nh.loginfo("Can't set interrupt on ITimer3");
 
}
void loop() {

  currT = millis();

  pos[0] = encLeft.read();
  pos[1] = encRight.read();
  
  Vx=velocity[0]+(velocity[1]-velocity[0])/2;
  W=velocity[0]/((velocity[0]*WHEEL_DIST)/(velocity[0]+velocity[1]));

  if(currT-prevT>PUB_PERIOD){
    publishState();
  }


  nh.spinOnce();
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

  pwr[0] = 0; //SOmething
  pwr[1] = 0; //Something

  //Set motor power
  setMotor(pwr[0], mobileBaseL);
  setMotor(pwr[1], mobileBaseR);

}

void setMotor(int pwr, Servo servo)
{
  servo.write(pwr);

}

void GoalCb(const geometry_msgs::Twist& twist_msg){
  
  float x = twist_msg.linear.x;
  float z = twist_msg.angular.z;

  goal[0] = x-z*WHEEL_DIST/2;
  goal[1] = x+z*WHEEL_DIST/2;
}

void calOdom(){
  
    float delta_s = WHEEL_RAD * (rad[0]+rad[1])/2.0;
    float theta = WHEEL_RAD * (rad[1]-rad[0])/WHEEL_DIST;

    float delta_theta = theta-last_theta;
    
    odom_pose[0] = delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] = delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] = delta_theta;
    
    
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = odom_pose[0];
    odom_trans.transform.translation.y = odom_pose[1];
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionFromYaw(odom_pose[2]); ;
    odom_trans.header.stamp = nh.now();

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.angular.z = W;
}
void calJointStates(){
    rad[0] = deltaPos[0]*2*M_PI/TICKS_PER_REV;
    rad[1] = deltaPos[1]*2*M_PI/TICKS_PER_REV;

    for (int i = 0; i < 2; ++i)
      joint_states.position[i] += rad[i];
    joint_states.velocity = velocity;
    joint_states.header.stamp=nh.now();
}
void publishState(){
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