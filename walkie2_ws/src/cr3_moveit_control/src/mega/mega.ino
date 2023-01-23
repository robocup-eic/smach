//////////////////////////////////////////////////////////////////
/////////////////////////ros plugin///////////////////////////////
//////////////////////////////////////////////////////////////////
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
// platform.io require this
#include <Arduino.h>
#include <ezButton.h>
#include <Servo.h>
#define vmotor A0
int vmotor_value;
ezButton sw_down_done(14);
ezButton sw_up_done(7);
//state switch
int state_sw;
// The aformention connections
int in1 = 8;
int in2 = 9;
int enA = 10;
// servo control real sense
Servo realsense_pitch;
Servo realsense_yaw;
int realsense_pitch_array[10] = {55, 55, 55, 55, 55, 55, 55, 55, 55, 55};
int realsense_pitch_array_size = sizeof(realsense_pitch_array) / sizeof(realsense_pitch_array[0]);
int realsense_yaw_array[10] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100};
int realsense_yaw_array_size = sizeof(realsense_yaw_array)/sizeof(realsense_yaw_array[0]);
// time count
unsigned long PreviousTime=0, Time=0;
//NodeHandle
ros::NodeHandle nh;
//lift Declare variable publisher
std_msgs::Bool done;
ros::Publisher pub_lift_done("done", &done);
std_msgs::Float32 lift_state;
ros::Publisher pub_lift_state("lift_state", &lift_state);
std_msgs::Bool sw2;
ros::Publisher pub_sw2("sw2", &sw2);
std_msgs::Bool sw1;
ros::Publisher pub_sw1("sw1", &sw1);
std_msgs::Int16 realsense_pitch_angle;
ros::Publisher pub_realsense_pitch_angle("realsense_pitch_angle", &realsense_pitch_angle);
std_msgs::Int16 realsense_yaw_angle;
ros::Publisher pub_realsense_yaw_angle("realsense_yaw_angle", &realsense_yaw_angle);
void lift_cb( const std_msgs::Bool& lift)
{
  if(lift.data){
    while(true)
     {
      Time = millis() / 1000;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, 180);

      done.data = false;
      pub_lift_done.publish(&done); //1

      lift_state.data = 0.24;
      pub_lift_state.publish(&lift_state); //2

      sw_down_done.loop();
      sw1.data = ! sw_down_done.getState();
      pub_sw1.publish(&sw1); //3
      
      sw_up_done.loop();
      state_sw = ! sw_up_done.getState();
      sw2.data = state_sw;
      pub_sw2.publish(&sw2); //4
      
      realsense_pitch_angle.data = realsense_pitch.read() - 55;
      pub_realsense_pitch_angle.publish(&realsense_pitch_angle); //5

      realsense_yaw_angle.data = realsense_yaw.read() - 100;
      pub_realsense_yaw_angle.publish(&realsense_yaw_angle); //6

      nh.spinOnce();
      delay(1);

      if(state_sw)
      {
        PreviousTime = Time;
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        
        done.data = true;
        pub_lift_done.publish(&done); //1

        lift_state.data = 0.24;
        pub_lift_state.publish(&lift_state); //2

        sw_down_done.loop();
        sw1.data = ! sw_down_done.getState();
        pub_sw1.publish(&sw1); //3

        sw_up_done.loop();
        sw2.data = state_sw;
        pub_sw2.publish(&sw2); //4

        realsense_pitch_angle.data = realsense_pitch.read() - 55;
        pub_realsense_pitch_angle.publish(&realsense_pitch_angle); //5

        realsense_yaw_angle.data = realsense_yaw.read() - 100;
        pub_realsense_yaw_angle.publish(&realsense_yaw_angle); //6

        nh.spinOnce();
        
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(enA, 27);
        delay(200);
        nh.spinOnce();
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, 27);
        
        break;
      }
     }
  }
  else
  {
    while(true)
     {
      Time = millis() / 1000;
      done.data = false;
      pub_lift_done.publish(&done); //1;

      lift_state.data = 0.0;
      pub_lift_state.publish(&lift_state); //2

      sw_down_done.loop();
      state_sw = ! sw_down_done.getState();
      sw1.data = state_sw;
      pub_sw1.publish(&sw1); //3

      sw_up_done.loop();
      sw2.data = ! sw_up_done.getState();
      pub_sw2.publish(&sw2); //4

      realsense_pitch_angle.data = realsense_pitch.read() - 55;
      pub_realsense_pitch_angle.publish(&realsense_pitch_angle); //5
      
      realsense_yaw_angle.data = realsense_yaw.read() - 100;
      pub_realsense_yaw_angle.publish(&realsense_yaw_angle); //6

      nh.spinOnce();
      delay(1);

     vmotor_value = analogRead(vmotor);

     if(vmotor_value > 250)
     {
       digitalWrite(in1, HIGH);
       digitalWrite(in2, LOW);
       analogWrite(enA, 5);
     }
     else if(vmotor_value < 10)
     {
       digitalWrite(in1, LOW);
       digitalWrite(in2, HIGH);
       analogWrite(enA, 50);
     }
     else
     {
       digitalWrite(in1, LOW);
       digitalWrite(in2, LOW);
     }

      if(state_sw)
      {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);

        done.data = true;
        pub_lift_done.publish(&done); //1

        lift_state.data = 0.0;
        pub_lift_state.publish(&lift_state); //2
        
        sw_down_done.loop();
        state_sw = ! sw_down_done.getState();
        sw1.data = state_sw;
        pub_sw1.publish(&sw1); //3

        sw_up_done.loop();
        sw2.data = ! sw_up_done.getState();
        pub_sw2.publish(&sw2); //4

        realsense_pitch_angle.data = realsense_pitch.read() - 55;
        pub_realsense_pitch_angle.publish(&realsense_pitch_angle); //5
      
        realsense_yaw_angle.data = realsense_yaw.read() - 100;
        pub_realsense_yaw_angle.publish(&realsense_yaw_angle); //6
        delay(1);
        nh.spinOnce();
        break;
      }
     }
   }
}
void realsense_pitch_relative_cb( const std_msgs::Int16& realsense_pitch_relative_control)
{
  int delta_deg = realsense_pitch_relative_control.data;
  int deg = realsense_pitch.read();
  int angle = delta_deg + deg;
  if (angle > 180)
  {
    angle = 180;
  }
  else if(angle < 0)
  {
    angle = 0 ;
  }
  /////////////////// filter /////////////////////////////////////
  int sum = 0;
  for(int i=0;i<realsense_pitch_array_size - 1;i++)
  {
    realsense_pitch_array[i] = realsense_pitch_array[i+1];
  }
  realsense_pitch_array[realsense_pitch_array_size-1] = angle;
  for(int i=0;i<realsense_pitch_array_size;i++)
  {
    sum += realsense_pitch_array[i];
  }
  angle = sum / realsense_pitch_array_size;
  ////////////////////////////////////////////////////////////////
  realsense_pitch.write(angle);
  realsense_pitch_angle.data = realsense_pitch.read() - 55;
  pub_realsense_pitch_angle.publish(&realsense_pitch_angle);
  delay(10);
}
void realsense_pitch_absolute_cb( const std_msgs::Int16& realsense_pitch_absolute_control)
{
  int angle = realsense_pitch_absolute_control.data + 55;
  if (angle > 180)
  {
    angle = 180;
  }
  else if(angle < 0)
  {
    angle = 0 ;
  }
  realsense_pitch.write(angle);
  realsense_pitch_angle.data = realsense_pitch.read() - 55;
  pub_realsense_pitch_angle.publish(&realsense_pitch_angle);
  for(int i=0;i<realsense_pitch_array_size;i++)
  {
    realsense_pitch_array[i] = realsense_pitch.read();
  }
}
void realsense_yaw_relative_cb( const std_msgs::Int16& realsense_yaw_relative_control)
{
  int delta_deg = realsense_yaw_relative_control.data;
  int deg = realsense_yaw.read();
  int angle = delta_deg + deg;
  if (angle > 180)
  {
    angle = 180;
  }
  else if(angle < 0)
  {
    angle = 0 ;
  }
  /////////////////// filter /////////////////////////////////////
  int sum = 0;
  for(int i=0;i<realsense_yaw_array_size - 1;i++)
  {
    realsense_yaw_array[i] = realsense_yaw_array[i+1];
  }
  realsense_yaw_array[realsense_yaw_array_size-1] = angle;
  for(int i=0;i<realsense_yaw_array_size;i++)
  {
    sum += realsense_yaw_array[i];
  }
  angle = sum / realsense_yaw_array_size;
  ////////////////////////////////////////////////////////////////
  realsense_yaw.write(angle);
  realsense_yaw_angle.data = realsense_yaw.read() - 100;
  pub_realsense_yaw_angle.publish(&realsense_yaw_angle);
  delay(10);
}
void realsense_yaw_absolute_cb( const std_msgs::Int16& realsense_yaw_absolute_control)
{
  int angle = realsense_yaw_absolute_control.data + 100;
  if (angle > 180)
  {
    angle = 180;
  }
  else if(angle < 0)
  {
    angle = 0 ;
  }
  realsense_yaw.write(angle);
  realsense_yaw_angle.data = realsense_yaw.read() - 100;
  pub_realsense_yaw_angle.publish(&realsense_yaw_angle);
  for(int i=0;i<realsense_yaw_array_size;i++)
  {
    realsense_yaw_array[i] = realsense_yaw.read();
  }
}
//Defining Subscriber in a topic called lift_command
ros::Subscriber<std_msgs::Bool> lift_command("lift_command", &lift_cb);
//Defining Subscriber in a topic calle realsense_pitch_relative_command
ros::Subscriber<std_msgs::Int16> realsense_pitch_relative_command("realsense_pitch_relative_command", &realsense_pitch_relative_cb);
//Defining Subscriber in a topic calle realsense_pitch_absolute_command
ros::Subscriber<std_msgs::Int16> realsense_pitch_absolute_command("realsense_pitch_absolute_command", &realsense_pitch_absolute_cb);
//Defining Subscriber in a topic calle realsense_yaw_relative_command
ros::Subscriber<std_msgs::Int16> realsense_yaw_relative_command("realsense_yaw_relative_command", &realsense_yaw_relative_cb);
//Defining Subscriber in a topic calle realsense_yaw_absolute_command
ros::Subscriber<std_msgs::Int16> realsense_yaw_absolute_command("realsense_yaw_absolute_command", &realsense_yaw_absolute_cb);
void home_lift_walkie()
{
  while(true)
     {
      Time = millis() / 1000;
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, 5);
      delay(50);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 100);
      delay(25);
      sw_down_done.loop();
      state_sw = ! sw_down_done.getState();
      lift_state.data = 0.0;
      if(state_sw)
      {
        PreviousTime = Time;
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        delay(100);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(enA, 100);
        delay(200);
        analogWrite(enA, 40);
        break;
      }
     }
}
void setup() {
  //////////////////////////////////////////////////////////////////
  /////////////////////////ros node init////////////////////////////
  //////////////////////////////////////////////////////////////////
  nh.initNode();
  nh.advertise(pub_lift_done);
  nh.advertise(pub_lift_state);
  nh.advertise(pub_sw1);
  nh.advertise(pub_sw2);
  nh.advertise(pub_realsense_pitch_angle);
  nh.advertise(pub_realsense_yaw_angle);
  nh.subscribe(lift_command);
  nh.subscribe(realsense_pitch_relative_command);
  nh.subscribe(realsense_pitch_absolute_command);
  nh.subscribe(realsense_yaw_relative_command);
  nh.subscribe(realsense_yaw_absolute_command);
  //////////////////////////////////////////////////////////////////
  ///////////////////////Arduino and switch/////////////////////////
  //////////////////////////////////////////////////////////////////
  // Set used pins as output
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(vmotor, INPUT);
  sw_down_done.setDebounceTime(50);
  sw_up_done.setDebounceTime(50);
  // Making sure I don't get my beard tangled
  // in the motor when I'm hanging over it
  // plugging it in AKA motor off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  //realsense_pitch
  realsense_pitch.attach(4);
  //realsense_yaw
  realsense_yaw.attach(5);
  //publish
  done.data = true;
  pub_lift_done.publish(&done);
  nh.spinOnce();
//  home_lift_walkie();
//  lift_state.data = 0.465;
  lift_state.data = 0.0;
  realsense_pitch.write(55);
  realsense_yaw.write(100);
//  Serial.begin(9600);
}
void loop() {
  //////////////////////////////////////////////////////////////////
  /////////////////////////ros node init////////////////////////////
  //////////////////////////////////////////////////////////////////
  //publish
  done.data = true;
  pub_lift_done.publish(&done);
  sw_down_done.loop();
  state_sw = ! sw_down_done.getState();
  sw1.data = state_sw;
  pub_sw1.publish(&sw1);
  sw_up_done.loop();
  state_sw = ! sw_up_done.getState();
  sw2.data = state_sw;
  pub_sw2.publish(&sw2);
  realsense_pitch_angle.data = realsense_pitch.read() - 55;
  pub_realsense_pitch_angle.publish(&realsense_pitch_angle);
  realsense_yaw_angle.data = realsense_yaw.read() - 100;
  pub_realsense_yaw_angle.publish(&realsense_yaw_angle);
  pub_lift_state.publish(&lift_state);
  vmotor_value = analogRead(vmotor);
//  Serial.println(vmotor_value);
  delay(3);
  nh.spinOnce();
}
