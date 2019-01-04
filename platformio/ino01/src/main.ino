#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "mot.h"
single_wheel wheel0(MOT0_A, MOT0_B, MOT0_P, ENC0_A, ENC0_B);
single_wheel wheel1(MOT1_A, MOT1_B, MOT1_P, ENC1_A, ENC1_B);
single_wheel wheel2(MOT2_A, MOT2_B, MOT2_P, ENC2_A, ENC2_B);

ros::NodeHandle nh;
std_msgs::String str_msg;


std_msgs::Int32 int_msg;
ros::Publisher wheel0_position("wheel0/position", &int_msg);
ros::Publisher wheel1_position("wheel1/position", &int_msg);
ros::Publisher wheel2_position("wheel2/position", &int_msg);
ros::Publisher wheel0_velocity("wheel0/velocity", &int_msg);
ros::Publisher wheel1_velocity("wheel1/velocity", &int_msg);
ros::Publisher wheel2_velocity("wheel2/velocity", &int_msg);

void wheel0_cb(const std_msgs::Float32& msg){
  wheel0.set_command(msg.data);
}
ros::Subscriber<std_msgs::Float32> wheel0_command("wheel0/command", &wheel0_cb);

void wheel1_cb(const std_msgs::Float32& msg){
  wheel1.set_command(msg.data);
}
ros::Subscriber<std_msgs::Float32> wheel1_command("wheel1/command", &wheel1_cb);

void wheel2_cb(const std_msgs::Float32& msg){
  wheel2.set_command(msg.data);
}
ros::Subscriber<std_msgs::Float32> wheel2_command("wheel2/command", &wheel2_cb);

void wheel0_int(){
  wheel0.enc_int();
}
void wheel1_int(){
  wheel1.enc_int();
}
void wheel2_int(){
  wheel2.enc_int();
}

void setup()
{
  nh.initNode();
  nh.advertise(wheel0_position);
  nh.advertise(wheel1_position);
  nh.advertise(wheel2_position);

  nh.advertise(wheel0_velocity);
  nh.advertise(wheel1_velocity);
  nh.advertise(wheel2_velocity);

  nh.subscribe(wheel0_command);
  nh.subscribe(wheel1_command);
  nh.subscribe(wheel2_command);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(13, OUTPUT);

  //wheel
  attachInterrupt(digitalPinToInterrupt(wheel0.enc_a_), wheel0_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(wheel1.enc_a_), wheel1_int, CHANGE);
  attachInterrupt(digitalPinToInterrupt(wheel2.enc_a_), wheel2_int, CHANGE);
}

void loop()
{
  int_msg.data = wheel0.position_;
  wheel0_position.publish( &int_msg );
  int_msg.data = wheel0.velocity_;
  wheel0_velocity.publish( &int_msg );

  int_msg.data = wheel1.position_;
  wheel1_position.publish( &int_msg );
  int_msg.data = wheel1.velocity_;
  wheel1_velocity.publish( &int_msg );

  int_msg.data = wheel2.position_;
  wheel2_position.publish( &int_msg );
  int_msg.data = wheel2.velocity_;
  wheel2_velocity.publish( &int_msg );

  //control
  wheel0.tick();
  wheel1.tick();
  wheel2.tick();

  nh.spinOnce();
  delay(50);
}
