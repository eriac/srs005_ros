#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

//import
//#include <MsTimer2.h>

//wheel
#define MOT0_A 32
#define MOT0_B 33
#define MOT0_P 5
#define ENC0_A 2
#define ENC0_B 22

#define MOT1_A 34
#define MOT1_B 35
#define MOT1_P 6
#define ENC1_A 3
#define ENC1_B 23

#define MOT2_A 36
#define MOT2_B 37
#define MOT2_P 7
#define ENC2_A 18
#define ENC2_B 24

#define LED0 9
#define LED1 10
#define LEDL 13
#define SW0 48
#define SW1 49

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";
char tick[13] = "tick!";

std_msgs::Int32 int_msg;
ros::Publisher odom0("odom0", &int_msg);

void led_cb(const std_msgs::Bool& msg){
  if(msg.data)digitalWrite(13, HIGH);
  else digitalWrite(13, LOW);
}

void set_wheel0(int value){
  if(value>=0){
    digitalWrite(MOT0_A, HIGH);
    digitalWrite(MOT0_B, LOW);
    analogWrite(MOT0_P,value);  
  }
  else{
    digitalWrite(MOT0_A, LOW);
    digitalWrite(MOT0_B, HIGH);
    analogWrite(MOT0_P,-value);  
  }
}
volatile float cmd0=0.0;
void wheel0_cb(const std_msgs::Float32& msg){
  //set_wheel0(msg.data);
  cmd0=msg.data;
}
ros::Subscriber<std_msgs::Bool> sub0("led", &led_cb);
ros::Subscriber<std_msgs::Float32> sub_wheel0("wheel0", &wheel0_cb);

volatile int count_odom0=0;
void wheel0_int() {
  bool sa=digitalRead(ENC0_A);
  bool sb=digitalRead(ENC0_B);
  if(sa==sb)count_odom0++;
  else count_odom0--;
}


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.advertise(odom0);
  nh.subscribe(sub0);
  nh.subscribe(sub_wheel0);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(13, OUTPUT);

  //wheel
  pinMode(MOT0_A, OUTPUT);
  pinMode(MOT0_B, OUTPUT);
  pinMode(MOT0_P, OUTPUT);
  pinMode(ENC0_A, INPUT_PULLUP);
  pinMode(ENC0_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC0_A), wheel0_int, CHANGE);
}

void loop()
{
  static int last_odom0;
  float state0=(count_odom0-last_odom0)*20;
  int_msg.data = state0;
  last_odom0=count_odom0;
  odom0.publish( &int_msg );

  //control
  float delta=state0-cmd0;
  static float integrate=0;
  integrate+=delta/20.0;
  float out=-0.2*(delta+10*integrate);
  if(out>250)out=250;
  else if(out<-250)out=-250;
  set_wheel0((int)out);

  nh.spinOnce();
  delay(50);
}
