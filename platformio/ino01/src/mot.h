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

class single_wheel{
public:
  int mot_a_;
  int mot_b_;
  int mot_p_;
  int enc_a_;
  int enc_b_;

  long position_;  
  long last_position_;  
  int velocity_;  

  float command_;
  float integral_;
  float max_integral_;

  float hz_;

  single_wheel(int mot_a, int mot_b, int mot_p, int enc_a, int enc_b){
    mot_a_=mot_a;
    mot_b_=mot_b;
    mot_p_=mot_p;
    enc_a_=enc_a;
    enc_b_=enc_b;

    pinMode(enc_a_, INPUT_PULLUP);
    pinMode(enc_b_, INPUT_PULLUP);

    position_=0;
    last_position_=0;
    velocity_=0;

    integral_=0.0;
    max_integral_=200.0;
    hz_=20;
  }

  void set_command(float command){
    command_=command;
  }

  void set_pwm(int value){
    if(value<-255){
      digitalWrite(mot_a_, LOW);
      digitalWrite(mot_b_, HIGH);
      analogWrite(mot_p_,255);  
    }
    else if(value<0){
      digitalWrite(mot_a_, LOW);
      digitalWrite(mot_b_, HIGH);
      analogWrite(mot_p_,-value);  
    }
    else if(value<=255){
      digitalWrite(mot_a_, HIGH);
      digitalWrite(mot_b_, LOW);
      analogWrite(mot_p_,value);
    }
    else{
      digitalWrite(mot_a_, HIGH);
      digitalWrite(mot_b_, LOW);
      analogWrite(mot_p_,255);
    }
  }

  void enc_int(){
    bool sa=digitalRead(enc_a_);
    bool sb=digitalRead(enc_b_);
    if(sa==sb)position_++;
    else position_--;
  }

  void tick(){
    float speed=(position_-last_position_)*hz_;
    velocity_=speed;
    last_position_=position_;
    float delta=speed-command_;
    integral_+=delta/hz_;
    if(integral_<-max_integral_)integral_=-max_integral_;
    if(integral_> max_integral_)integral_= max_integral_;
    if(command_==0 && speed==0)integral_/=2.0;
    float out=-0.3*(delta+5*integral_);
    set_pwm((int)out);
  }
};

