
#include <ros.h>
#include <geometry_msgs/Twist.h>


#define encoder0PinA 18      // encoder 1
#define encoder0PinB 19

#define encoder1PinA 17     // encoder 2
#define encoder1PinB 5




const uint8_t R_PWM =  12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;

const uint8_t L_PWM =  33;
const uint8_t L_BACK = 25;
const uint8_t L_FORW = 26;

const uint8_t channel_R= 1;
const uint8_t channel_L= 0;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

const int freq = 5000;
const int res = 8;
int dirRight,dirLeft;

long prevT = 0;
int posPrevRight = 0,posPrevLeft = 0;
int pos = 0;

long currT = 0;
float deltaT = 0;
float velocity1 = 0,velocity2 = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;


float kp = 1.5;
float kd = 0;
float ki = 8;

float errorRight = 0,errorLeft = 0,targetRight= 0,targetLeft = 0;
float eintegralRight = 0, eintegralLeft= 0;
float pid_outRight = 0, pid_outLeft = 0;
float pwrRight = 0,pwrLeft = 0;


int ppr = 3000;
float wheelDia = 0.044;  //in m
float circumference = wheelDia * 3.142;
float speedReqRight = 0.0,speedReqLeft = 0.0;

float left_wheel = 0;
float right_wheel = 0;

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );


void setup() {
  //Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderD, CHANGE); 
  
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  ledcSetup(channel_R ,freq , res);
  ledcAttachPin(R_PWM,channel_R);
  ledcSetup(channel_L ,freq , res);
  ledcAttachPin(L_PWM,channel_L);
}

void loop() {
  nh.spinOnce();
  direction();
  speedReqRight = abs(right_wheel);
  speedReqLeft = abs(left_wheel);

  targetRight = (speedReqRight *60) /circumference;
  targetLeft = (speedReqLeft *60) /circumference;
  noInterrupts();
  int posRight= abs(encoder0Pos);
  int posLeft= abs(encoder1Pos);
  interrupts();
//  dirRight = 1;
//  dirLeft = 1;
  
//  Serial.print("TARGET_RIGHT  ");
//  Serial.print(targetRight);
//  
//  Serial.print(" ERROR  LEFT");
//  Serial.print(errorRight);
//  Serial.print(" CURRENT  ");
//  Serial.print(v1Filt);
//  Serial.print(" PID_OUT LEFT");
//  Serial.print(pid_outRight);
//
//
//  Serial.print("TARGET_LEFT  ");
//  Serial.print(targetLeft);
//  
//  Serial.print(" ERROR LEFT  ");
//  Serial.print(errorLeft);
//  Serial.print(" CURRENT  ");
//  Serial.print(v2Filt);
//  Serial.print(" PID_OUT LEFT");
//  Serial.println(pid_outLeft);

 // Compute velocity
 currT = micros();
 deltaT = ((float)(currT - prevT)) / 1.0e6;
 velocity1 = (posRight - posPrevRight)/deltaT;
 velocity2 = (posLeft - posPrevLeft)/deltaT;
 posPrevRight = posRight;
 posPrevLeft = posLeft;
 prevT= currT;
  
 float v1 = velocity1/3000.0*60.0;
 float v2 = velocity2/3000.0*60.0;
 
 v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
 v1Prev = v1;

 v2Filt = 0.854*v2Filt + 0.0728*v2 + 0.0728*v2Prev;
 v2Prev = v2;

 errorRight = targetRight - v1Filt;
 errorLeft = targetLeft - v2Filt;
// if (error < -10) error = 0;
 eintegralRight = eintegralRight + errorRight*deltaT;
 eintegralLeft = eintegralLeft + errorRight*deltaT;
 pid_outRight = kp * errorRight + ki *eintegralRight;
 pid_outLeft = kp * errorLeft + ki *eintegralLeft;
 

// if (pid_outRight<0){
//    dirRight = -1;
// }
// if (pid_outLeft<0){
//    dirLeft = -1;
// }
 
 
  if(pid_outRight > 800){
    pid_outRight = 800;
  }
  if(pid_outRight <= 80){
    pid_outRight = 80;
    }


  
  if(pid_outLeft > 800){
    pid_outLeft = 800;
  }
  if(pid_outLeft <= 80){
    pid_outLeft = 80;
    }
pwrRight = (int) fabs(pid_outRight);
    setMotorRight(dirRight,pwrRight);

pwrLeft = (int) fabs(pid_outLeft);
    setMotorLeft(dirLeft,pwrLeft);

}

void direction(){
  if(left_wheel < 0) dirLeft = -1;
  else{ dirLeft = 1; }
  if(right_wheel < 0) dirRight = -1;
  else{dirRight = 1; }

}

void setMotorLeft(int dir, int pwr){
  ledcWrite(channel_L, pwr); 
  if(dir == 1)
  {
    digitalWrite(L_FORW,LOW);
    digitalWrite(L_BACK,HIGH);
    }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(L_FORW,HIGH);  
    digitalWrite(L_BACK,LOW);
  }
  else{
    // Or dont turn
    digitalWrite(L_FORW,LOW);
    digitalWrite(L_BACK,LOW);    
  } 
  }

void setMotorRight(int dir, int pwr){
  ledcWrite(channel_R, pwr); 
  if(dir == 1)
  {
    digitalWrite(R_FORW,LOW);
    digitalWrite(R_BACK,HIGH);
    }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(R_FORW,HIGH);  
    digitalWrite(R_BACK,LOW);
  }
  else{
    // Or dont turn
    digitalWrite(R_FORW,LOW);
    digitalWrite(R_BACK,LOW);    
  } 
  }




void doEncoderA(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
 
}

void doEncoderB(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  

}

// ************** encoder 2 *********************

void doEncoderC(){  

  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinB) == HIGH) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
 
}

void doEncoderD(){  

  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {  
      encoder1Pos = encoder1Pos - 1;         // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder1PinA) == LOW) {   
      encoder1Pos = encoder1Pos - 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;          // CCW
    }
  }
  

}

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){
    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
}
