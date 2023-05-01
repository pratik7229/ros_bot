// note data will be in string format we need to write a node that can extract this data.
// in usable and readable format

// libraries to be included for library manager 
// adafruit mpu6050
// Adafruit Unified Sensor
// Adafruit Bus IO


#include <ros.h>
#include <std_msgs/Int16.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <geometry_msgs/Twist.h>


#define encoder0PinA 18      // encoder 1
#define encoder0PinB 19

#define encoder1PinA 17     // encoder 2
#define encoder1PinB 5

#define SPEED_OF_SOUND 0.034

ros::NodeHandle nh;
std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &cmdVel_to_pwm_cb );

std_msgs::Int16 enc_r_msg;
std_msgs::Int16 enc_l_msg;
ros::Publisher right_enc("rwheel", &enc_r_msg );
ros::Publisher left_enc("lwheel", &enc_l_msg );
std_msgs::Int16 ultrasonic_msg;
ros::Publisher ultrasonic_node("ultrasonic_values", &ultrasonic_msg );

const uint8_t R_PWM =  12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;

const uint8_t L_PWM =  33;
const uint8_t L_BACK = 25;
const uint8_t L_FORW = 26;

const uint8_t channel_L =0;
const uint8_t channel_R= 1;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

const int trig_Pin = 4;
const int echo_Pin = 16;
long duration ;
float distance_in_cm;

float left_wheel;
float right_wheel;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

Adafruit_MPU6050 mpu;

void setup(void) {

  nh.initNode();
  nh.advertise(imu);
  nh.subscribe(sub);
  nh.advertise(right_enc);
  nh.advertise(left_enc);
  nh.advertise(ultrasonic_node);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  pin_def();
  stop();
  delay(100);
}
void pin_def(){

  const int freq = 5000;
  const int res = 8;

  pinMode(encoder0PinA, INPUT_PULLUP);    // encoder pins
  pinMode(encoder0PinB, INPUT_PULLUP);

  pinMode(encoder1PinA, INPUT_PULLUP); 
  pinMode(encoder1PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), doEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder0PinB), doEncoderB, CHANGE); 

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoderC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoderD, CHANGE); 
  
  pinMode(trig_Pin,OUTPUT);
  pinMode(echo_Pin,INPUT);
  
  pinMode(L_PWM,  OUTPUT);
  pinMode(L_FORW, OUTPUT);
  pinMode(L_BACK, OUTPUT);
  pinMode(R_PWM,  OUTPUT);
  pinMode(R_FORW, OUTPUT);
  pinMode(R_BACK, OUTPUT);
  
  ledcSetup(channel_R ,freq , res);
  ledcSetup(channel_L ,freq , res);

  ledcAttachPin(R_PWM,channel_R);
  ledcAttachPin(L_PWM,channel_L);

}

void loop() {
  /* Get new sensor events with the readings */
   nh.spinOnce();
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  AcX = a.acceleration.x;
  AcY = a.acceleration.y;
  AcZ = a.acceleration.z;
  GyX = g.gyro.x;
  GyY = g.gyro.y;
  GyZ = g.gyro.z;
  String AX = String(AcX);
  String AY = String(AcY);
  String AZ = String(AcZ);
  String GX = String(GyX);
  String GY = String(GyY);
  String GZ = String(GyZ);
  String data = "A" + AX + "B"+ AY + "C" + AZ + "D" + GX + "E" + GY + "F" + GZ + "G" ;
  int length = data.indexOf("G") +2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  imu_msg.data = data_final;
  imu.publish(&imu_msg);
  right_enc.publish(&enc_r_msg);
  left_enc.publish(&enc_l_msg);

  digitalWrite(trig_Pin,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Pin,LOW);

    duration = pulseIn(echo_Pin ,HIGH);
    distance_in_cm = ( duration * SPEED_OF_SOUND  )/ 2;
    ultrasonic_msg.data=distance_in_cm;
    ultrasonic_node.publish(&ultrasonic_msg);
  delay(3);
  
}

void cmdVel_to_pwm_cb( const geometry_msgs::Twist& velocity_msg){

    right_wheel = (velocity_msg.linear.x + velocity_msg.angular.z ) / 2 ;
    left_wheel = (velocity_msg.linear.x - velocity_msg.angular.z ) /2 ;
    direction();
    speed();
    if ( velocity_msg.linear.x ==0.0 & velocity_msg.angular.z ==0.0){
        stop();
    }
    

}

void direction(){
    digitalWrite(L_FORW, left_wheel >0 );
    digitalWrite(L_BACK,left_wheel < 0);
    digitalWrite(R_FORW,right_wheel > 0 );
    digitalWrite(R_BACK,right_wheel < 0);
}

void speed (){
    ledcWrite(channel_R, 200);  
    ledcWrite(channel_L, 200);
}

void stop()
{
   
   
   ledcWrite(channel_R, 0);  
   ledcWrite(channel_L, 0);
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
 enc_r_msg.data=encoder0Pos;
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
  
enc_r_msg.data=encoder0Pos;
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
 enc_l_msg.data=encoder1Pos;
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
  enc_l_msg.data=encoder1Pos;

}
