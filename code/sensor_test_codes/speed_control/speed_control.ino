#define encoder0PinA 18      // encoder 1
#define encoder0PinB 19

#define encoder1PinA 17     // encoder 2
#define encoder1PinB 5

const uint8_t R_PWM =  12;
const uint8_t R_BACK = 14;
const uint8_t R_FORW = 27;
const uint8_t channel_R= 1;

volatile long encoder0Pos = 0;    // encoder 1
volatile long encoder1Pos = 0;    // encoder 2

const int freq = 5000;
const int res = 8;
int dir;

long prevT = 0;
int posPrev = 0;
int pos = 0;

long currT = 0;
float deltaT = 0;
float velocity1 = 0;

float v1Filt = 0;
float v1Prev = 0;
float v2Filt = 0;
float v2Prev = 0;


float kp = 1.1;
float kd = 0;
float ki = 8;

float error = 0,target = 0;
float eintegral = 0;
float pid_out = 0;
float pwr = 0;


int ppr = 3000;
float wheelDia = 0.044;  //in m
float circumference = wheelDia * 3.142;
float speedReq = 0.06;


void setup() {
  Serial.begin(9600);
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

  ledcSetup(channel_R ,freq , res);
  ledcAttachPin(R_PWM,channel_R);
}

void loop() {
  
  target = (speedReq *60) /circumference;
  noInterrupts();
  int posi= encoder0Pos;
  
  interrupts();
  dir = 1;
  Serial.print("  TARGET  ");
  Serial.print(target);
  
  Serial.print("  ERROR  ");
  Serial.print(error);
  Serial.print("  CURRENT  ");
  Serial.print(v1Filt);
  Serial.print("  PID_OUT ");
  Serial.println(pid_out);

 // Compute velocity
 currT = micros();
 deltaT = ((float)(currT - prevT)) / 1.0e6;
 velocity1 = (encoder0Pos - posPrev)/deltaT;
 posPrev = encoder0Pos;
 prevT= currT;
  
  float v1 = velocity1/3000.0*60.0;
 
 v1Filt = 0.854*v1Filt + 0.0728*v1 + 0.0728*v1Prev;
 v1Prev = v1;

 error = target - v1Filt;
// if (error < -10) error = 0;
 eintegral = eintegral + error*deltaT;
 pid_out = kp * error + ki *eintegral;
 
 
 dir = 1;
 if (pid_out<0){
    dir = -1;
 }
 
 
  if(pid_out > 800){
    pid_out = 800;
  }
  if(pid_out <= 80){
    pid_out = 80;
    }
pwr = (int) fabs(pid_out);
    setMotor(dir,pwr);
}

void setMotor(int dir, int pwr){
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
