#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <std_msgs/Int16.h>

const int trig_Pin = 4;
const int echo_Pin = 16;

#define SPEED_OF_SOUND 0.034

long duration ;
float distance_in_cm;

ros::NodeHandle  nh;
IPAddress server(192,168,0,238);
uint16_t serverPort = 11411;
const char*  ssid = "pratik";
const char*  password = "Pratikwalunj@7229";

std_msgs::Int16 ultrasonic_msg;
ros::Publisher ultrasonic_node("ultrasonic_values", &ultrasonic_msg );

void setup()
{ 
    setupWiFi();
    nh.getHardware()->setConnection(server, serverPort);
    pinMode(trig_Pin,OUTPUT);
    pinMode(echo_Pin,INPUT);
    nh.initNode();
    nh.advertise(ultrasonic_node);
}

void loop()
{ 

  digitalWrite(trig_Pin,LOW);
  delayMicroseconds(2);
  digitalWrite(trig_Pin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_Pin,LOW);

    duration = pulseIn(echo_Pin ,HIGH);
    distance_in_cm = ( duration * SPEED_OF_SOUND  )/ 2;
    ultrasonic_msg.data=distance_in_cm;
    ultrasonic_node.publish(&ultrasonic_msg);
    nh.spinOnce();
    delay(100);
}

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());

}
