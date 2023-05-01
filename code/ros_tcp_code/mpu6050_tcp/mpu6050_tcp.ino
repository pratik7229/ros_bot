// note data will be in string format we need to write a node that can extract this data.
// in usable and readable format

// libraries to be included for library manager 
// adafruit mpu6050
// Adafruit Unified Sensor
// Adafruit Bus IO

#define ROSSERIAL_ARDUINO_TCP
#include "WiFi.h"
#include <ros.h>
#include <Wire.h>
#include <std_msgs/String.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



IPAddress server(192,168,0,238);
uint16_t serverPort = 11411;
const char*  ssid = "pratik";
const char*  password = "Pratikwalunj@7229";


std_msgs::String imu_msg;
ros::Publisher imu("imu", &imu_msg);
ros::NodeHandle nh;


int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

Adafruit_MPU6050 mpu;

void setup(void) {
  setupWiFi();
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.advertise(imu);
 
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  
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
 
  delay(100);
  
}
