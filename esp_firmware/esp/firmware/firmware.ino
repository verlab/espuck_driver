/*
 ************************************************************************
 *  E - P U C K   F I R M W A R E
 ************************************************************************
 *
 * Paulo Rezeck <rezeck@dcc.ufmg.br>
 * Mauricio Ferrari <mauferrari@dcc.ufmg.br>
 * 
 * E-Puck Upgrate
 * Computer Vision and Robotics Lab
 * Federal University of Minas Gerais - Brazil
 ************************************************************************/
 
#include <ros.h>
#include "WifiHardware.h"

/* Message types */
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Byte.h>

/************************************************************************
 * Defines
 ************************************************************************/
/* Basic setup */
#define DEBUG 1
IPAddress ROS_MASTER_ADDRESS(150, 164, 212, 68);
uint16_t ROS_MASTER_PORT = 11411;

char* WIFI_SSID = "hero";
char* WIFI_PASSWD = "hero_network";
#define ROBOT_ID "1571"

/* Robot Identification Setup */
#define ROBOT_NAME  "/epuck_" ROBOT_ID
#define VELOCITY_TOPIC "/epuck_" ROBOT_ID "/cmd_vel"
#define BATTERY_TOPIC "/epuck_" ROBOT_ID "/battery"
#define LED_TOPIC "/epuck_" ROBOT_ID "/led"
#define PROXIMITY_TOPIC "/epuck_" ROBOT_ID "/proximity"
/************************************************************************/


void update_proximity(void);
void update_battery(void);

/************************************************************************
 * ROS Setup
 ************************************************************************/
ros::NodeHandle_<WifiHardware> nh;

//void velocity_cb(const geometry_msgs::Twist& msg);
//ros::Subscriber<geometry_msgs::Twist> velocity_sub(VELOCITY_TOPIC, velocity_cb);

sensor_msgs::Range proximity_msg;
ros::Publisher proximity_pub(PROXIMITY_TOPIC, &proximity_msg);

std_msgs::Byte battery_msg;
ros::Publisher battery_pub(BATTERY_TOPIC, &battery_msg);

/************************************************************************/

/************************************************************************
 * Arduino Setup
 ************************************************************************/
void setup() {
  Serial.begin(230400);
  Serial.print("Node name: ");
  Serial.println(ROBOT_ID);
  Serial.print("ROS MASTER: ");
  Serial.print(ROS_MASTER_ADDRESS);
  Serial.print(":");
  Serial.println(ROS_MASTER_PORT);
  Serial.println("ESPUCK authors:");
  Serial.println("\t-Paulo Rezeck rezeck@dcc.ufmg.br");
  Serial.println("\t-Mauricio Ferrari mauferrari@dcc.ufmg.br");
  Serial.println("");

  if(DEBUG) Serial.println("Init networking ...");
  nh.getHardware()->setROSConnection(ROS_MASTER_ADDRESS, ROS_MASTER_PORT);
  nh.getHardware()->connectWifi(WIFI_SSID, WIFI_PASSWD);
  
  /* Starting ros node */
  if(DEBUG) Serial.println("Init ros node ...");
  nh.initNode();
  nh.advertise(proximity_pub);
  nh.advertise(battery_pub);
  //nh.subscribe(velocity_sub);
  
  proximity_msg.header.frame_id = ROBOT_NAME;
  proximity_msg.header.seq = -1;

  /* Setup complete LED message */
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i=0; i<= 6; i++){
    digitalWrite(LED_BUILTIN, LOW);
    delay(150);                   
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);        
  }
  if(DEBUG) Serial.println("Setup finished");
  
  Serial.swap();
  delay(1000);
  
  /* Calibrating Proximity Sensors */
  while(Serial.available() > 0) Serial.read();
  Serial.write("K\n");  
  delay(3000);
  
  char buf [6];
  for (int i = 0; i < 8; i++){
    sprintf (buf, "L,%d,1\n", i);
    Serial.write(buf);
    delay(150);
  }
  
  for (int i = 0; i < 8; i++){
    sprintf (buf, "L,%d,0\n", i);
    Serial.write(buf);
    delay(150);
  }

  
}
/************************************************************************/

/************************************************************************
 * M A I N  L O 0 P
 ************************************************************************/
void loop(){
  update_proximity();
  update_battery();
  
  nh.spinOnce();
  delayMicroseconds(1000);
}
/************************************************************************/

void update_proximity(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read();
  Serial.write(-'N');
  Serial.write('\0');
  Serial.flush();
  delayMicroseconds(1000);

  if (Serial.available() > 0){
    byte0 = Serial.read();
    byte1 = Serial.read();
    
    proximity_msg.header.stamp = nh.now();
    proximity_msg.header.seq++;
    proximity_msg.min_range = (float)byte0;
    proximity_msg.max_range = (float)byte1;
    proximity_msg.range = byte1 << 8;
    proximity_msg.range += byte0;
    proximity_pub.publish( &proximity_msg );
  }
}

void update_battery(void){
  while(Serial.available() > 0) Serial.read();
  Serial.write(-'b');
  Serial.write('\0');
  Serial.flush();
  delayMicroseconds(1000);
  if (Serial.available() > 0){
    battery_msg.data = (byte)Serial.read();
    battery_pub.publish( &battery_msg );
  }
}



/************************************************************************
 * C A L L B A C K S
 ************************************************************************/
/*void velocity_cb(const geometry_msgs::Twist& msg){
  /* set motor velocities */
/*
  float linear = _max(_min(msg.linear.x, WHEEL_DIAMETER), -WHEEL_DIAMETER);
  float angular = _max(_min(msg.angular.z, 2*WHEEL_DIAMETER/WHEEL_SEPARATION), -2*WHEEL_DIAMETER/WHEEL_SEPARATION);

  float wl0 = (linear - ((float) WHEEL_SEPARATION/2.0) * angular) / (float) WHEEL_DIAMETER;
  float wr0 = (linear + ((float) WHEEL_SEPARATION/2.0) * angular) / (float) WHEEL_DIAMETER;

  int wr = -(int)(_map2(wr0, -2.0, 2.0, -500, 500)) + int(WHEEL_R_MID);
  int wl = (int)(_map2(wl0, -2.0, 2.0, -500, 500)) + int(WHEEL_L_MID);
  
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));   // Turn the LED on (Note that LOW is the voltage level

  /* Control the wheels */
 /* //if (abs (wr - wheel_right.readMicroseconds()) > 5){
    wheel_right.writeMicroseconds(wr);
  //}
  //if (abs(wl - wheel_left.readMicroseconds()) > 5){
    wheel_left.writeMicroseconds(wl);
  //}
  delayMicroseconds(200);
}*/

/************************************************************************/
/* T H E  E N D */
/************************************************************************/
