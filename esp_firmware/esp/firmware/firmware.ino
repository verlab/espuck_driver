/************************************************************************
 *  E - P U C K   F I R M W A R E
 ************************************************************************
 *
 * Paulo Rezeck <rezeck@dcc.ufmg.br>
 * Mauricio Ferrari <mauferrari@dcc.ufmg.br>
 * 
 * E-Puck Upgrade
 * Computer Vision and Robotics Lab
 * Federal University of Minas Gerais - Brazil
 ************************************************************************/
 /* ROS lib */
#include <ros.h>
#include "WifiHardware.h"

/* Message types */
#include <espuck_driver/Proximity.h>
#include <espuck_driver/Battery.h>
#include <espuck_driver/Light.h>
#include <espuck_driver/Temperature.h>
#include <espuck_driver/Microphone.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
/************************************************************************/


/************************************************************************
 * Defines
 ************************************************************************/
#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
/************************************************************************/

/* Basic setup */
IPAddress ROS_MASTER_ADDRESS(10, 42, 0, 1); // ros master ip
char* WIFI_SSID = "epuck_net"; // network name
char* WIFI_PASSWD = "epuck_9895"; // network password
String epuck_name; // epuck name is get from epuck serial (epuck_####)
/************************************************************************/

/************************************************************************
 * ROS Setup
 ************************************************************************/
ros::NodeHandle_<WifiHardware> nh;
/* Command velocity callback */
void cmdvel_callback(const geometry_msgs::Twist& msg);
String cmdvel_topic;
ros::Subscriber<geometry_msgs::Twist> *cmdvel_sub;
/* Proximity sensor publisher */
espuck_driver::Proximity proximity_msg;
void update_proximity(void);
String proximity_topic;
ros::Publisher *proximity_pub;
/* Battery state publisher */
espuck_driver::Battery battery_msg;
void update_battery(void);
String battery_topic;
ros::Publisher *battery_pub;
/* Ambient light sensor publisher */
espuck_driver::Light light_msg;
void update_ambient_light(void);
String light_topic;
ros::Publisher *light_pub;
/* Temperature sensor publisher */
espuck_driver::Temperature temperature_msg;
void update_temperature(void);
String temperature_topic;
ros::Publisher *temperature_pub;
/* Microphone sensor publisher */
espuck_driver::Microphone microphone_msg;
void update_microphone(void);
String microphone_topic;
ros::Publisher *microphone_pub;
/* IMU sensor publisher */
sensor_msgs::Imu imu_msg;
void update_imu(void);
String imu_topic;
ros::Publisher *imu_pub;
/************************************************************************/


/************************************************************************
 * Arduino Setup
 ************************************************************************/
 
void setup() {  
  /* Serial Setup */
  Serial.begin(230400); // baudrate used on epuck
  Serial.swap(); // swap to use serial2 on esp8266
  delay(1000); // wait while
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  /* Get epuck id */
  delay(1000); // wait while
  Serial.write(-'v'); // command to get epuck id
  Serial.write('\0'); // command end
  Serial.flush();
  //while(Serial.available() != 2) delayMicroseconds(50);
  delay(1000); // wait while
  uint16_t epuck_id = 0;
  if (Serial.available() == 2){
    char byte0 = Serial.read();
    char byte1 = Serial.read();
    epuck_id = byte1 << 8;
    epuck_id += byte0; 
  }
  epuck_name = String("/epuck_") + String(epuck_id);
  /* Define tcp port as 1000 + epuck id */
  //uint16_t ROS_MASTER_PORT = 10000 + epuck_id;
  uint16_t ROS_MASTER_PORT = 11411;
  /* Start ROS communication module */
  nh.getHardware()->setROSConnection(ROS_MASTER_ADDRESS, ROS_MASTER_PORT);
  nh.getHardware()->connectWifi(WIFI_SSID, WIFI_PASSWD);
  /* Start publishers */
  proximity_topic = epuck_name + String("/proximity");
  proximity_pub = new ros::Publisher(proximity_topic.c_str(), &proximity_msg);
  battery_topic = epuck_name + String("/battery");
  battery_pub = new ros::Publisher(battery_topic.c_str(), &battery_msg);
  light_topic = epuck_name + String("/ambient_light");
  light_pub = new ros::Publisher(light_topic.c_str(), &light_msg);
  temperature_topic = epuck_name + String("/temperature");
  temperature_pub = new ros::Publisher(temperature_topic.c_str(), &temperature_msg);
  microphone_topic = epuck_name + String("/microphone");
  microphone_pub = new ros::Publisher(microphone_topic.c_str(), &microphone_msg);
  imu_topic = epuck_name + String("/imu");
  imu_pub = new ros::Publisher(imu_topic.c_str(), &imu_msg);
  /* Start subscribers */
  cmdvel_topic = epuck_name + String("/cmd_vel");
  cmdvel_sub = new ros::Subscriber<geometry_msgs::Twist>(cmdvel_topic.c_str(), cmdvel_callback);
  /* Starting ros node */
  nh.initNode();
  /* Address Publishers */
  nh.advertise(*proximity_pub);
  nh.advertise(*battery_pub);
  nh.advertise(*light_pub);
  nh.advertise(*imu_pub);
  nh.advertise(*temperature_pub);
  nh.advertise(*microphone_pub);
  /* Address Subscribers */
  nh.subscribe(*cmdvel_sub);
  /* Message Setup */
  proximity_msg.header.frame_id = "proximity";
  proximity_msg.header.seq = -1;
  proximity_msg.proximity_length = 8;
  proximity_msg.proximity = (int16_t *)malloc(8*sizeof(int16_t));
  
  light_msg.header.frame_id = "ambient_light";
  light_msg.header.seq = -1;
  light_msg.ambient_light_length = 8;
  light_msg.ambient_light = (int16_t *)malloc(8*sizeof(int16_t));
  
  imu_msg.header.frame_id = "imu";
  imu_msg.header.seq = -1;

  microphone_msg.header.frame_id = "microphone";
  microphone_msg.header.seq = -1;
  microphone_msg.microphone_length = 3;
  microphone_msg.microphone = (int16_t *)malloc(3*sizeof(int16_t));
  
  /* Setup complete esp LED message */
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i <= 20; i++){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(80);                   
  }
  /* Calibrating Proximity Sensors */
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write("K\n");
  delay(3000);
  /* Setup complete epuck LED message */
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
  update_temperature();
  update_ambient_light();
  update_imu();
  update_microphone();
  
  nh.spinOnce();
  delayMicroseconds(5000); // this delay is necessary because esp8266 is too faster than dspic6014A
}
/************************************************************************/

/* Update and publish proximity sensors data */
void update_proximity(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'N'); // command to receive proximity sensors
  Serial.write('\0'); // command end
  Serial.flush(); // wait til the command be sent
  while(Serial.available() != 16) delayMicroseconds(50); // wait til the completed data come

  if (Serial.available() == 16){ // if the data has 16bytes than it's probabily our sensor
    for (int i = 0; i < 8; i++){
      byte0 = Serial.read();
      byte1 = Serial.read();
      proximity_msg.proximity[i] = byte1 << 8;
      proximity_msg.proximity[i] += byte0;
    }
    proximity_msg.header.stamp = nh.now(); // update sequency and timestamp
    proximity_msg.header.seq++; 
    proximity_pub->publish( &proximity_msg ); // send message to be published
  }
}

/* Update and publish battery data */
void update_battery(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'b'); // command to receive battery state
  Serial.write('\0'); // command end
  Serial.flush(); // wait til the command be sent
  while(Serial.available() != 2) delayMicroseconds(50); // wait til the completed data come
  
  if (Serial.available() == 2){
    byte0 = Serial.read();
    byte1 = Serial.read();
    battery_msg.state = byte1 << 8;
    battery_msg.state += byte0;
    battery_pub->publish( &battery_msg );
  }
}

/* Update and publish imu data */
void update_imu(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'a');
  Serial.write(-'g');
  Serial.write('\0');
  Serial.flush();
  while(Serial.available() != 12) delayMicroseconds(50); // wait til the completed data come
  if (Serial.available() == 12){
    // Get acc X
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.x =  byte1 << 8;
    imu_msg.linear_acceleration.x += byte0;
    // Get acc Y
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.y =  byte1 << 8;
    imu_msg.linear_acceleration.y += byte0;
    // Get acc Z
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.z =  byte1 << 8;
    imu_msg.linear_acceleration.z += byte0;

    // Get gy X
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.angular_velocity.z =  byte1 << 8;
    imu_msg.angular_velocity.z += byte0;

    // Get gy Y
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.angular_velocity.z =  byte1 << 8;
    imu_msg.angular_velocity.z += byte0;

    // Get gy Z
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.angular_velocity.z =  byte1 << 8;
    imu_msg.angular_velocity.z += byte0;
    
    imu_msg.header.stamp = nh.now();
    imu_msg.header.seq++;
    imu_pub->publish( &imu_msg );
  }
}

/* Update and publish ambient light data */
void update_ambient_light(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'O');
  Serial.write('\0');
  Serial.flush(); 
  while(Serial.available() != 16) delayMicroseconds(50); // wait til the completed data come
  //delayMicroseconds(1000);

  if (Serial.available() == 16){
    for (int i = 0; i < 8; i++){
      byte0 = Serial.read();
      byte1 = Serial.read();
      light_msg.ambient_light[i] = byte1 << 8;
      light_msg.ambient_light[i] += byte0;
    }
    light_msg.header.stamp = nh.now();
    light_msg.header.seq++;
    light_pub->publish( &light_msg );
  } 
  
}

/* Update and publish temperature data */
void update_temperature(void){
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'t');
  Serial.write('\0');
  Serial.flush(); 
  while(Serial.available() != 1) delayMicroseconds(50); // wait til the completed data come

  if (Serial.available() == 1){
    temperature_msg.temp = Serial.read();
    temperature_pub->publish( &temperature_msg );
  }  
}

/* Update and publish microphone data */
void update_microphone(void){
  char byte0, byte1;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'u');
  Serial.write('\0');
  Serial.flush();
  while(Serial.available() != 6) delayMicroseconds(50); // wait til the completed data come
  
  if (Serial.available() == 6){ 
    for (int i = 0; i < 3; i++){
      byte0 = Serial.read();
      byte1 = Serial.read();
      microphone_msg.microphone[i] = byte1 << 8;
      microphone_msg.microphone[i] += byte0;
    }
    microphone_msg.header.stamp = nh.now();
    microphone_msg.header.seq++;
    microphone_pub->publish( &microphone_msg );
  }
}

/************************************************************************
 * C A L L B A C K S
 ************************************************************************/
void cmdvel_callback(const geometry_msgs::Twist& msg){
  /* set motor velocities */
  float linear = msg.linear.x;
  float angular = msg.angular.z;

  // Kinematic model for differential robot.
  float wl = (linear - (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;
  float wr = (linear + (WHEEL_SEPARATION / 2.0) * angular) / WHEEL_DIAMETER;

  // At input 1000, angular velocity is 1 cycle / s or  2*pi/s.
  int speedLeft = int(wl * 1000.0);
  int speedRight = int(wr * 1000.0);

  Serial.write(-'D');
  Serial.write(speedLeft & 0xff);
  Serial.write(speedLeft >> 8);
  Serial.write(speedRight & 0xff);
  Serial.write(speedRight >> 8);
  Serial.write('\0');
}

/************************************************************************/
/* T H E  E N D */
/************************************************************************/
