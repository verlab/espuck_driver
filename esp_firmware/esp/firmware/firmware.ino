/************************************************************************
 *  E - P U C K   F I R M W A R E
 ************************************************************************
 *
 * Paulo Rezeck <rezeck@dcc.ufmg.br>
 * Mauricio Ferrari <mauricio.ferrari@dcc.ufmg.br>
 * 
 * E-Puck Upgrade
 * Computer Vision and Robotics Lab (VERLAB)
 * Federal University of Minas Gerais (UFMG) - Brazil
 ************************************************************************/
 /* ROS lib */
#include <ros.h>
#include <ESP8266WiFi.h>

/* Message types */
#include <espuck_driver/Proximity.h>
#include <espuck_driver/Battery.h>
#include <espuck_driver/Light.h>
#include <espuck_driver/Temperature.h>
#include <espuck_driver/Microphone.h>
#include <espuck_driver/Sound.h>
#include <espuck_driver/Led.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Empty.h>

/* Defines */
#define WHEEL_DIAMETER 4.0        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

/* Wifi setup */
IPAddress ROS_MASTER_ADDRESS(10, 42, 0, 1); // ros master ip
char* WIFI_SSID = "epuck_net"; // network name
char* WIFI_PASSWD = "epuck_9895"; // network password

/* Commons Variables */
String epuck_name; // epuck name is get from epuck serial (epuck_####)
unsigned long timer, currentTime, lastTime;

/* Used to compute Odometry */
double xPos, yPos, theta;
double deltaSteps, deltaTheta;
double leftStepsDiff = 0, rightStepsDiff = 0;
double leftStepsPrev = 0, rightStepsPrev = 0;

/* ROS Setup */
/* ROS Node Instaciatation */
ros::NodeHandle nh;
/* Command velocity callback */
void cmdvel_callback(const geometry_msgs::Twist& msg);
String cmdvel_topic;
ros::Subscriber<geometry_msgs::Twist> *cmdvel_sub;
/* Led callback */
void led_callback(const espuck_driver::Led& msg);
String led_topic;
ros::Subscriber<espuck_driver::Led> *led_sub;
/* Sound callback */
void sound_callback(const espuck_driver::Sound& msg);
String sound_topic;
ros::Subscriber<espuck_driver::Sound> *sound_sub;
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
/* Odometry publisher */
nav_msgs::Odometry odom_msg;
void update_odom(void);
String odom_topic;
ros::Publisher *odom_pub;
/* TF to publish odom to base_link transformation */
tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped t;



/* Services */
/* Proximity sensors calibration */
void proxcalibration_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
String prox_calibration_topic;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> *prox_calibration_srv;
/* Stop epuck*/
void stop_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
String stop_topic;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> *stop_srv;
/* Reset epuck*/
void reset_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
String reset_topic;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> *reset_srv;
/* Reset odometry epuck*/
void odom_reset_callback(const std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
String odom_reset_topic;
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> *odom_reset_srv;

/************************************************************************/


/************************************************************************
 * Arduino Setup
 ************************************************************************/
 
void setup() {
  /* Connect the ESP8266 the the wifi AP */
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  /* Serial Setup */
  Serial.begin(230400); // baudrate used on epuck
  Serial.swap(); // swap to use serial2 on esp8266
  delay(2000); // wait while
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  /* Get epuck id */
  Serial.write(-'v'); // command to get epuck id
  Serial.write('\0'); // command end
  Serial.flush();
  delay(1000); // wait til the completed data come
  uint16_t epuck_id = 0;
  if (Serial.available() == 2){
    char byte0 = Serial.read();
    char byte1 = Serial.read();
    epuck_id = byte1 << 8;
    epuck_id += byte0; 
  }
  epuck_name = String("/epuck_") + String(epuck_id);
  /* Define tcp port as 1000 + epuck id */
  uint16_t ROS_MASTER_PORT = 10000 + epuck_id;
  //uint16_t ROS_MASTER_PORT = 11571;
  /* Start ROS communication module */
  nh.getHardware()->setConnection(ROS_MASTER_ADDRESS, ROS_MASTER_PORT);
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
  odom_topic = epuck_name + String("/odom");
  odom_pub = new ros::Publisher(odom_topic.c_str(), &odom_msg);
  
  /* Start subscribers */
  cmdvel_topic = epuck_name + String("/cmd_vel");
  cmdvel_sub = new ros::Subscriber<geometry_msgs::Twist>(cmdvel_topic.c_str(), cmdvel_callback);
  led_topic = epuck_name + String("/led");
  led_sub = new ros::Subscriber<espuck_driver::Led>(led_topic.c_str(), led_callback);
  sound_topic = epuck_name + String("/sound");
  sound_sub = new ros::Subscriber<espuck_driver::Sound>(sound_topic.c_str(), sound_callback);
  /* Service setup */
  prox_calibration_topic = epuck_name + String("/proximity_calibration");
  prox_calibration_srv = new ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>(prox_calibration_topic.c_str(), &proxcalibration_callback);
  stop_topic = epuck_name + String("/stop");
  stop_srv = new ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>(stop_topic.c_str(), &stop_callback);
  reset_topic = epuck_name + String("/reset");
  reset_srv = new ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>(reset_topic.c_str(), &reset_callback);
  odom_reset_topic = epuck_name + String("/odom_reset");
  odom_reset_srv = new ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response>(odom_reset_topic.c_str(), &odom_reset_callback);
  
  /* Starting ros node */
  nh.initNode();
  /* Address Publishers */
  nh.advertise(*proximity_pub);
  nh.advertise(*battery_pub);
  nh.advertise(*light_pub);
  nh.advertise(*imu_pub);
  nh.advertise(*odom_pub);
  broadcaster.init(nh);
  nh.advertise(*temperature_pub);
  nh.advertise(*microphone_pub);
  /* Address Subscribers */
  nh.subscribe(*cmdvel_sub);
  nh.subscribe(*led_sub);
  nh.subscribe(*sound_sub);
  /* Address Services */
  nh.advertiseService(*prox_calibration_srv);
  nh.advertiseService(*stop_srv);
  nh.advertiseService(*reset_srv);
  nh.advertiseService(*odom_reset_srv);
  
  /* Message Setup */
  proximity_msg.header.frame_id = "proximity";
  proximity_msg.header.seq = -1;
  proximity_msg.data_length = 8;
  proximity_msg.max_range =  0.05;        // 5 cm.     
  proximity_msg.min_range =  0.005;       // 0.5 cm.
  proximity_msg.field_of_view = 0.26;    // About 15 degrees...to be checked!
  proximity_msg.data = (float *)malloc(8*sizeof(float));
  
  light_msg.header.frame_id = "ambient_light";
  light_msg.header.seq = -1;
  light_msg.ambient_light_length = 8;
  light_msg.ambient_light = (int16_t *)malloc(8*sizeof(int16_t));
  
  imu_msg.header.frame_id = "imu";
  imu_msg.header.seq = -1;

  odom_msg.header.frame_id = "odom";
  odom_msg.header.seq = -1;
  odom_msg.child_frame_id = "/base_link";
/*  tf_msg.transforms.header.frame_id = "odom";
  tf_msg.transforms.header.seq = -1;
  tf_msg.transforms.child_frame_id = "base_link";*/
  lastTime = micros();

  microphone_msg.header.frame_id = "microphone";
  microphone_msg.header.seq = -1;
  microphone_msg.microphone_length = 3;
  microphone_msg.microphone = (int16_t *)malloc(3*sizeof(int16_t));

  nh.loginfo("[EPUCK] Starting Epuck Communication");
  /* Setup complete esp LED message */
  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i <= 20; i++){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(80);                   
  }
  nh.loginfo("[EPUCK] Calibrating Proximity Sensors (3 seconds)");
  /* Calibrating Proximity Sensors */
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write("K\n");
  delay(3000);
  nh.loginfo("[EPUCK] Epuck is Ready!");
  /* Setup complete epuck LED message */
  char buf [6];
  Serial.write("S\n"); // stop epuck
  Serial.flush();
  for (int i = 0; i < 32; i++){
    sprintf (buf, "L,%d,2\n", i % 8); // inverse led state
    Serial.write(buf);
    delay(100);
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
  update_odom();
  
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
  timer = micros();
  while(Serial.available() != 16 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come

  if (Serial.available() == 16){ // if the data has 16bytes than it's probabily our sensor
    for (int i = 0; i < 8; i++){
      byte0 = Serial.read();
      byte1 = Serial.read();
      proximity_msg.data[i] = byte1 << 8;
      proximity_msg.data[i] += byte0;
      if (proximity_msg.data[i] > 0){
        proximity_msg.data[i] = 0.5/sqrt(proximity_msg.data[i]); // Transform the analog value to a distance value in meters (given from field tests).
      }
      else{
        proximity_msg.data[i] = proximity_msg.max_range;
      }
      proximity_msg.data[i] = max(min(proximity_msg.data[i], proximity_msg.max_range), proximity_msg.min_range);
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
  timer = micros();
  while(Serial.available() != 2 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come
  
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
  timer = micros();
  while(Serial.available() != 12 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come
  
  if (Serial.available() == 12){
    // Get acc X
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.x =  byte1 << 8;
    imu_msg.linear_acceleration.x += byte0;
    imu_msg.linear_acceleration.x  =  (imu_msg.linear_acceleration.x - 2048.0) / 800.0 * 9.81; // 1 g = about 800, then transforms in m/s^2.
    // Get acc Y
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.y =  byte1 << 8;
    imu_msg.linear_acceleration.y += byte0;
    imu_msg.linear_acceleration.y  =  (imu_msg.linear_acceleration.y - 2048.0) / 800.0 * 9.81; // 1 g = about 800, then transforms in m/s^2.
    // Get acc Z
    byte0 = Serial.read();
    byte1 = Serial.read();
    imu_msg.linear_acceleration.z =  byte1 << 8;
    imu_msg.linear_acceleration.z += byte0;
    imu_msg.linear_acceleration.z  =  (imu_msg.linear_acceleration.z - 2048.0) / 800.0 * 9.81; // 1 g = about 800, then transforms in m/s^2.
    imu_msg.linear_acceleration_covariance[0] = 0.01;
    imu_msg.linear_acceleration_covariance[4] = 0.01;
    imu_msg.linear_acceleration_covariance[8] = 0.01;

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
  timer = micros();
  while(Serial.available() != 16 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come

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
  timer = micros();
  while(Serial.available() != 1 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come

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
  timer = micros();
  while(Serial.available() != 6 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come
  
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

/* Update and publish microphone data */
void update_odom(void){
  char byte0, byte1, byte2, byte3;
  long motorPositionLeft, motorPositionRight;
  while(Serial.available() > 0) Serial.read(); // clear serial buffer
  Serial.write(-'Q');
  Serial.write('\0');
  Serial.flush();
  timer = micros();
  while(Serial.available() != 8 && (micros() - timer) < 5000) delayMicroseconds(10); // wait til the completed data come
  
  if (Serial.available() == 8){ 
    /* Read left encoder 32 bits */
    byte0 = Serial.read();
    byte1 = Serial.read();
    byte2 = Serial.read();
    byte3 = Serial.read();
    motorPositionLeft = byte3 << 8;
    motorPositionLeft += byte2;
    motorPositionLeft = motorPositionLeft << 8;
    motorPositionLeft += byte1;
    motorPositionLeft = motorPositionLeft << 8;
    motorPositionLeft += byte0;
    /* Read right encoder 32 bits */
    byte0 = Serial.read();
    byte1 = Serial.read();
    byte2 = Serial.read();
    byte3 = Serial.read();
    motorPositionRight = byte3 << 8;
    motorPositionRight += byte2;
    motorPositionRight = motorPositionRight << 8;
    motorPositionRight += byte1;
    motorPositionRight = motorPositionRight << 8;
    motorPositionRight += byte0;

    /* Compute odometry */
    leftStepsDiff = motorPositionLeft * MOT_STEP_DIST - leftStepsPrev; // Expressed in meters.
    rightStepsDiff = motorPositionRight * MOT_STEP_DIST - rightStepsPrev;   // Expressed in meters.

    deltaTheta = (rightStepsDiff - leftStepsDiff) / WHEEL_DISTANCE;   // Expressed in radiant.
    deltaSteps = (rightStepsDiff + leftStepsDiff) / 2.0;        // Expressed in meters.

    xPos += deltaSteps * cos(theta + deltaTheta/2);   // Expressed in meters.
    yPos += deltaSteps * sin(theta + deltaTheta/2);   // Expressed in meters.
    theta += deltaTheta;    // Expressed in radiant.

    leftStepsPrev = motorPositionLeft * MOT_STEP_DIST;     // Expressed in meters.
    rightStepsPrev = motorPositionRight * MOT_STEP_DIST;    // Expressed in meters.

    odom_msg.pose.pose.position.x = xPos;       
    odom_msg.pose.pose.position.y = yPos;

    // Since all odometry is 6DOF we'll need a quaternion created from yaw.
    odom_msg.pose.pose.orientation = tf::createQuaternionFromYaw(theta);
    currentTime = micros(); 
    odom_msg.twist.twist.linear.x = deltaSteps / ((currentTime-lastTime)/1000000.0);   // "deltaSteps" is the linear distance covered in meters from the last update (delta distance);
                                                                                        // the time from the last update is measured in seconds thus to get m/s we multiply them.
    odom_msg.twist.twist.angular.z = deltaTheta / ((currentTime-lastTime)/1000000.0);  // "deltaTheta" is the angular distance covered in radiant from the last update (delta angle);
                                                                                        // the time from the last update is measured in seconds thus to get rad/s we multiply them.
    lastTime = micros();

    /* publish odometry */
    odom_msg.header.stamp = nh.now();
    odom_msg.header.seq++;
    odom_pub->publish( &odom_msg );
    
    /* tf odom->base_link */
    String odom_topic = epuck_name  + String("/odom");
    t.header.frame_id = odom_topic.c_str();
    String baselink_topic = epuck_name  + String("/base_link");
    t.child_frame_id = baselink_topic.c_str();
    t.transform.translation.x = xPos;
    t.transform.translation.y = yPos;
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
    broadcaster.sendTransform(t);
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
  Serial.flush();
}

void led_callback(const espuck_driver::Led& msg){
  /* set led state */
  Serial.write(-'L');
  Serial.write(msg.index);
  Serial.write(msg.state);
  Serial.write('\0');
  Serial.flush();
}

void sound_callback(const espuck_driver::Sound& msg){
  /* play sound */
  Serial.write(-'T');
  Serial.write(msg.index);
  Serial.write('\0');
  Serial.flush();
}

void proxcalibration_callback( const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  Serial.write("K\n");
  Serial.flush();
  nh.loginfo("[EPUCK] Calibrating Proximity Sensors (3 seconds)");
  nh.loginfo("[EPUCK] Proximity Sensors Calibrated!");
}

void stop_callback( const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  Serial.write(-'S');
  Serial.flush();
  nh.loginfo("[EPUCK] Stop!");
}

void reset_callback( const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  Serial.write(-'R');
  Serial.flush();
  nh.loginfo("[EPUCK] Reset!");
}

void odom_reset_callback( const std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  Serial.write(-'P');
  Serial.write(0);
  Serial.write(0);
  Serial.write(0);
  Serial.write(0);
  Serial.flush();
  nh.loginfo("[EPUCK] Odometry Reset!");
}
/************************************************************************/
/* T H E  E N D */
/************************************************************************/
