#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>

#define UPDATE_RATE 50

ros::NodeHandle nh;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Motor Control Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define L_ESC 5 //Teensy pin for the left motor's ESC
#define R_ESC 6 //Teensy pin for the right motor's ES

#define STOP_PWM 4915
#define RANGEPWM 1311

#define MIN_UPDATE_RATE 1000

unsigned long motor_update = millis();

//Callback function for updating the both ESC
/*Since we are controlling both motors using the Geometry Twist messages
We need to both motors to behave sychronously to either move forward/backwards or to turn left/right*/
void ESC_Change( const geometry_msgs::Twist& msg) {
  //float scalingFactor = 1;
  float linear = msg.linear.x;
  float angular = msg.angular.y;
/*We can not go forward/backwards while turning left/right
Only one of the four options can be selected at a time
turning left/right has highest priority
moving forward/backwards has the nent priority*/
if(linear !=0 && angular != 0) //if both joysticks are moving do nothing
{
  motor_update = millis(); // Record the time that the motor was updated
 
  }
 
else if(angular > 0  && linear == 0) //turn right
{
  analogWrite(L_ESC, (unsigned short)(STOP_PWM + RANGEPWM * 0));
  analogWrite(R_ESC, (unsigned short)(STOP_PWM + RANGEPWM * angular));
  motor_update = millis(); // Record the time that the motor was updated
 
  }

 else if(angular < 0  && linear == 0) //turn left
{
  analogWrite(L_ESC, (unsigned short)(STOP_PWM + RANGEPWM * angular));
  analogWrite(R_ESC, (unsigned short)(STOP_PWM + RANGEPWM * 0));
  motor_update = millis(); // Record the time that the motor was updated
 
  }


else if(linear > 0  && angular == 0) //turn forward or backwards depends on the value for linear. Values are between [-1 1]
{
  analogWrite(L_ESC, (unsigned short)(STOP_PWM + RANGEPWM * linear));
  analogWrite(R_ESC, (unsigned short)(STOP_PWM + RANGEPWM * linear));
  motor_update = millis(); // Record the time that the motor was updated
 
  }  

 
}
/*                                  I changed the topic name from "/cmd_vel" to "cmd_vel"                                                               */
ros::Subscriber<geometry_msgs::Twist> sub_to_cmd_vel("cmd_vel", &ESC_Change);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IMU  Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void updateIMU(void);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); //Initializing the Adafruit BNO055 Sensor

geometry_msgs::Vector3Stamped IMU_eulerOrientationMsg;
ros::Publisher IMU_eulerOrientation("IMU_eulerOrientation", &IMU_eulerOrientationMsg);

geometry_msgs::Vector3Stamped IMU_angularVelocityMsg;
ros::Publisher IMU_angularVelocity("IMU_angularVelocity", &IMU_angularVelocityMsg);

geometry_msgs::Vector3Stamped IMU_linearAccelerationMsg;
ros::Publisher IMU_linearAcceleration("IMU_linearAcceleration", &IMU_linearAccelerationMsg);

geometry_msgs::Vector3Stamped IMU_absoluteOrientationMsg;
ros::Publisher IMU_absoluteOrientation("IMU_absoluteOrientation", &IMU_absoluteOrientationMsg);

geometry_msgs::Vector3Stamped IMU_magVecMsg;
ros::Publisher IMU_magVec("IMU_magVec", &IMU_magVecMsg);

#define IMU_PUBLISH_RATE 100 //hz

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GPS Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

sensor_msgs::NavSatFix navSat_msg;
ros::Publisher gpsPub("gps", &navSat_msg);

#define GPSSerial Serial2       // GPS connected to Teensy Serial port 2
Adafruit_GPS GPS(&GPSSerial);   // Connect to the GPS on the hardware port
#define GPSECHO false

// Publish Timing Control Variables
uint32_t publish_imu_time = 0;
uint32_t publish_gps_time = 0;

#define GPS_PUBLISH_RATE 1 //hz

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Setup ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Motor Control Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //Configuring the PWM pins for output
  pinMode(L_ESC, OUTPUT);
  pinMode(R_ESC, OUTPUT);

  //Allow for 16-bits of PWM write resolution
  analogWriteResolution(16);

  //Set the frequencies for the PWM output that is sent to the ESCs
  analogWriteFrequency(L_ESC, UPDATE_RATE);
  analogWriteFrequency(R_ESC, UPDATE_RATE);

  //Set the PWM value to the ESC predetermined STOP duty cycle
  analogWrite(L_ESC, STOP_PWM);
  analogWrite(R_ESC, STOP_PWM);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ IMU Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  while (!bno.begin());   //Wait until the BNO055 has been initialized

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GPS Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
  GPS.begin(9600);
  // uncomment this line to turn on RMC
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but
  // either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ROS Configuration ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //ROS initialization of the publishers and subscribers
  nh.initNode();
  nh.getHardware()->setBaud(57600);

  nh.subscribe(sub_to_cmd_vel);

  nh.advertise(IMU_eulerOrientation);
  nh.advertise(IMU_angularVelocity);
  nh.advertise(IMU_linearAcceleration);
  nh.advertise(IMU_absoluteOrientation);
  nh.advertise(IMU_magVec);

  nh.advertise(gpsPub);

  while (!nh.connected()) {
    nh.spinOnce();
  }
}

void loop() {
  //Collecting the data from the BNO055
  if (((millis() - publish_imu_time) >= 1000 / IMU_PUBLISH_RATE))
  {
    updateIMU();
    publish_imu_time = millis();
  }

  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }

  //this block publishes velocity based on defined rate
  if ((millis() - publish_gps_time) >= (1000 / GPS_PUBLISH_RATE))
  {
    navSat_msg.latitude = convertGPS(GPS.latitude);
    navSat_msg.longitude = convertGPS(GPS.longitude);
    navSat_msg.altitude = GPS.altitude * 100; // Converting to [m] from [cm]
    navSat_msg.status.status = GPS.fix;
    navSat_msg.status.service = GPS.fixquality;
    gpsPub.publish(&navSat_msg);

    publish_gps_time = millis();
  }

 
  nh.spinOnce(); // Ensure remains connected to Ros_Serial Server

  if(millis() - motor_update > MIN_UPDATE_RATE){ // Stop the motors if they haven't been updated in 1 second
    analogWrite(R_ESC, (unsigned short)(STOP_PWM));
    analogWrite(L_ESC, (unsigned short)(STOP_PWM));
  }
 
}

void updateIMU(void) {
  sensors_event_t oData , aData , lData, event;
 
  bno.getEvent(&oData, Adafruit_BNO055::VECTOR_EULER);
  IMU_eulerOrientationMsg.header.stamp = nh.now();
 
  bno.getEvent(&aData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  IMU_angularVelocityMsg.header.stamp = nh.now();
 
  bno.getEvent(&lData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  IMU_linearAccelerationMsg.header.stamp = nh.now();
 
  // Get absolute orientation
  bno.getEvent(&event);
  IMU_absoluteOrientationMsg.header.stamp = nh.now();

  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  IMU_magVecMsg.header.stamp = nh.now();

  //Updating the orientation publisher
  IMU_eulerOrientationMsg.vector.x = oData.orientation.x;  // values in Euler angles or 'degrees', from 0..359
  IMU_eulerOrientationMsg.vector.y = oData.orientation.y;  // values in Euler angles or 'degrees', from 0..359
  IMU_eulerOrientationMsg.vector.z = oData.orientation.z;  // values in Euler angles or 'degrees', from 0..359

  //Updating the angular velocity publisher
  IMU_angularVelocityMsg.vector.x = aData.gyro.x;  // values in rps, radians per second
  IMU_angularVelocityMsg.vector.y = aData.gyro.y;  // values in rps, radians per second
  IMU_angularVelocityMsg.vector.z = aData.gyro.z;  // values in rps, radians per second

  //Updating the linear acceleration publisher
  IMU_linearAccelerationMsg.vector.x = lData.acceleration.x; // meters/second^2
  IMU_linearAccelerationMsg.vector.y = lData.acceleration.y; // meters/second^2
  IMU_linearAccelerationMsg.vector.z = lData.acceleration.z; // meters/second^2

  //Updating the absolute orientation acceleration - the y and z values have to be switches here
  IMU_absoluteOrientationMsg.vector.x = event.orientation.z; // deg
  IMU_absoluteOrientationMsg.vector.y = event.orientation.y; // deg
  IMU_absoluteOrientationMsg.vector.z = (event.orientation.x > 180) ? (event.orientation.x - 360) : (event.orientation.x); // deg

  //Updating the linear acceleration publisher
  IMU_magVecMsg.vector.x = magnet.x(); // uT
  IMU_magVecMsg.vector.y = magnet.y(); // uT
  IMU_magVecMsg.vector.z = magnet.z(); // uT
 

  //Publishing the orientation, angular velocity, and linear acceleration Vectors
  IMU_eulerOrientation.publish(&IMU_eulerOrientationMsg);
  IMU_angularVelocity.publish(&IMU_angularVelocityMsg);
  IMU_linearAcceleration.publish(&IMU_linearAccelerationMsg);
  IMU_absoluteOrientation.publish(&IMU_absoluteOrientationMsg);
  IMU_magVec.publish(&IMU_magVecMsg);
}

// Converts the GPS coordinates into degrees from degrees/minutes/secondss
float convertGPS(float f) {
 int firsttwodigits = ((int)f)/100; //This assumes that f < 10000.
 float nexttwodigits = f - (float)(firsttwodigits*100);
 float theFinalAnswer = (float)(firsttwodigits + nexttwodigits/60.0);
 return theFinalAnswer;
}
