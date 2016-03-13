/*****************************************************************************/
// Simple remote controlled robot using Genuino 101 (LE stack on the Intel Curie chip)
// Author: Anthony Abbot
// Date: 13 Mar 2016
// 
// This code has been developed for use by the Olney Coder Dojo club
//
// This requires the following components:
//  - An Arduino Genuino 101
//  - An AdaFruit Motor Shield v2.3
//  - A castor car with two powered wheels
//  - A HC-SR04 ultrasonic sensor mounted on...
//  - A servo (so it can look around)
//
/*****************************************************************************/
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <CurieBLE.h>
#include <CurieUart.h>
#include <CuriePing.h>

// Calibrated angles for the sensor
#define ANGLE_CENTRE        85
#define ANGLE_MIN           10
#define ANGLE_MAX           165

// Motor speeds
#define SPEED_MAX           130
#define SPEED_SLOW          80

// Loop delay
#define LOOP_DURATION       20000       

// Create an instance of the motor shield and two motors
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *lMotor = AFMS1.getMotor(3);
Adafruit_DCMotor *rMotor = AFMS1.getMotor(4);

Madgwick filter; // initialise Madgwick object
int ax, ay, az;
int gx, gy, gz;
float yaw;
float pitch;
float roll;
int factor = 800; // variable by which to divide gyroscope values, used to control sensitivity
// note that an increased baud rate requires an increase in value of factor

// Create the pinger. Send on 2, echo back on 3
CuriePing pinger(2, 3);

// Create the servo control
Servo servo;
uint8_t servoangle;
uint16_t distance;

// Create the UART connection
CurieUart uart("dojoCurie");    // Create a named UART peripheral

String data;                    // Data received from the LE device

/*****************************************************************************/
// Setup function - runs once at startup
void setup() 
{
  while(!Serial);
  Serial.begin(9600);  

  // initialize device
  CurieIMU.begin();

  Serial.print("Starting Gyroscope calibration...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");
  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieIMU.setGyroOffsetEnabled(true);
  CurieIMU.setAccelOffsetEnabled(true); 
  
  Serial.print("Initialising UART...");
  uart.begin();
  Serial.println("Done");

  Serial.print("Initialising Motor control...");
  AFMS1.begin();              // Start the motor control
  lMotor->setSpeed(SPEED_MAX);
  lMotor->run(RELEASE);
  rMotor->setSpeed(SPEED_MAX);
  rMotor->run(RELEASE);
  Serial.println("Done");

  Serial.print("Initialising servo...");
  servo.attach(9);
  servoangle = ANGLE_CENTRE;
  servo.write(servoangle);       // centre the sensor
  Serial.println("Done");
}

/*****************************************************************************/
// Main loop - runs repeatedly
void loop() 
{  
  BLECentral central = uart.central();
  if (!central) 
  {
    // Stop the motors
    lMotor->run(RELEASE);
    rMotor->run(RELEASE);
    
    return;
  }
  
  // Loop while connected
  while (central.connected()) 
  {
    unsigned int timer = micros();
    distance = pinger.ping();

    // read raw accel/gyro measurements from device
    CurieIMU.readMotionSensor(ax, ay, az, gx, gy, gz); 
  
    // use function from MagdwickAHRS.h to return quaternions
    filter.updateIMU(gx / factor, gy / factor, gz / factor, ax, ay, az);  
  
    // functions to find yaw roll and pitch from quaternions
    yaw = filter.getYaw();
    roll = filter.getRoll();
    pitch = filter.getPitch();

    //nMake the sensor always point in one direction
    servoangle = ANGLE_CENTRE + (yaw / 3.1415 * 180.0);
    servoangle = constrain(servoangle, ANGLE_MIN, ANGLE_MAX);
    servo.write(servoangle);
    
    // Check if the uart has a string 
    if (uart.hasString())
    {
      data = uart.getString();
      Serial.println(data);

      if (data == "Play")
      {
        lMotor->setSpeed(SPEED_MAX);
        rMotor->setSpeed(SPEED_MAX);
        lMotor->run(FORWARD);
        rMotor->run(FORWARD);        
      }
      else if (data == "Pause")
      {
        lMotor->setSpeed(SPEED_MAX);
        rMotor->setSpeed(SPEED_MAX);
        lMotor->run(BACKWARD);
        rMotor->run(BACKWARD);        
      }
      else if (data == "Rewind")
      {
        lMotor->setSpeed(SPEED_SLOW);
        rMotor->setSpeed(SPEED_SLOW);
        lMotor->run(BACKWARD);
        rMotor->run(FORWARD);        
      }
      else if (data == "FastForward")
      {
        lMotor->setSpeed(SPEED_SLOW);
        rMotor->setSpeed(SPEED_SLOW);
        lMotor->run(FORWARD);
        rMotor->run(BACKWARD);        
      }
      else
      {
        lMotor->run(RELEASE);
        rMotor->run(RELEASE);        
        
      }
    } 

    // Force a fixed delay
    unsigned int timeleft = (LOOP_DURATION) - (micros() - timer);
    timeleft = constrain(timeleft, 0, LOOP_DURATION);
    delayMicroseconds(timeleft);
    Serial.println(timeleft);
  }

  // Stop the motors
  lMotor->run(RELEASE);
  rMotor->run(RELEASE);
}
