/*****************************************************************************/
// Simpke remote controlled robot using Genuino 101 (LE stack on the Intel Curie chip)
// Author: Anthony Abbot
// Date: 13 Mar 2016
// 
// This code has been developed for use by the Olney Coder Dojo club
//
// This requires the following components:
//  - An Arduino Genuino 101
//  - An AdaFruit Motor Shield v2.3
//  - A car with two wheels
//  - A HC-SR04 ultrasonic sensor mounted on...
//  - A servo
//
/*****************************************************************************/

#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <CurieBLE.h>
#include <CurieUart.h>
#include <CuriePing.h>

// Calibrated angles for the sensor
#define ANGLE_CENTRE        85
#define ANGLE_MIN           10
#define ANGLE_MAX           165

// Create an instance of the motor shield and two motors
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x61);
Adafruit_DCMotor *lMotor = AFMS1.getMotor(3);
Adafruit_DCMotor *rMotor = AFMS1.getMotor(4);

// Create the pinger
CuriePing pinger(2, 3);

// Create the servo control
Servo servo;
uint8_t servoangle;

// Create the UART connection
CurieUart uart("dojoCurie");    // Create a named UART peripheral

String str;                     // Data read from the local serial input
String data;                    // Data received from the LE device

/*****************************************************************************/
// Setup function - runs once at startup
void setup() 
{
  while(!Serial);
  Serial.begin(9600);  
  
  Serial.print("Initialising UART...");
  uart.begin();

  Serial.print("Initialising Motor control...");
  AFMS1.begin();              // Start the motor control
  lMotor->setSpeed(150);
  lMotor->run(RELEASE);
  rMotor->setSpeed(150);
  rMotor->run(RELEASE);

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
  
  Serial.print("Connected to central: ");
  Serial.println(central.address());

  // Loop while connected
  while (central.connected()) 
  {
    
    // Check if the uart has a string and print to the serial console
    if (uart.hasString())
    {
      data = uart.getString();
      Serial.print("<-- ");
      Serial.println(data);

      int dist = pinger.ping();
      String ret{">" + data};
      uart.sendString(ret);
      ret = "dst=" + String(dist);
      uart.sendString(ret);
    } 
  }

//  lMotor->run(RELEASE);
//  rMotor->run(RELEASE);
//  myservo.write(120);


  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}
