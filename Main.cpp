/**
 *  @file Main.cpp
 *
 *	@brief Main entry point for Robot Program. This program uses the old Arduino L293D motor shield.
 *
 *  Created on: Apr 22, 2022
 *      Author: Wm. T. Thompson
 */

#include <Arduino.h>
#include <AFMotor.h>
#include <Servo.h>
#include "ScanSensor.h"
#include "RoboMotors.h"
#include <Wire.h>

#define SCAN_SENSOR_PAN_SERVO_PIN 10 /**< Pan servo pin */
#define MOTOR1_NUM_ON_MOTOR_SHIELD 4 /**< Motor shield motor number 4 */
#define MOTOR3_NUM_ON_MOTOR_SHIELD 3 /**< Motor shield motor number 3 */

ScanSensor scs1(10);

ScanState ss2;
Servo scan_servo;
RoboMotors robot1;

Servo steering_servo;
AF_DCMotor motor(4);
AF_DCMotor motor3(3);

enum RobotMode {SCANNING, MOVING};


char temp_buffer[13];

Action a1;

float rew1;

RobotMode mode1 = SCANNING;
volatile uint8_t command_received = 0;

void receiveEvent(int howMany);
void requestEvent();
void get_next_state(ScanState &ss1);

void setup() {
	  Serial.begin(9600);
	  while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB port only
	  }
	  scs1.setup(scan_servo);
	  robot1.setup(steering_servo, motor, motor3);
	  Wire.begin(8);
	  Wire.onReceive(receiveEvent);
	  Wire.onRequest(requestEvent);
	  a1.direction = STOP;
	  a1.speed = FAST;
	  a1.steer = R;

	  ss2.left = NEAR;
	  ss2.right = NEAR;
	  ss2.center = FAR;

	  Serial.print("Size of Action is = ");
	  Serial.println(sizeof(a1));
}

void loop() {
//	scs1.get_scan(ss2);
//	delay(1000);
	switch(mode1)
	{
	case SCANNING:
		scs1.get_scan(ss2);
		Serial.println("In SCANNING MODE");
		scs1.print_scan(ss2);
		if (command_received == 1)
		{
			mode1 = MOVING;
		}
		break;
	case MOVING:
		robot1.command(a1);
		command_received = 0;
		Serial.println("In MOVING MODE");
		mode1 = SCANNING;
		break;
	default:
		break;
	}
	delay(1000);
}

void receiveEvent(int howMany)
{

	char c;
	if (Wire.available() >= sizeof(Action))
	{

		for(int i = 0; i < sizeof(Action); i++)
		{
			temp_buffer[i] = Wire.read();
		}

		memcpy((void *)&a1, (void *)temp_buffer, sizeof(Action));
		robot1.print_action(a1);

		while (Wire.available() > 0 )
		{
			c = Wire.read();
		}
		command_received = 1;
	}

}

void requestEvent()
{

	memset((void *)temp_buffer, 0, 13);
	memcpy((void *)temp_buffer, (void *)&ss2, sizeof(ScanState));
	Wire.write(temp_buffer, sizeof(ScanState));

}

void get_next_state(ScanState &ss1)
{
	uint8_t left = static_cast<uint8_t>(random(0,3));
	uint8_t right = static_cast<uint8_t>(random(0,3));
	uint8_t center = static_cast<uint8_t>(random(0,3));

	ss1.left = static_cast<Scan>(left);
	ss1.right = static_cast<Scan>(right);
	ss1.center = static_cast<Scan>(center);
}
