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

#define SCAN_SENSOR_PAN_SERVO_PIN 10 /**< Pan servo pin. */
#define MOTOR1_NUM_ON_MOTOR_SHIELD 4 /**< Motor shield motor number 4. */
#define MOTOR3_NUM_ON_MOTOR_SHIELD 3 /**< Motor shield motor number 3. */

ScanSensor scan_sensor1(10); /**< Scan Sensor 1, initialized to use pin 10. */

ScanState scan_state1; /**< Scan State struct instance. */
Servo scan_servo; /**< Servo object instance for use in scanning. */
RoboMotors robot_motors1; /**< RobotMotors object instance. */

Servo steering_servo; /**< Servo object instance for use in steering. */

AF_DCMotor af_motor1(4); // Adafruit DCMotor object instance for use in main driving wheels. */

AF_DCMotor af_motor2(3); // Adafruit DCMotor object instance for use in main driving wheels. */

enum RobotMode {SCANNING, MOVING}; /**< RobotMode enum to for robot state.*/


char buffer[13]; /**< A buffer to hold data read from I2C.*/

Action action1; /**< An Action object instance used to manage commands.*/

RobotMode mode1 = SCANNING; /**< Robot state enum, initialized to SCANNING.*/

uint8_t command_received = 0; /**< A logical variable to keep track of commands received.*/

/**
 *  @brief The receiveEvent function handles an I2C receive event, in this case used to receive movement command.
 *
 *  @param int howMany is the number of bytes received.
 */
void receiveEvent(int howMany);

/**
 *  @brief The requestEvent function handles an I2C request event. In this case, used to request scan data.
 *
 */
void requestEvent();

/**
 *  @brief The setup function to be called to initialize variables pin modes, serial etc.
 *
 */
void setup() {
	  Serial.begin(9600);
	  while (!Serial) {
	    ; // wait for serial port to connect. Needed for native USB port only
	  }
	  scan_sensor1.setup(scan_servo);
	  robot_motors1.setup(steering_servo, af_motor1, af_motor2);
	  Wire.begin(8);
	  // Register the callbacks for I2C.
	  Wire.onReceive(receiveEvent);
	  Wire.onRequest(requestEvent);

	  // Initializing
	  action1.direction = STOP;
	  action1.speed = FAST;
	  action1.steer = R;

	  // Initializing
	  scan_state1.left = NEAR;
	  scan_state1.right = NEAR;
	  scan_state1.center = FAR;
}


void loop() {

	// Switchyard for the state machine
	switch(mode1)
	{
	case SCANNING: // scanning mode will do a scan unless otherwise commanded
		scan_sensor1.get_scan(scan_state1);
		Serial.println("In SCANNING MODE");
		scan_sensor1.print_scan(scan_state1);
		if (command_received == 1) //once a command is received, now in MOVING mode.
		{
			mode1 = MOVING;
		}
		break;
	case MOVING:
		robot_motors1.command(action1);
		command_received = 0;
		Serial.println("In MOVING MODE");
		mode1 = SCANNING;
		break;
	default:
		break;
	}
	delay(1000); // need to have some time for events to be processed.
}

void receiveEvent(int howMany)
{

	char dummy_character;

	if (Wire.available() >= sizeof(Action))
	{

		for(int i = 0; i < sizeof(Action); i++)
		{
			buffer[i] = Wire.read();
		}

		memcpy((void *)&action1, (void *)buffer, sizeof(Action));
		robot_motors1.print_action(action1);

		while (Wire.available() > 0 ) //empty the buffer if theres still some garbage.
		{
			dummy_character = Wire.read();
		}
		command_received = 1; // set command received to 1, so that MOVING mode will be next.
	}

}

void requestEvent()
{

	memset((void *)buffer, 0, 13);
	// Read the latest scan data and send it back on the I2C.
	memcpy((void *)buffer, (void *)&scan_state1, sizeof(ScanState));
	Wire.write(buffer, sizeof(ScanState));

}
