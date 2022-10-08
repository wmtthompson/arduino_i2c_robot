/*
 * RoboMotors.h
 *
 *  Created on: Apr 24, 2022
 *      Author: Tom
 */

#ifndef ROBOMOTORS_H_
#define ROBOMOTORS_H_
#include "Arduino.h"
#include "common.h"
#include <AFMotor.h>
#include <Servo.h>

//The following are angle values to be used by the servo for commanding direction
#define ANGLE_RIGHT 45
#define ANGLE_CENTER 57
#define ANGLE_LEFT 70

//The following are PWM values
#define SPEED_FAST 254
#define SPEED_SLOW 190


class RoboMotors {
public:
	RoboMotors(int servo_pin = 9);
	void command(Action &action);
	void print_action(Action &action);
	void setup(Servo &servo_in, AF_DCMotor &motor_in, AF_DCMotor &motor3_in);
	virtual ~RoboMotors();
private:
	AF_DCMotor *m_motor;
	AF_DCMotor *m_motor3;
	int m_servo_pin;
	Servo *m_servo;
};

#endif /* ROBOMOTORS_H_ */
