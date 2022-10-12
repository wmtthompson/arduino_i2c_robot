/*
 * RoboMotors.cpp
 *
 *  Created on: Apr 24, 2022
 *      Author: william
 */

#include "RoboMotors.h"


RoboMotors::RoboMotors(int servo_pin = 9) {

	m_servo_pin = servo_pin;
	m_motor = NULL;
	m_motor3 = NULL;
	m_servo = NULL;
}

void RoboMotors::setup(Servo &servo_in, AF_DCMotor &motor_in, AF_DCMotor &motor3_in)
{
	m_servo = &servo_in;
	m_servo->attach(m_servo_pin);
	m_motor = &motor_in;
	m_motor3 = &motor3_in;
}

void RoboMotors::print_action(Action &action)
{
	Serial.print("direction = ");
	switch(action.direction)
	{
	case FWD:
		Serial.print("FORWARD");
		break;
	case REV:
		Serial.print("REVERSE");
		break;
	case STOP:
		Serial.print("STOP");
		break;
	default:
		break;
	}
	Serial.print(", speed = ");
	switch(action.speed)
	{
	case SLOW:
		Serial.print("SLOW");
		break;
	case FAST:
		Serial.print("FAST");
		break;
	default:
		break;
	}
	Serial.print(", steering = ");
	switch(action.steer)
	{
	case L:
		Serial.print("LEFT");
		break;
	case R:
		Serial.print("RIGHT");
		break;
	case S:
		Serial.print("STRAIGHT");
		break;
	default:
		break;
	}
	Serial.println("");
}

void RoboMotors::command(Action &action)
{
	switch(action.steer)
	{
	case L:
		m_servo->write(ANGLE_LEFT);
		break;
	case R:
		m_servo->write(ANGLE_RIGHT);
		break;
	case S:
		m_servo->write(ANGLE_CENTER);
		break;
	default:
		break;
	}

	switch(action.speed)
	{
	case SLOW:
		m_motor->setSpeed(SPEED_SLOW);
		m_motor3->setSpeed(SPEED_SLOW);
		break;
	case FAST:
		m_motor->setSpeed(SPEED_FAST);
		m_motor3->setSpeed(SPEED_FAST);
		break;
	default:
		break;
	}

	switch(action.direction)
	{
	case FWD:
		m_motor->run(FORWARD);
		m_motor3->run(FORWARD);
		break;
	case REV:
		m_motor->run(BACKWARD);
		m_motor3->run(BACKWARD);
		break;
	case STOP:
		m_motor->run(RELEASE);
		m_motor3->run(RELEASE);
		break;
	default:
		break;
	}
	switch(action.speed)
	{
	case SLOW:
		delay(800);
		break;
	case FAST:
		delay(1000);
		break;
	default:
		break;
	}


	m_motor->run(RELEASE);
	m_motor3->run(RELEASE);
}

RoboMotors::~RoboMotors() {
	// Auto-generated destructor stub
}

