/**
 *  @file RoboMotors.h
 *
 *  @brief Class that handles the operation of PWM driven motors and steering servo.
 *
 *  Created on: Apr 24, 2022
 *      Author: Wm. T. Thompson
 */

#ifndef ROBOMOTORS_H_
#define ROBOMOTORS_H_
#include "Arduino.h"
#include <AFMotor.h>
#include <Servo.h>
#include "Common.h"

#define ANGLE_RIGHT 45 /**< Right angle value to be used by steering servo. */
#define ANGLE_CENTER 57 /**< Center angle value to be used by steering servo. */
#define ANGLE_LEFT 70 /**< Left angle value to be used by steering servo. */

#define SPEED_FAST 254 /**< Fast speed PWM value for motors. */
#define SPEED_SLOW 190 /**< Slow speed PWM value for motors. */

/**
 *  RoboMotors class encapsulates the operation of moving the robot motors.
 *
 */
class RoboMotors {
public:
	/**
	 * @param servo_pin an int that corresponds to the pin assigned to the steering servo. Default is pin 9.
	 */
	RoboMotors(int servo_pin = 9);

	/**
	 *  @brief The setup method is intended to be called in the setup function of the main Arduino file.
	 *
	 *  @param servo_in is the reference to the Servo object to be used for the steering servo.
	 *  @param motor_in is the reference to the AF_DCMotor object to be used for the motor driver.
	 *  @param motor3_in is another reference to an AF_DCMotor object to be used for the motor driver.
	 */
	void setup(Servo &servo_in, AF_DCMotor &motor_in, AF_DCMotor &motor3_in);

	/**
	 *  @brief The command method is used to command steering and forward/reverse/stop as well as fast or slow motor operation.
	 *
	 *  @param action is the reference to the Action object to be used for commanding steering and forward/reverse/stop operation.
	 */
	void command(Action &action);

	/**
	 *  @brief The print_action method is used print the command to the serial monitor forward/reverse/stop as well as fast or slow motor operation.
	 *
	 *  @param action is reference to the Action object to be printed to the serial monitor.
	 */
	void print_action(Action &action);
	virtual ~RoboMotors();
private:
	AF_DCMotor *m_motor;
	AF_DCMotor *m_motor3;
	int m_servo_pin;
	Servo *m_servo;
};

#endif /* ROBOMOTORS_H_ */
