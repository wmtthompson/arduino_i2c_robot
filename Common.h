/**
 *  @file Common.h
 *
 *  @brief This file contains common enumerations and structs to be used for commands and status.
 *
 *  Created on: Apr 20, 2022
 *      Author: Wm. T. Thompson
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "Arduino.h"

/** The Steering enum corresponds to left right and straight ahead steering. */
enum Steering: uint8_t {L = 0, S = 1, R = 2};

/** The Speed enum corresponds to slow or fast speed. */
enum Speed: uint8_t {SLOW = 0, FAST = 1};

/** The Direction enum corresponds to the direction of the motor, forward, reverse, and stop*/
enum Direction: uint8_t {FWD = 0, REV =1, STOP = 2};

/** The Scan enum corresponds to the scan distance categories. */
enum Scan: uint8_t {NEAR = 0, FAR = 1, VERY_CLOSE = 2};

/** The Action struct combines steering, direction and speed. */
struct Action
{
	Steering steer;
	Direction direction;
	Speed speed;
};

/** The ScanState struct has a Scan enum for sectors left, right and center. */
struct ScanState
{
	Scan left;
	Scan right;
	Scan center;
};



#endif /* COMMON_H_ */
