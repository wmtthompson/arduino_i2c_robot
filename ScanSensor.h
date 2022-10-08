/*
 *  @file ScanSensor.h
 *
 *	@brief Class that handles scanning using the ping ultrasonic ping sensor and a pan servo.
 *
 *  Created on: Apr 22, 2022
 *      Author: Tom
 */

#ifndef SCANSENSOR_H_
#define SCANSENSOR_H_

#include "Arduino.h"
#include "common.h"
#include <Servo.h>

#define THRESHOLD_VERY_CLOSE 8  //8 cm
#define THRESHOLD_NEAR 15 //15 cm

enum Sector {LEFT, RIGHT, CENTER};
enum Scan_Direction {FORWARD_SCAN, REVERSE_SCAN, END_SCAN};

class ScanSensor {
public:

	ScanSensor(int servo_pin);
	void setup(Servo &servo_in);
	void get_scan(ScanState &s1);
	void print_scan(ScanState &s1);
	virtual ~ScanSensor();

private:
	float get_average_measurement();
	long microsecondsToCentimeters(long microseconds);
	long read_ping();
	Scan categorize_distance(float cm);
	Sector categorize_sector(int position);
	Sector evaluate_sector(Scan_Direction scan_dir, Sector current_sector, int *pos, ScanState &s1);
	int m_servo_pin;
	Servo *m_servo;
};

#endif /* SCANSENSOR_H_ */
