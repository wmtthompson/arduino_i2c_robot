/**
 *  @file ScanSensor.h
 *
 *	@brief Class that handles scanning using the ultrasonic ping sensor and a pan servo.
 *
 *  Created on: Apr 22, 2022
 *      Author: Wm. T. Thompson
 */

#ifndef SCANSENSOR_H_
#define SCANSENSOR_H_

#include "Arduino.h"
#include <Servo.h>
#include "Common.h"

#define THRESHOLD_VERY_CLOSE 8  /**< 8 cm for when the robot is very close to an obstacle. */
#define THRESHOLD_NEAR 15 /**< 15 cm for when the robot is just nearby an obstacle*/

/** Azimuth sectors. */
enum Sector
{
	LEFT, RIGHT, CENTER
};

/** Stages of the scan progress. */
enum Scan_Direction
{
	FORWARD_SCAN, REVERSE_SCAN, END_SCAN
};

/**
 *  ScanSensor class encapsulates the operation of scanning the area in front of the robot and returning results.
 *
 */
class ScanSensor
{
public:

	/**
	 * @param servo_pin an int that corresponds to the pin assigned to the panning servo.
	 */
	ScanSensor(int servo_pin);

	/**
	 *  @brief The setup method is intended to be called in the setup function of the main Arduino file.
	 *
	 *  @param servo_in is the Servo object to be used for panning the scan head.
	 */
	void setup(Servo &servo_in);

	/**
	 *  @brief The get_scan method updates a reference to the latest scan results. Scan is performed on demand.
	 *
	 *  @param ScanState s1 is a reference to a ScanState instance to be updated.
	 */
	void get_scan(ScanState &s1);

	/**
	 *  @brief The print_scan method prints the contents of the ScanState object passed to it.
	 *
	 *  @param ScanState s1 is a reference to a ScanState instance to be printed.
	 */
	void print_scan(ScanState &s1);


	virtual ~ScanSensor();

private:
	// The get_average_measurement method returns a running average of distance measurement.
	float get_average_measurement();

	// Utility function that goes with the ping sensor, converts microseconds to centimeters.
	long microsecondsToCentimeters(long microseconds);

	// Returns the results of reading the ping sensor.
	long read_ping();

	// Returns a Scan enum value that corresponds to the centimeter measurement.
	Scan categorize_distance(float cm);

	// Returns the Sector enum result of categorizing a sector.
	Sector categorize_sector(int position);

	// Returns a Sector enum based on the passed in scan_dir and current_sector.
	Sector evaluate_sector(Scan_Direction scan_dir, Sector current_sector,
			int *pos, ScanState &s1);

	// Servo pin that will be attached, used to operate pan servo.
	int m_servo_pin;

	// Servo member object instance used to hold data relavent to the pan servo.
	Servo *m_servo;
};

#endif /* SCANSENSOR_H_ */
