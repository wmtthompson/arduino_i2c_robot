/*
 * ScanSensor.cpp
 *
 *  Created on: Apr 22, 2022
 *      Author: william
 */

#include "ScanSensor.h"



ScanSensor::ScanSensor(int servo_pin = 10) {
	m_servo_pin = servo_pin;
	m_servo = NULL;

}

void ScanSensor::setup(Servo &servo_in){
	m_servo = &servo_in;
	m_servo->attach(m_servo_pin);
	//now to setup the ping sensor
	//A0 will be trigger pin
	pinMode(A0, OUTPUT);
	//A1 will be echo pin
	pinMode(A1, INPUT);

}

long ScanSensor::read_ping()
{
	long duration;

	digitalWrite(A0, LOW);
	delayMicroseconds(2);
	digitalWrite(A0, HIGH);
	delayMicroseconds(5);
	digitalWrite(A0, LOW);

	duration = pulseIn(A1, HIGH);
	return duration;
}

void ScanSensor::print_scan(ScanState &s1)
{
	Serial.print("left = ");
	Serial.print(s1.left);
	Serial.print(", right = ");
	Serial.print(s1.right);
	Serial.print(", center = ");
	Serial.print(s1.center);
	Serial.println("");
}

long ScanSensor::microsecondsToCentimeters(long microseconds)
{
	// The speed of sound is 340 m/s or 29 microseconds per centimeter.
	// The ping travels out and back, so to find the distance of the
	// object we take half of the distance travelled.
	return microseconds / 29 / 2;
}

Scan ScanSensor::categorize_distance(float cm)
{
	if (cm < THRESHOLD_VERY_CLOSE)
	{
		return VERY_CLOSE;
	}
	else if (cm < THRESHOLD_NEAR)
	{
		return NEAR;
	}
	else
	{
		return FAR;
	}
}

Sector ScanSensor::categorize_sector(int position)
{
	if (position <= 120)
	{
		return RIGHT;
	}
	else if (position > 120 && position < 150)
	{
		return CENTER;
	}
	else
	{
		return LEFT;
	}
}

float ScanSensor::get_average_measurement()
{
	float avg = 0;
	long duration;
	long cm;
	float cm_f;
	for (int i = 1; i < 6; i++)
	{
		duration = read_ping();
		cm = microsecondsToCentimeters(duration);
		cm_f = static_cast<float>(cm);
		avg = avg + (1.0/i)*(cm_f-avg);
	}
	return avg;
}

Sector ScanSensor::evaluate_sector(const Scan_Direction scan_direction, Sector current_sector, int *pos, ScanState &s1)
{
	float lowest_cm = 500.0;
	float cm = 0;
	Sector next_sector = RIGHT;
	cm = get_average_measurement();

	if (cm < lowest_cm)
	{
		lowest_cm = cm;
	}
	switch (scan_direction)
	{
	case FORWARD_SCAN:
		*pos += 2;
		break;
	case REVERSE_SCAN:
		*pos -= 2;
		break;
	default:
		break;
	}
	if (*pos >= 90 && *pos <= 180)
	{
		m_servo->write(*pos);
	}
	delay(15);
	next_sector = categorize_sector(*pos);
	if (current_sector != next_sector)
	{
		switch (current_sector)
		{
		case LEFT:
			s1.left = categorize_distance(lowest_cm);
			break;
		case RIGHT:
			s1.right = categorize_distance(lowest_cm);
			break;
		case CENTER:
			s1.center = categorize_distance(lowest_cm);
			break;
		default:
			break;
		}

	}
	current_sector = next_sector;
	return current_sector;
}

void ScanSensor::get_scan(ScanState &s1)
{
	int pos = 90;

	Scan_Direction state = FORWARD_SCAN;
	Sector sector = LEFT;

	//move to initial position of 90
	m_servo->write(pos);
	delay(20);
	while (true)
	{
		switch(state)
		{
		case FORWARD_SCAN:
			sector = evaluate_sector(state, sector, &pos, s1);
			if (pos >= 179)
			{
				state = REVERSE_SCAN;
			}
			break;
		case REVERSE_SCAN:
			sector = evaluate_sector(state, sector, &pos, s1);
			if (pos <= 89)
			{
				state = END_SCAN;
			}
			break;
		case END_SCAN:
			break;
		default:
			break;
		}
		if (state == END_SCAN)
		{
			break;
		}
	}
}

ScanSensor::~ScanSensor() {
	// Auto-generated destructor stub
}

