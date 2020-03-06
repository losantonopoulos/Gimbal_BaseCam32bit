#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <sstream>
#include <string.h> 

#define DEFAULT_PORT "/dev/ttyUSB1"

#define GIMBAL_ANGLE_RATE_UNIT  0.1220740379
#define GIMBAL_ANGLE_UNIT       0.02197265625

// ROLL
#define GIMBAL_MAX_ROLL_ANGLE   45.0   // deg
#define GIMBAL_MIN_ROLL_ANGLE  -45.0   // deg
#define GIMBAL_MAX_ROLL_SPEED   50.0   // deg/sec
#define GIMBAL_MIN_ROLL_SPEED  -50.0   // deg/sec

// PITCH
#define GIMBAL_MAX_PITCH_ANGLE   60.0   // deg
#define GIMBAL_MIN_PITCH_ANGLE  -60.0   // deg
#define GIMBAL_MAX_PITCH_SPEED   90.0   // deg/sec
#define GIMBAL_MIN_PITCH_SPEED  -90.0   // deg/sec

// YAW
#define GIMBAL_MAX_YAW_ANGLE   90.0   // deg
#define GIMBAL_MIN_YAW_ANGLE  -90.0   // deg
#define GIMBAL_MAX_YAW_SPEED   90.0   // deg/sec
#define GIMBAL_MIN_YAW_SPEED  -90.0   // deg/sec

class Gimbal {

	public:

		bool engage_gimbal();

		bool disengage_gimbal();

		void set_gimbal_speed(double roll_speed, double pitch_speed, double yaw_speed);

		void set_gimbal_angle(double roll_angle, double pitch_angle, double yaw_angle);


		void set_gimbal_angle_speed(double roll_speed,  double roll_angle, 
		                double pitch_speed, double pitch_angle,
		                double yaw_speed,   double yaw_angle);

		bool connect_gimbal();

		bool connect_gimbal(const char *port);

		void disconnect_gimbal();


	private:

		void 	INFO(const char *msg);

		int16_t constrain(int16_t value, int16_t min_val, int16_t max_val);

		int 	constrain(int value, int min_val, int max_val);

		double 	constrain(double value, double min_val, double max_val);

};

#endif