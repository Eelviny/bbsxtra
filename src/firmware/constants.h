#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

#include "intellisense.h"
#include <stdint.h>
#include <stdbool.h>

#define ASSIST_FLAG_NONE				0x00
#define ASSIST_FLAG_CRUISE				0x04
#define ASSIST_FLAG_PAS_VARIABLE		0x08	// pas mode using throttle to set power level
#define ASSIST_FLAG_OVERRIDE_CADENCE	0x20	// pas option where max cadence is set to 100% when throttle overrides pas

#define ASSIST_MODE_SELECT_OFF			0x00
#define ASSIST_MODE_SELECT_STANDARD		0x01
#define ASSIST_MODE_SELECT_LIGHTS		0x02
#define ASSIST_MODE_SELECT_PAS0_LIGHT	0x03
#define ASSIST_MODE_SELECT_PAS1_LIGHT	0x04
#define ASSIST_MODE_SELECT_PAS2_LIGHT	0x05
#define ASSIST_MODE_SELECT_PAS3_LIGHT	0x06
#define ASSIST_MODE_SELECT_PAS4_LIGHT	0x07
#define ASSIST_MODE_SELECT_PAS5_LIGHT	0x08
#define ASSIST_MODE_SELECT_PAS6_LIGHT	0x09
#define ASSIST_MODE_SELECT_PAS7_LIGHT	0x0A
#define ASSIST_MODE_SELECT_PAS8_LIGHT	0x0B
#define ASSIST_MODE_SELECT_PAS9_LIGHT	0x0C
#define ASSIST_MODE_SELECT_BRAKE_BOOT	0x0D


#define TEMPERATURE_SENSOR_CONTR		0x01
#define TEMPERATURE_SENSOR_MOTOR		0x02

#define WALK_MODE_DATA_SPEED			0
#define WALK_MODE_DATA_TEMPERATURE		1
#define WALK_MODE_DATA_REQUESTED_POWER	2
#define WALK_MODE_DATA_BATTERY_PERCENT	3

#define THROTTLE_GLOBAL_SPEED_LIMIT_DISABLED	0
#define THROTTLE_GLOBAL_SPEED_LIMIT_ENABLED		1
#define THROTTLE_GLOBAL_SPEED_LIMIT_STD_LVLS	2

#define LIGHTS_MODE_DEFAULT						0
#define LIGHTS_MODE_DISABLED					1
#define LIGHTS_MODE_ALWAYS_ON					2
#define LIGHTS_MODE_BRAKE_LIGHT					3
#define LIGHTS_MODE_DEFAULT_AND_BRAKE_LIGHT		4 //When light is off, the light will be used as brake light, When its turned on it stays on

#define CONFIG_VERSION					5
#define PSTATE_VERSION					1


typedef struct
{
	uint8_t flags;
	uint8_t max_cadence_percent;
	uint8_t max_pas_speed_kph;
	uint16_t target_power_watts;
	uint8_t max_throttle_speed_kph;
	uint16_t max_throttle_power_watts;

}  assist_level_t;

#if defined(BBSHD)
	#define HAS_MOTOR_TEMP_SENSOR				1
#else
	#define HAS_MOTOR_TEMP_SENSOR				0
#endif

#define HAS_CONTROLLER_TEMP_SENSOR				1

#if defined(BBS02)
	#define MAX_CADENCE_RPM_X10					1500
#elif defined(BBSHD)
	// Measured on BBSHD at 48V
	#define MAX_CADENCE_RPM_X10					1680
#else
	#define MAX_CADENCE_RPM_X10					1200
#endif

#define PAS_PULSES_REVOLUTION					24

// No battery percent mapping
#define BATTERY_PERCENT_MAP_NONE				0
// Map battery percent to provide a linear relationship on the
// 5-bar battery indicator of the SW102 display.
#define BATTERY_PERCENT_MAP_SW102				1

#define THROTTLE_RESPONSE_LINEAR				1
#define THROTTLE_RESPONSE_QUADRATIC				2
#define THROTTLE_RESPONSE_CUSTOM				3

// Custom throttle map
// y = pow(x / 100.0, 1.5) * 100.0
#define THROTTLE_CUSTOM_MAP						\
	0,  1,  1,  1,  2,  2,  2,  3,  3,  3,		\
	4,  4,  4,  5,  5,  6,  6,  7,  8,  8,		\
	9,  10, 10, 11, 12, 12, 13, 14, 15, 16,		\
	16, 17, 18, 19, 20, 21, 22, 23, 23, 24,		\
	25, 26, 27, 28, 29, 30, 31, 32, 33, 34,		\
	35, 36, 37, 39, 40, 41, 42, 43, 44, 45,		\
	46, 48, 49, 50, 51, 52, 54, 55, 56, 57,		\
	59, 60, 61, 62, 64, 65, 66, 68, 69, 70,		\
	72, 73, 74, 76, 77, 78, 80, 81, 83, 84,		\
	85, 87, 88, 90, 91, 93, 94, 96, 97, 99,		\
	100

	// Number of PAS sensor pulses to engage cruise mode,
// there are 24 pulses per revolution.
#define CRUISE_ENGAGE_PAS_PULSES				PAS_PULSES_REVOLUTION / 2

// Number of PAS sensor pulses to disengage curise mode
// by pedaling backwards.
#define CRUISE_DISENGAGE_PAS_PULSES				PAS_PULSES_REVOLUTION / 2


// Option to control what data is displayed in "Range" field on display
// since range calculation is not implemented.
#define DISPLAY_RANGE_FIELD_ZERO				0
#define DISPLAY_RANGE_FIELD_TEMPERATURE			1	// max temperature of controller / motor
#define DISPLAY_RANGE_FIELD_POWER				2	// requested current x10 (lights off) / actual current x10 (lights on)

// uncomment and select option above
// #define DISPLAY_RANGE_FIELD_DATA		DISPLAY_RANGE_FIELD_ZERO

// default to temperature if temperature sensors available (BBS2/BBSHD), else power
#ifndef DISPLAY_RANGE_FIELD_DATA
		#if HAS_CONTROLLER_TEMP_SENSOR || HAS_MOTOR_TEMP_SENSOR
			#define DISPLAY_RANGE_FIELD_DATA		DISPLAY_RANGE_FIELD_TEMPERATURE
		#else
			#define DISPLAY_RANGE_FIELD_DATA		DISPLAY_RANGE_FIELD_POWER
		#endif
#endif

#endif
