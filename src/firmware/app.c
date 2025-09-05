/*
 * bbs-fw
 *
 * Copyright (C) Daniel Nilsson, 2022.
 *
 * Released under the GPL License, Version 3
 */

#include "app.h"
#include "fwconfig.h"
#include "constants.h"
#include "motor.h"
#include "sensors.h"
#include "throttle.h"
#include "lights.h"
#include "eventlog.h"
#include "util.h"
#include "system.h"


typedef struct
{
	assist_level_t level;

	// cached precomputed values
	// ---------------------------------

	// speed
	int32_t max_pas_wheel_speed_rpm_x10;
	int32_t max_throttle_wheel_speed_rpm_x10;

	// pas
	uint8_t keep_current_target_percent;
	uint16_t keep_current_ramp_start_rpm_x10;
	uint16_t keep_current_ramp_end_rpm_x10;

} assist_level_data_t;

static uint8_t assist_level;
static uint8_t operation_mode;
static uint16_t global_speed_limit_rpm;
static int32_t global_throttle_speed_limit_rpm_x10;

static uint16_t lvc_voltage_x100;
static uint16_t lvc_ramp_down_start_voltage_x100;
static uint16_t lvc_ramp_down_end_voltage_x100;

static uint16_t full_voltage_range_x100;
static uint16_t padded_voltage_range_x100;
static uint16_t low_voltage_pad_x100;
static uint16_t high_voltage_pad_x100;

static assist_level_data_t assist_level_data;

static bool cruise_paused;
static int8_t temperature_contr_c;
static int8_t temperature_motor_c;

static uint16_t ramp_up_current_interval_ms;
static uint32_t power_blocked_until_ms;

static uint16_t pretension_cutoff_speed_rpm_x10;

static bool lights_state = false;

extern const assist_level_t assist_levels[2][10];

void apply_pas_cadence(uint8_t* target_current, uint8_t throttle_percent);

void apply_pretension(uint8_t* target_current);
void apply_cruise(uint8_t* target_current, uint8_t throttle_percent);
uint8_t calculate_current_for_power(uint16_t watts);
bool apply_throttle(uint8_t* target_current, uint8_t throttle_percent);
void apply_speed_limit(uint8_t* target_current, bool throttle_override);
bool apply_thermal_limit(uint8_t* target_current);
bool apply_low_voltage_limit(uint8_t* target_current);
bool apply_shift_sensor_interrupt(uint8_t* target_current);
void set_brake_lights(bool on);
void apply_current_ramp_up(uint8_t* target_current, bool enable);
void apply_current_ramp_down(uint8_t* target_current, bool enable);

bool check_power_block();
void block_power_for(uint16_t ms);

void reload_assist_params();

uint16_t convert_wheel_speed_kph_to_rpm(uint8_t speed_kph, bool display_scaled);

void app_init()
{
	motor_disable();
	lights_disable();
	lights_set(LIGHTS_MODE == LIGHTS_MODE_ALWAYS_ON);

	lvc_voltage_x100 = LOW_CUT_OFF_V * 100u;

	full_voltage_range_x100 = MAX_BATTERY_X100V - lvc_voltage_x100;
	low_voltage_pad_x100 = full_voltage_range_x100 * BATTERY_EMPTY_OFFSET_PERCENT / 100;
	high_voltage_pad_x100 = full_voltage_range_x100 * BATTERY_FULL_OFFSET_PERCENT / 100;
	padded_voltage_range_x100 = full_voltage_range_x100 - low_voltage_pad_x100 - high_voltage_pad_x100;

	// The LVC ramp down end is at 0% battery, which is LVC + the low padding value.
	lvc_ramp_down_end_voltage_x100 = lvc_voltage_x100 + low_voltage_pad_x100;

	// The LVC ramp down starts at LVC_RAMP_DOWN_OFFSET_PERCENT battery, using the padded range.
	uint16_t lvc_ramp_down_offset_x100 = padded_voltage_range_x100 * LVC_RAMP_DOWN_OFFSET_PERCENT / 100;
	lvc_ramp_down_start_voltage_x100 = lvc_ramp_down_end_voltage_x100 + lvc_ramp_down_offset_x100;

	global_speed_limit_rpm = 0;
	global_throttle_speed_limit_rpm_x10 = 0;
	temperature_contr_c = 0;
	temperature_motor_c = 0;

	ramp_up_current_interval_ms = (MAX_CURRENT_AMPS * 10u) / CURRENT_RAMP_AMPS_S;
	power_blocked_until_ms = 0;

	pretension_cutoff_speed_rpm_x10 = convert_wheel_speed_kph_to_rpm(PRETENSION_SPEED_CUTOFF_KPH, false) * 10;

	cruise_paused = true;
	operation_mode = OPERATION_MODE_DEFAULT;

	app_set_assist_level(ASSIST_STARTUP_LEVEL);
	reload_assist_params();

	#if ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_BRAKE_BOOT
	if (brake_is_activated())
	{
		app_set_operation_mode(OPERATION_MODE_SPORT);
	}
	#endif

	#if (LIGHTS_MODE == LIGHTS_MODE_DISABLED /*|| (motor_status() & MOTOR_ERROR_LVC) */)
		lights_disable();
	#else
		lights_enable();
	#endif
}

void app_process()
{
	uint8_t target_current = 0;
	uint8_t target_cadence = assist_level_data.level.max_cadence_percent;
	uint8_t throttle_percent = throttle_map_response(throttle_read());

	bool throttle_override = false;

	if (check_power_block())
	{
		target_current = 0;
	}
	else if (brake_is_activated())
	{
		target_current = 0;
		set_brake_lights(true);
	}
	else if (assist_level == ASSIST_PUSH && USE_PUSH_WALK)
	{
		target_current = calculate_current_for_power(WALK_MODE_TARGET_POWER_WATTS);
	}
	else
	{
		set_brake_lights(false);

		#if USE_SPEED_SENSOR && USE_PRETENSION
			apply_pretension(&target_current);
		#endif
		apply_pas_cadence(&target_current, throttle_percent);
		apply_cruise(&target_current, throttle_percent);

		throttle_override = apply_throttle(&target_current, throttle_percent);

		// override target cadence if configured in assist level
		if (throttle_override &&
			assist_level_data.level.target_power_watts > 0 &&
			assist_level_data.level.max_pas_speed_kph > 0 &&
			(assist_level_data.level.flags & ASSIST_FLAG_OVERRIDE_CADENCE))
		{
			target_cadence = THROTTLE_CADENCE_OVERRIDE_PERCENT;
		}
	}

	bool thermal_limiting = apply_thermal_limit(&target_current);
	bool lvc_limiting = apply_low_voltage_limit(&target_current);
	bool shift_limiting =
	#if USE_SHIFT_SENSOR
		apply_shift_sensor_interrupt(&target_current);
	#else
		false;
	#endif
	bool is_limiting = thermal_limiting || lvc_limiting || shift_limiting;

	apply_current_ramp_up(&target_current, is_limiting || !throttle_override);
	apply_current_ramp_down(&target_current, !shift_limiting);
	#if USE_SPEED_SENSOR
		apply_speed_limit(&target_current, throttle_override);
	#endif

	motor_set_target_speed(target_cadence);
	motor_set_target_current(target_current);

	if (target_current > 0)
	{
		motor_enable();
	}
	else
	{
		motor_disable();
	}
}


void app_set_assist_level(uint8_t level)
{
	if (assist_level != level)
	{
		if (assist_level == ASSIST_PUSH && USE_PUSH_WALK)
		{
			// When releasing push walk mode pedals may have been rotating
			// with the motor, block motor power for 2 seconds to prevent PAS
			// sensor from incorrectly applying power if returning to a PAS level.
			block_power_for(1000);
		}

		assist_level = level;
		eventlog_write_data(EVT_DATA_ASSIST_LEVEL, assist_level);
		reload_assist_params();
	}
}

void app_set_lights(bool on)
{
	if ( // it's ok to write ugly code if you say it's ugly...
		(ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_LIGHTS) ||
		(assist_level == ASSIST_0 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS0_LIGHT) ||
		(assist_level == ASSIST_1 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS1_LIGHT) ||
		(assist_level == ASSIST_2 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS2_LIGHT) ||
		(assist_level == ASSIST_3 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS3_LIGHT) ||
		(assist_level == ASSIST_4 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS4_LIGHT) ||
		(assist_level == ASSIST_5 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS5_LIGHT) ||
		(assist_level == ASSIST_6 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS6_LIGHT) ||
		(assist_level == ASSIST_7 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS7_LIGHT) ||
		(assist_level == ASSIST_8 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS8_LIGHT) ||
		(assist_level == ASSIST_9 && ASSIST_MODE_SELECT == ASSIST_MODE_SELECT_PAS9_LIGHT)
		)
	{
		if (on)
		{
			app_set_operation_mode(OPERATION_MODE_SPORT);
		}
		else
		{
			app_set_operation_mode(OPERATION_MODE_DEFAULT);
		}
	}
	else
	{
		if (LIGHTS_MODE == LIGHTS_MODE_DEFAULT && lights_state != on ||
			LIGHTS_MODE == LIGHTS_MODE_DEFAULT_AND_BRAKE_LIGHT && lights_state != on)
		{
			lights_state = on;
			eventlog_write_data(EVT_DATA_LIGHTS, on);
			lights_set(on);
		}
	}
}

void set_brake_lights(bool on)
{
	#if LIGHTS_MODE == LIGHTS_MODE_BRAKE_LIGHT
		lights_set(on);
	#elif LIGHTS_MODE == LIGHTS_MODE_DEFAULT_AND_BRAKE_LIGHT
		if (!app_get_lights()) // If lights are on, don't use brake light
			lights_set(on);
	#endif
}

#ifdef SPEED_LIMIT_SPORT_SWITCH_KPH
void app_set_speed_limit_operation_mode(uint16_t display_speed_limit_rpm)
{
	if (display_speed_limit_rpm == convert_wheel_speed_kph_to_rpm(SPEED_LIMIT_SPORT_SWITCH_KPH, true))
	{
		app_set_operation_mode(OPERATION_MODE_SPORT);
	}
	else
	{
		app_set_operation_mode(OPERATION_MODE_DEFAULT);
	}
}
#endif

void app_set_operation_mode(uint8_t mode)
{
	if (operation_mode != mode)
	{
		operation_mode = mode;
		eventlog_write_data(EVT_DATA_OPERATION_MODE, operation_mode);
		reload_assist_params();
	}
}

uint8_t app_get_assist_level()
{
	return assist_level;
}

uint8_t app_get_lights()
{
	return lights_state;
}

uint8_t app_get_status_code()
{
	uint16_t motor = motor_status();

	if (motor & MOTOR_ERROR_HALL_SENSOR)
	{
		return STATUS_ERROR_HALL_SENSOR;
	}

	if (motor & MOTOR_ERROR_CURRENT_SENSE)
	{
		return STATUS_ERROR_CURRENT_SENSE;
	}

	if (motor & MOTOR_ERROR_POWER_RESET)
	{
		// Phase line error code reused, cause and meaning
		// of MOTOR_ERROR_POWER_RESET triggered on bbs02 is currently unknown
		return STATUS_ERROR_PHASE_LINE;
	}

	if (!throttle_ok())
	{
		return STATUS_ERROR_THROTTLE;
	}

	if (temperature_motor_c > MAX_TEMPERATURE)
	{
		return STATUS_ERROR_MOTOR_OVER_TEMP;
	}

	if (temperature_contr_c > MAX_TEMPERATURE)
	{
		return STATUS_ERROR_CONTROLLER_OVER_TEMP;
	}

	// Disable LVC error since it is not shown on display in original firmware
	// Uncomment if you want to enable
	// if (motor & MOTOR_ERROR_LVC)
	// {
	//     return STATUS_ERROR_LVC;
	// }

	if (brake_is_activated())
	{
		return STATUS_BRAKING;
	}

	return STATUS_NORMAL;
}

uint8_t app_get_temperature()
{
	int8_t temp_max = MAX(temperature_contr_c, temperature_motor_c);

	if (temp_max < 0)
	{
		return 0;
	}

	return (uint8_t)temp_max;
}

#if USE_SPEED_SENSOR && USE_PRETENSION
	void apply_pretension(uint8_t* target_current)
	{
		uint16_t current_speed_rpm_x10 = speed_sensor_get_rpm_x10();

		if (current_speed_rpm_x10 > pretension_cutoff_speed_rpm_x10)
		{
			*target_current = 1;
		}
	return;
	}
#endif

void apply_pas_cadence(uint8_t* target_current, uint8_t throttle_percent)
{
	if (assist_level_data.level.target_power_watts > 0 && assist_level_data.level.max_pas_speed_kph > 0)
	{
		if (pas_is_pedaling_forwards() && pas_get_pulse_counter() > PAS_START_DELAY_PULSES)
		{
			uint8_t level_target_current = calculate_current_for_power(assist_level_data.level.target_power_watts);
			if (assist_level_data.level.flags & ASSIST_FLAG_PAS_VARIABLE)
			{
				uint8_t current = (uint8_t)MAP16(throttle_percent, 0, 100, 0, level_target_current);
				if (current > *target_current)
				{
					*target_current = current;
				}
			}
			else
			{
				if (level_target_current > *target_current)
				{
					*target_current = level_target_current;
				}

				// apply "keep current" ramp
				#if PAS_KEEP_CURRENT_PERCENT < 100
					if (*target_current > assist_level_data.keep_current_target_percent &&
						pas_get_cadence_rpm_x10() > assist_level_data.keep_current_ramp_start_rpm_x10)
					{
						uint32_t cadence = MIN(pas_get_cadence_rpm_x10(), assist_level_data.keep_current_ramp_end_rpm_x10);

						// ramp down current towards keep_current_target_percent with rpm above keep_current_ramp_start_rpm_x10
						*target_current = MAP32(
							cadence,	// in
							assist_level_data.keep_current_ramp_start_rpm_x10,		// in_min
							assist_level_data.keep_current_ramp_end_rpm_x10,		// in_max
							*target_current,										// out_min
							assist_level_data.keep_current_target_percent);			// out_max
					}
				#endif
			}
		}
	}
}

void apply_cruise(uint8_t* target_current, uint8_t throttle_percent)
{
	static bool cruise_block_throttle_return = false;

	if ((assist_level_data.level.flags & ASSIST_FLAG_CRUISE) && throttle_ok())
	{
		// pause cruise if brake activated
		if (brake_is_activated())
		{
			cruise_paused = true;
			cruise_block_throttle_return = true;
		}

		// pause cruise if started pedaling backwards
		else if (pas_is_pedaling_backwards() && pas_get_pulse_counter() > CRUISE_DISENGAGE_PAS_PULSES)
		{
			cruise_paused = true;
			cruise_block_throttle_return = true;
		}

		// pause cruise if throttle touched while cruise active
		else if (!cruise_paused && !cruise_block_throttle_return && throttle_percent > 0)
		{
			cruise_paused = true;
			cruise_block_throttle_return = true;
		}

		// unpause cruise if pedaling forward while engaging throttle > 50%
		else if (cruise_paused && !cruise_block_throttle_return && throttle_percent > 50 && pas_is_pedaling_forwards() && pas_get_pulse_counter() > CRUISE_ENGAGE_PAS_PULSES)
		{
			cruise_paused = false;
			cruise_block_throttle_return = true;
		}

		// reset flag tracking throttle to make sure throttle returns to idle position before engage/disenage cruise with throttle touch
		else if (cruise_block_throttle_return && throttle_percent == 0)
		{
			cruise_block_throttle_return = false;
		}

		if (cruise_paused)
		{
			*target_current = 0;
		}
		else
		{
			uint8_t level_target_current = calculate_current_for_power(assist_level_data.level.target_power_watts);
			if (level_target_current > *target_current)
			{
				*target_current = level_target_current;
			}
		}
	}
}

bool apply_throttle(uint8_t* target_current, uint8_t throttle_percent)
{
	if (assist_level_data.level.max_throttle_power_watts > 0 && assist_level_data.level.max_throttle_speed_kph > 0 && throttle_percent > 0 && throttle_ok())
	{
		uint8_t current = (uint8_t)MAP16(throttle_percent, 0, 100, THROTTLE_START_PERCENT, calculate_current_for_power(assist_level_data.level.max_throttle_power_watts));

		if (current >= *target_current)
		{
			*target_current = current;
			return true;
		}
	}

	return false;
}

// The motor expects current to be provided as a percentage of the max current,
// So calculate the current in amps first before dividing it by the max
uint8_t calculate_current_for_power(uint16_t watts)
{
	static uint8_t power_current_percent = 0;
	uint16_t voltage_x10 = motor_get_battery_voltage_x10();
	if (voltage_x10 > 0) {
		// We don't do anything to combat a feedback loop caused by voltage sag.
		// The highest it will go in this case is 100%
		power_current_percent = MIN(100, ((uint32_t)watts * 1000U) /
			((uint32_t)voltage_x10 * MAX_CURRENT_AMPS));
	}
	return power_current_percent;
}

#if USE_SPEED_SENSOR
	void apply_speed_limit(uint8_t* target_current, bool throttle_override)
	{
		static uint32_t last_pid_ms = 50;
		static uint16_t last_speed_rpm_x10;
		static uint8_t clamped_output;
		static float i_term;
		static bool speed_limiting = false;

		uint16_t max_speed_rpm_x10;
		// If the user engages the throttle while the throttle limit is lower than the current PAS limit, the behaviour seen is that the motor stops.
		// Instead, only override if the throttle limit is higher than the PAS limit, or if the user is not pedaling
		if (throttle_override && (assist_level_data.max_throttle_wheel_speed_rpm_x10 > assist_level_data.max_pas_wheel_speed_rpm_x10 || !pas_is_pedaling_forwards()))
		{
			max_speed_rpm_x10 = assist_level_data.max_throttle_wheel_speed_rpm_x10;
		}
		else
		{
			max_speed_rpm_x10 = assist_level_data.max_pas_wheel_speed_rpm_x10;
		}

		if (max_speed_rpm_x10 > 0 && *target_current > 0)
		{
			// PID controller. Evaluates every 50ms
			uint32_t now_ms = system_ms();
			uint16_t time_diff = (uint16_t)(now_ms - last_pid_ms);
			if (time_diff >= 50)
			{
				uint16_t current_speed_rpm_x10 = speed_sensor_get_rpm_x10();

				// If the PID has been off for >=2s, reset
				if (time_diff >= 2000) {
					last_speed_rpm_x10 = current_speed_rpm_x10;
					i_term = (float)*target_current;
				}

				int16_t error = (int16_t)max_speed_rpm_x10 - (int16_t)current_speed_rpm_x10;
				// Accumulate the difference. This is what tracks the value it's "hunting" for
				// and if it's above the max speed, this will go into negative
				i_term += SPEED_LIMIT_PID_KI_X005 * (float)error;
				// Don't allow the error to go above the target current
				i_term = CLAMP(i_term, 0, *target_current);

				int16_t d_input = (int16_t)current_speed_rpm_x10 - (int16_t)last_speed_rpm_x10;

				int16_t output = (int16_t)(SPEED_LIMIT_PID_KP * (float)error + i_term - SPEED_LIMIT_PID_KD_X5 * (float)d_input);
				// We want to keep the motor spinning at 1% even if at the speed limit to avoid jerky behaviour
				if (output < 1) {
	                clamped_output = 1;
	            } else if (output > *target_current) {
	                clamped_output = *target_current;
	            } else {
	                clamped_output = (uint8_t)output;
	            }

				// Commit current loops' vars for the next loop to use
				last_speed_rpm_x10 = current_speed_rpm_x10;
				last_pid_ms = now_ms;
			}

			if (*target_current > clamped_output)
			{
				if (!speed_limiting)
				{
					speed_limiting = true;
					eventlog_write_data(EVT_DATA_SPEED_LIMITING, 1);
				}
				*target_current = clamped_output;
			}
			else if (speed_limiting)
			{
				speed_limiting = false;
				eventlog_write_data(EVT_DATA_SPEED_LIMITING, 0);
			}
		}
	}
#endif

bool apply_thermal_limit(uint8_t* target_current)
{
	static uint32_t next_log_temp_ms = 10000;

	static bool temperature_limiting = false;

	int16_t temp_contr_x100 = temperature_contr_x100();
	temperature_contr_c = temp_contr_x100 / 100;

	int16_t temp_motor_x100 = temperature_motor_x100();
	temperature_motor_c = temp_motor_x100 / 100;

	int16_t max_temp_x100 = MAX(temp_contr_x100, temp_motor_x100);
	int8_t max_temp = MAX(temperature_contr_c, temperature_motor_c);

	if (eventlog_is_enabled() && USE_TEMPERATURE_SENSOR && system_ms() >= next_log_temp_ms)
	{
		next_log_temp_ms = system_ms() + 10000;
		eventlog_write_data(EVT_DATA_TEMPERATURE, (uint16_t)temperature_motor_c << 8 | temperature_contr_c);
	}

	if (max_temp >= (MAX_TEMPERATURE - MAX_TEMPERATURE_RAMP_DOWN_INTERVAL))
	{
		if (!temperature_limiting)
		{
			temperature_limiting = true;
			eventlog_write_data(EVT_DATA_THERMAL_LIMITING, 1);
		}

		if (max_temp_x100 > MAX_TEMPERATURE * 100)
		{
			max_temp_x100 = MAX_TEMPERATURE * 100;
		}

		uint8_t tmp = (uint8_t)MAP32(
			max_temp_x100,													// value
			(MAX_TEMPERATURE - MAX_TEMPERATURE_RAMP_DOWN_INTERVAL) * 100,	// in_min
			MAX_TEMPERATURE * 100,											// in_max
			100,															// out_min
			MAX_TEMPERATURE_LOW_CURRENT_PERCENT								// out_max
		);

		if (*target_current > tmp)
		{
			*target_current = tmp;
			return true;
		}
	}
	else
	{
		if (temperature_limiting)
		{
			temperature_limiting = false;
			eventlog_write_data(EVT_DATA_THERMAL_LIMITING, 0);
		}
	}

	return false;
}

bool apply_low_voltage_limit(uint8_t* target_current)
{
	static uint32_t next_log_volt_ms = 10000;
	static bool lvc_limiting = false;

	static uint32_t next_voltage_reading_ms = 125;
	static int32_t flt_min_bat_volt_x100 = 100 * 100;

	if (system_ms() > next_voltage_reading_ms)
	{
		next_voltage_reading_ms = system_ms() + 125;
		int32_t voltage_reading_x100 = motor_get_battery_voltage_x10() * 10ul;

		if (voltage_reading_x100 < flt_min_bat_volt_x100)
		{
			flt_min_bat_volt_x100 = EXPONENTIAL_FILTER(flt_min_bat_volt_x100, voltage_reading_x100, 8);
		}

		if (eventlog_is_enabled() && system_ms() > next_log_volt_ms)
		{
			next_log_volt_ms = system_ms() + 10000;
			eventlog_write_data(EVT_DATA_VOLTAGE, (uint16_t)voltage_reading_x100);
		}
	}

	uint16_t voltage_x100 = flt_min_bat_volt_x100;

	if (voltage_x100 <= lvc_ramp_down_start_voltage_x100)
	{
		if (!lvc_limiting)
		{
			eventlog_write_data(EVT_DATA_LVC_LIMITING, voltage_x100);
			lvc_limiting = true;
		}

		if (voltage_x100 < lvc_voltage_x100)
		{
			voltage_x100 = lvc_voltage_x100;
		}

		// Ramp down power until LVC_LOW_CURRENT_PERCENT when approaching LVC
		int8_t tmp = (int8_t)MAP32(
			voltage_x100,						// value
			lvc_ramp_down_end_voltage_x100,		// in_min
			lvc_ramp_down_start_voltage_x100,	// in_max
			LVC_LOW_CURRENT_PERCENT,			// out_min
			100									// out_max
		);
		uint8_t tmp2 = (uint8_t)CLAMP(tmp, 0, 100);

		if (*target_current > tmp2)
		{
			*target_current = tmp2;
			return true;
		}
	}

	return false;
}

#if USE_SHIFT_SENSOR
	bool apply_shift_sensor_interrupt(uint8_t* target_current)
	{
		static uint32_t shift_sensor_act_ms = 0;
		static bool shift_sensor_last = false;
		static bool shift_sensor_interrupting = false;
		static bool shift_sensor_logged = false;

		bool active = shift_sensor_is_activated();
		if (active)
		{
			// Check for new pulse from the gear sensor during shift interrupt
			if (!shift_sensor_last && shift_sensor_interrupting)
			{
				// Consecutive gear change, do restart.
				shift_sensor_interrupting = false;
			}
			if (!shift_sensor_interrupting)
			{
				uint16_t duration_ms = SHIFT_INTERRUPT_DURATION_MS;
				shift_sensor_act_ms = system_ms() + duration_ms;
				shift_sensor_interrupting = true;
			}
			shift_sensor_last = true;
		}
		else
		{
			shift_sensor_last = false;
		}

		if (!shift_sensor_interrupting)
		{
			return false;
		}

		if (system_ms() >= shift_sensor_act_ms)
		{
			// Shift is finished, reset function state.
			shift_sensor_interrupting = false;
			// Logging is skipped, unless current has been clamped during shift interrupt.
			if (shift_sensor_logged)
			{
				shift_sensor_logged = false;
				eventlog_write_data(EVT_DATA_SHIFT_SENSOR, 0);
			}
			return false;
		}

		if ((*target_current) > SHIFT_INTERRUPT_CURRENT_THRESHOLD_PERCENT)
		{
			if (!shift_sensor_logged)
			{
				// Logging only once per shifting interrupt.
				shift_sensor_logged = true;
				eventlog_write_data(EVT_DATA_SHIFT_SENSOR, 1);
			}
			// Set target current based on desired current threshold during shift.
			*target_current = SHIFT_INTERRUPT_CURRENT_THRESHOLD_PERCENT;

			return true;
		}

		return false;
	}
#endif

void apply_current_ramp_up(uint8_t* target_current, bool enable)
{
	static uint8_t ramp_up_target_current = 0;
	static uint32_t last_ramp_up_increment_ms = 0;

	if (enable && *target_current > ramp_up_target_current)
	{
		uint32_t now = system_ms();
		uint16_t time_diff = now - last_ramp_up_increment_ms;

		if (time_diff >= ramp_up_current_interval_ms)
		{
			++ramp_up_target_current;

			if (last_ramp_up_increment_ms == 0)
			{
				last_ramp_up_increment_ms = now;
			}
			else
			{
				// offset for time overshoot to not accumulate large ramp error
				last_ramp_up_increment_ms = now - (uint8_t)(time_diff - ramp_up_current_interval_ms);
			}
		}

		*target_current = ramp_up_target_current;
	}
	else
	{
		ramp_up_target_current = *target_current;
		last_ramp_up_increment_ms = 0;
	}
}

void apply_current_ramp_down(uint8_t* target_current, bool enable)
{
	static uint8_t ramp_down_target_current = 0;
	static uint32_t last_ramp_down_decrement_ms = 0;

	// apply fast ramp down if coming from high target current (> 50%)
	if (enable && *target_current < ramp_down_target_current)
	{
		uint32_t now = system_ms();
		uint16_t time_diff = now - last_ramp_down_decrement_ms;

		if (time_diff >= 10)
		{
			uint8_t diff = ramp_down_target_current - *target_current;

			if (diff >= CURRENT_RAMP_DOWN_PERCENT_10MS)
			{
				ramp_down_target_current -= CURRENT_RAMP_DOWN_PERCENT_10MS;
			}
			else
			{
				ramp_down_target_current -= diff;
			}

			if (last_ramp_down_decrement_ms == 0)
			{
				last_ramp_down_decrement_ms = now;
			}
			else
			{
				// offset for time overshoot to not accumulate large ramp error
				last_ramp_down_decrement_ms = now - (uint8_t)(time_diff - 10);
			}
		}

		*target_current = ramp_down_target_current;
	}
	else
	{
		ramp_down_target_current = *target_current;
		last_ramp_down_decrement_ms = 0;
	}
}


bool check_power_block()
{
	if (power_blocked_until_ms != 0)
	{
		// power block is active, check if time to release
		if (system_ms() > power_blocked_until_ms)
		{
			power_blocked_until_ms = 0;
			return false;
		}

		return true;
	}

	return false;
}

void block_power_for(uint16_t ms)
{
	power_blocked_until_ms = system_ms() + ms;
}

void reload_assist_params()
{
	if (assist_level < ASSIST_PUSH)
	{
		assist_level_data.level = assist_levels[operation_mode][assist_level];

		assist_level_data.max_pas_wheel_speed_rpm_x10 = ((int32_t)convert_wheel_speed_kph_to_rpm(assist_level_data.level.max_pas_speed_kph, false)) * 10;
		assist_level_data.max_throttle_wheel_speed_rpm_x10 = ((int32_t)convert_wheel_speed_kph_to_rpm(assist_level_data.level.max_throttle_speed_kph, false)) * 10;
		eventlog_write_data(EVT_DATA_WHEEL_SPEED_PPM, assist_level_data.max_pas_wheel_speed_rpm_x10);

		if (assist_level_data.level.target_power_watts > 0 && assist_level_data.level.max_pas_speed_kph > 0)
		{
			assist_level_data.keep_current_target_percent = (uint8_t)((uint16_t)PAS_KEEP_CURRENT_PERCENT * calculate_current_for_power(assist_level_data.level.target_power_watts) / 100);
			assist_level_data.keep_current_ramp_start_rpm_x10 = PAS_KEEP_CURRENT_CADENCE_RPM * 10;
			assist_level_data.keep_current_ramp_end_rpm_x10 = (uint16_t)(((uint32_t)assist_level_data.level.max_cadence_percent * MAX_CADENCE_RPM_X10) / 100);
		}

		// pause cruise if swiching level
		cruise_paused = true;
	}
	// only apply push walk params if push walk is active in config,
	// otherwise data of previous assist level is kept.
	else if (assist_level == ASSIST_PUSH && USE_PUSH_WALK)
	{
		assist_level_data.level.flags = 0;
		assist_level_data.level.max_cadence_percent = 15;
		assist_level_data.level.max_pas_speed_kph = 0;
		assist_level_data.level.target_power_watts = 0;
		assist_level_data.level.max_throttle_speed_kph = 0;
		assist_level_data.level.max_throttle_power_watts = 0;

		assist_level_data.max_pas_wheel_speed_rpm_x10 = convert_wheel_speed_kph_to_rpm(WALK_MODE_SPEED_KPH, false) * 10;
	}
}
