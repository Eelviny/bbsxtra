/*
 * bbs-fw
 *
 * Copyright (C) Daniel Nilsson, 2022.
 *
 * Released under the GPL License, Version 3
 */

#include "interrupt.h" // IMPORTANT: interrupt vector declarations must be included from main.c!
#include "timers.h"
#include "system.h"
#include "fwconfig.h"
#include "eventlog.h"
#include "app.h"
#include "battery.h"
#include "watchdog.h"
#include "adc.h"
#include "motor.h"
#include "extcom.h"
#include "sensors.h"
#include "throttle.h"
#include "lights.h"

#define APP_PROCESS_INTERVAL_MS		5

void main(void)
{
	motor_pre_init();

	watchdog_init();
	timers_init();
	system_init();

	eventlog_init(false);
	extcom_init();

	if (watchdog_triggered())
	{
		// force write watchdog reset to eventlog
		bool prev = eventlog_is_enabled();
		eventlog_set_enabled(true);
		eventlog_write(EVT_ERROR_WATCHDOG_TRIGGERED);
		eventlog_set_enabled(prev);
	}

	adc_init();
	sensors_init();

	pas_set_stop_delay((uint16_t)PAS_STOP_DELAY_X100S * 10);

	battery_init();
	throttle_init(
		THROTTLE_START_VOLTAGE_MV,
		THROTTLE_END_VOLTAGE_MV
	);

	motor_init(MAX_CURRENT_AMPS * 1000, LOW_CUT_OFF_V,
		ADC_VOLTAGE_CALIBRATION_STEPS_X100);

	lights_init();

	app_init();

	uint32_t next_app_proccess = system_ms();
	while (1)
	{
		uint32_t now = system_ms();

		adc_process();
		motor_process();

		if (now >= next_app_proccess)
		{
			next_app_proccess = now + APP_PROCESS_INTERVAL_MS;

			battery_process();
			sensors_process();
			extcom_process();
			app_process();
		}

		watchdog_yield();
	}
}
