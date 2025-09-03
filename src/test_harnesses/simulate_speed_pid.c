#include "stdint.h"
#include "stdbool.h"
#include "stdio.h"

#define SPEED_LIMIT_PID_KP 0.10f
#define SPEED_LIMIT_PID_KI_X005 0.004f // Multiply by 0.05
#define SPEED_LIMIT_PID_KD_X5 0.01f
#define WHEEL_CIRCUMFERENCE_MM 2268

#define MAX(x, y) (x) > (y) ? (x) : (y)
#define MIN(x, y) (x) < (y) ? (x) : (y)
#define CLAMP(x, min, max) (MIN(MAX(x, min), max))
#define MAX_CURRENT_AMPS 25
#define CURRENT_RAMP_AMPS_S 8
#define CURRENT_RAMP_DOWN_PERCENT_10MS 5

uint32_t system_ms;
uint8_t target_current = 0;
uint16_t ramp_up_current_interval_ms = (MAX_CURRENT_AMPS * 10u) / CURRENT_RAMP_AMPS_S;

uint16_t convert_wheel_speed_kph_to_rpm(uint8_t speed_kph)
{
	uint16_t true_rpm = ((uint32_t)speed_kph * 100000UL) / ((uint32_t)WHEEL_CIRCUMFERENCE_MM * 6UL);
	return true_rpm;
}

void apply_current_ramp_up(uint8_t* target_current, bool enable)
{
	static uint8_t ramp_up_target_current = 0;
	static uint32_t last_ramp_up_increment_ms = 0;

	if (enable && *target_current > ramp_up_target_current)
	{
		uint32_t now = system_ms;
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
		uint32_t now = system_ms;
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

void apply_speed_limit(uint8_t* target_current, uint16_t current_speed)
{
	static uint32_t last_pid_ms = 50;
	static uint16_t last_speed_rpm_x10;
	static uint8_t clamped_output;
	static float i_term;
	static bool speed_limiting = false;

	uint16_t max_speed_rpm_x10 = convert_wheel_speed_kph_to_rpm(25) * 10;

	if (max_speed_rpm_x10 > 0 && *target_current > 0)
	{
		// PID controller. Evaluates every 50ms
		uint32_t now_ms = system_ms;
		uint16_t time_diff = (uint16_t)(now_ms - last_pid_ms);
		if (time_diff >= 60)
		{
			uint16_t current_speed_rpm_x10 = convert_wheel_speed_kph_to_rpm(current_speed) * 10;

			// If the PID has been off for >=2s, reset
			if (time_diff >= 2000) {
				last_speed_rpm_x10 = current_speed_rpm_x10;
				i_term = (float)(*target_current);
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
			}
			*target_current = clamped_output;
		}
		else if (speed_limiting)
		{
			speed_limiting = false;
		}
	}
}

int main(void)
{
    float speed_kph = 0.0f;

    // constants for simple dynamics
    const float dt = 0.005f;        // 50 ms step
    const float accel_per_amp = 0.02f; // arbitrary gain from current % → accel
    const float drag = 0.006f;     // proportional drag vs speed

    for (uint32_t t = 0; t <= 100000; t += 5)
    {
    	system_ms = t;
    	target_current = 100;  // full throttle request
        // Apply speed limiter
        apply_speed_limit(&target_current, (uint16_t)speed_kph);
        apply_current_ramp_up(&target_current, true);
        apply_current_ramp_down(&target_current, true);

        // Simple bike dynamics: accel = thrust - drag
        float accel = target_current * accel_per_amp - drag * speed_kph;
        speed_kph += accel * dt;
        if (speed_kph < 0) speed_kph = 0;

        if (t % 200 == 0)
        {
        	printf("t=%5u ms | current=%3u %% | speed=%.2f kph\n", t, target_current, speed_kph);
        }
    }
}
