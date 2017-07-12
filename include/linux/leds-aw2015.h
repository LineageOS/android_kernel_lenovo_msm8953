
#ifndef __LINUX_AW2015_LED_H__
#define __LINUX_AW2015_LED_H__

/* The definition of each time described as shown in figure.
 *        /-----------\
 *       /      |      \
 *      /|      |      |\
 *     / |      |      | \-----------
 *       |hold_time_ms |      |
 *       |             |      |
 * rise_time_ms  fall_time_ms |
 *                       off_time_ms
 */

struct aw2015_platform_data {
	int max_current;
	int rise_time_ms;
	int hold_time_ms;
	int fall_time_ms;
	int off_time_ms;
	int pwm_duty;
	int mode;
	struct aw2015_led *led;
};

#endif
