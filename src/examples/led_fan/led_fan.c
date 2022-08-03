/**
 * @file led_fan.c
 * led and fan test application.
 *
 * @author Donghee Park <dongheepark@gmail.com>
 */

#include <px4_posix.h>
#include <px4_defines.h>
#include <px4_log.h>
#include <px4_module.h>

#include "drivers/drv_pwm_output.h"

__EXPORT int led_fan_main(int argc, char *argv[]);

int led_fan_main(int argc, char *argv[])
{
	int ret;
	const char *dev = "/dev/pwm_output1";
  unsigned long last_value;
  unsigned long group_mask;
  int pwm_value;
	PX4_INFO("LED (FMU_CH6) and FAN (FMU_CH8) Test");

	int fd = px4_open(dev, 0);

	if (fd < 0) {
		PX4_ERR("can't open %s", dev);
		return 1;
	}

  // FAN: FMU_CH8
  // set update rate
  ret = px4_ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, 25000);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET_UPDATE_RATE %d", ret);
  }

  // select group
  ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(2), (unsigned long)&group_mask);
 	if (ret != OK) {
	  PX4_INFO("SERVO_GET_RATEGROUP %d, %lu", ret, group_mask);
  }

	ret = px4_ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, group_mask);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET_SELECT_UPDATE_RATE %d", ret);
  }

  // get current pwm value
	ret = px4_ioctl(fd, PWM_SERVO_GET(8-1), (unsigned long)&last_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_GET %d", ret);
	  PX4_INFO("LAST VALUE: %lu", last_value);
  }

  // set pwm value to group 2 channel 8
  pwm_value = 20;
	ret = px4_ioctl(fd, PWM_SERVO_SET(8-1), pwm_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET %d", ret);
  }

	PX4_INFO("Set 50%% duty ratio in FMU_CH8 25kHz");
	PX4_INFO("Wait for 10 secs");
	px4_sleep(10);

  // reset pwm value
	ret = px4_ioctl(fd, PWM_SERVO_SET(8-1), last_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET %d", ret);
  }

  // LED: FMU_CH6
  group_mask = 0;

  // set update rate
  ret = px4_ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, 50);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET_UPDATE_RATE %d", ret);
  }

  // select group
  ret = px4_ioctl(fd, PWM_SERVO_GET_RATEGROUP(1), (unsigned long)&group_mask);
 	if (ret != OK) {
	  PX4_INFO("SERVO_GET_RATEGROUP %d, %lu", ret, group_mask);
  }

	ret = px4_ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, group_mask);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET_SELECT_UPDATE_RATE %d", ret);
  }

  // get current pwm value
	ret = px4_ioctl(fd, PWM_SERVO_GET(6-1), (unsigned long)&last_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_GET %d", ret);
	  PX4_INFO("LAST VALUE: %lu", last_value);
  }

  // set pwm value to group 1 channel 6
  pwm_value = 2000;
	ret = px4_ioctl(fd, PWM_SERVO_SET(6-1), pwm_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET %d", ret);
  }

	PX4_INFO("Set 2ms duty in FMU_CH6 50Hz");
	PX4_INFO("Wait for 10 secs");
	px4_sleep(10);

  // reset pwm value
	ret = px4_ioctl(fd, PWM_SERVO_SET(6-1), last_value);
 	if (ret != OK) {
	  PX4_INFO("SERVO_SET %d", ret);
  }

 	PX4_INFO("exiting");

	return 0;
}
