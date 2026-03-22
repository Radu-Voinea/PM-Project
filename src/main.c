#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
static const char *TAG = "BTN_OUT";

#define BUTTON_FRONTWARDS     GPIO_NUM_9
#define BUTTON_BACKWARDS      GPIO_NUM_10
#define BUTTON_LEFT           GPIO_NUM_11
#define BUTTON_RIGHT          GPIO_NUM_12

#define FRONT_FORWARD_LEFT    GPIO_NUM_1
#define FRONT_BACKWARDS_LEFT  GPIO_NUM_2
#define FRONT_FORWARD_RIGHT   GPIO_NUM_3
#define FRONT_BACKWARDS_RIGHT GPIO_NUM_4

#define BACK_BACKWARDS_RIGHT  GPIO_NUM_5
#define BACK_FRONTWARDS_RIGHT GPIO_NUM_6
#define BACK_FRONTWARDS_LEFT  GPIO_NUM_7
#define BACK_BACKWARDS_LEFT   GPIO_NUM_8

#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_RESOLUTION   LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ      1000

#define MOTOR_POWER_PCT   30
#define DUTY_ACTIVE       ((1 << 10) * MOTOR_POWER_PCT / 100)

const gpio_num_t btn_pins[4] = {
	BUTTON_FRONTWARDS, BUTTON_BACKWARDS, BUTTON_LEFT, BUTTON_RIGHT
};

typedef struct
{
	gpio_num_t pin;
	ledc_channel_t ch;
} motor_ch_t;

static const motor_ch_t motor_map[8] = {
	{FRONT_FORWARD_LEFT, LEDC_CHANNEL_0},
	{FRONT_BACKWARDS_LEFT, LEDC_CHANNEL_1},
	{FRONT_FORWARD_RIGHT, LEDC_CHANNEL_2},
	{FRONT_BACKWARDS_RIGHT, LEDC_CHANNEL_3},
	{BACK_FRONTWARDS_LEFT, LEDC_CHANNEL_4},
	{BACK_BACKWARDS_LEFT, LEDC_CHANNEL_5},
	{BACK_FRONTWARDS_RIGHT, LEDC_CHANNEL_6},
	{BACK_BACKWARDS_RIGHT, LEDC_CHANNEL_7},
};

const gpio_num_t frontwards_pins[4] = {
	FRONT_FORWARD_LEFT, FRONT_FORWARD_RIGHT,
	BACK_FRONTWARDS_LEFT, BACK_FRONTWARDS_RIGHT,
};
const gpio_num_t backwards_pins[4] = {
	FRONT_BACKWARDS_LEFT, FRONT_BACKWARDS_RIGHT,
	BACK_BACKWARDS_LEFT, BACK_BACKWARDS_RIGHT,
};

static void pin_pwm(gpio_num_t pin, bool active)
{
	for (int i = 0; i < 8; i++) {
		if (motor_map[i].pin == pin) {
			ledc_set_duty(LEDC_MODE, motor_map[i].ch, active ? DUTY_ACTIVE : 0);
			ledc_update_duty(LEDC_MODE, motor_map[i].ch);
			return;
		}
	}
}

static void gpio_init_buttons(void)
{
	uint64_t mask = 0;
	for (int i = 0; i < 4; i++)
		mask |= (1ULL << btn_pins[i]);

	gpio_config_t in_conf = {
		.pin_bit_mask = mask,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&in_conf);
}

static void ledc_init_motors(void)
{
	ledc_timer_config_t timer = {
		.speed_mode = LEDC_MODE,
		.timer_num = LEDC_TIMER,
		.duty_resolution = LEDC_RESOLUTION,
		.freq_hz = LEDC_FREQ_HZ,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ledc_timer_config(&timer);

	for (int i = 0; i < 8; i++) {
		ledc_channel_config_t ch_conf = {
			.gpio_num = motor_map[i].pin,
			.speed_mode = LEDC_MODE,
			.channel = motor_map[i].ch,
			.timer_sel = LEDC_TIMER,
			.duty = 0,
			.hpoint = 0,
		};
		ledc_channel_config(&ch_conf);
	}
}

static void motors_all_stop(void)
{
	for (int i = 0; i < 8; i++) {
		ledc_set_duty(LEDC_MODE, motor_map[i].ch, 0);
		ledc_update_duty(LEDC_MODE, motor_map[i].ch);
	}
}

static void motors_forward(void)
{
	motors_all_stop();
	for (int i = 0; i < 4; i++) pin_pwm(frontwards_pins[i], true);
}

static void motors_backward(void)
{
	motors_all_stop();
	for (int i = 0; i < 4; i++) pin_pwm(backwards_pins[i], true);
}

static void motors_turn_left(void)
{
	motors_all_stop();
	pin_pwm(FRONT_FORWARD_RIGHT, true);
	pin_pwm(BACK_FRONTWARDS_RIGHT, true);
	pin_pwm(FRONT_BACKWARDS_LEFT, true);
	pin_pwm(BACK_BACKWARDS_LEFT, true);
}

static void motors_turn_right(void)
{
	motors_all_stop();
	pin_pwm(FRONT_FORWARD_LEFT, true);
	pin_pwm(BACK_FRONTWARDS_LEFT, true);
	pin_pwm(FRONT_BACKWARDS_RIGHT, true);
	pin_pwm(BACK_BACKWARDS_RIGHT, true);
}

static void motors_forward_left(void)
{
	motors_all_stop();
	pin_pwm(FRONT_FORWARD_RIGHT, true);
	pin_pwm(BACK_FRONTWARDS_RIGHT, true);
}

static void motors_forward_right(void)
{
	motors_all_stop();
	pin_pwm(FRONT_FORWARD_LEFT, true);
	pin_pwm(BACK_FRONTWARDS_LEFT, true);
}

static void motors_backward_left(void)
{
	motors_all_stop();
	pin_pwm(FRONT_BACKWARDS_RIGHT, true);
	pin_pwm(BACK_BACKWARDS_RIGHT, true);
}

static void motors_backward_right(void)
{
	motors_all_stop();
	pin_pwm(FRONT_BACKWARDS_LEFT, true);
	pin_pwm(BACK_BACKWARDS_LEFT, true);
}

void app_main(void)
{
	gpio_init_buttons();
	ledc_init_motors();
	motors_all_stop();
	ESP_LOGI(TAG, "Car ready — power: %d%%", MOTOR_POWER_PCT);

	while (1) {
		bool fwd = (gpio_get_level(BUTTON_FRONTWARDS) == 0);
		bool bwd = (gpio_get_level(BUTTON_BACKWARDS) == 0);
		bool left = (gpio_get_level(BUTTON_LEFT) == 0);
		bool right = (gpio_get_level(BUTTON_RIGHT) == 0);

		if (fwd && !bwd) {
			if (left && !right) motors_forward_left();
			else if (right && !left) motors_forward_right();
			else motors_forward();
		} else if (bwd && !fwd) {
			if (left && !right) motors_backward_left();
			else if (right && !left) motors_backward_right();
			else motors_backward();
		} else if (left && !right) {
			motors_turn_left();
		} else if (right && !left) {
			motors_turn_right();
		} else {
			motors_all_stop();
		}

		vTaskDelay(pdMS_TO_TICKS(20));
	}
}
