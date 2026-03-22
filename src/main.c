#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "BTN_OUT";

// --- Pin Definitions ---
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

const gpio_num_t btn_pins[4] = {
    BUTTON_FRONTWARDS, BUTTON_BACKWARDS, BUTTON_LEFT, BUTTON_RIGHT
};
const gpio_num_t frontwards_pins[4] = {
    FRONT_FORWARD_LEFT,
    FRONT_FORWARD_RIGHT,
    BACK_FRONTWARDS_LEFT,
    BACK_FRONTWARDS_RIGHT,
};
const gpio_num_t backwards_pins[4] = {
    FRONT_BACKWARDS_LEFT,
    FRONT_BACKWARDS_RIGHT,
    BACK_BACKWARDS_LEFT,
    BACK_BACKWARDS_RIGHT
};

static void gpio_init_pins(void)
{
    {
        uint64_t input_mask = 0;
        for (int i = 0; i < 4; i++)
            input_mask |= (1ULL << btn_pins[i]);

        gpio_config_t in_conf = {
            .pin_bit_mask  = input_mask,
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_ENABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&in_conf);
    }
    {
        uint64_t output_mask = 0;
        for (int i = 0; i < 4; i++)
            output_mask |= (1ULL << frontwards_pins[i]);
        for (int i = 0; i < 4; i++)
            output_mask |= (1ULL << backwards_pins[i]);

        gpio_config_t out_conf = {
            .pin_bit_mask  = output_mask,
            .mode          = GPIO_MODE_OUTPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&out_conf);
    }
}

static void motors_all_stop(void)
{
    for (int i = 0; i < 4; i++) {
        gpio_set_level(frontwards_pins[i], 0);
        gpio_set_level(backwards_pins[i],  0);
    }
}

static void motors_forward(void)
{
    motors_all_stop();
    for (int i = 0; i < 4; i++)
        gpio_set_level(frontwards_pins[i], 1);
}

static void motors_backward(void)
{
    motors_all_stop();
    for (int i = 0; i < 4; i++)
        gpio_set_level(backwards_pins[i], 1);
}

static void motors_turn_left(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_FORWARD_RIGHT,   1);
    gpio_set_level(BACK_FRONTWARDS_RIGHT, 1);
    gpio_set_level(FRONT_BACKWARDS_LEFT,  1);
    gpio_set_level(BACK_BACKWARDS_LEFT,   1);
}

static void motors_turn_right(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_FORWARD_LEFT,    1);
    gpio_set_level(BACK_FRONTWARDS_LEFT,  1);
    gpio_set_level(FRONT_BACKWARDS_RIGHT, 1);
    gpio_set_level(BACK_BACKWARDS_RIGHT,  1);
}

static void motors_forward_left(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_FORWARD_RIGHT,   1);
    gpio_set_level(BACK_FRONTWARDS_RIGHT, 1);
}

static void motors_forward_right(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_FORWARD_LEFT,   1);
    gpio_set_level(BACK_FRONTWARDS_LEFT, 1);
}

static void motors_backward_left(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_BACKWARDS_RIGHT, 1);
    gpio_set_level(BACK_BACKWARDS_RIGHT,  1);
}

static void motors_backward_right(void)
{
    motors_all_stop();
    gpio_set_level(FRONT_BACKWARDS_LEFT, 1);
    gpio_set_level(BACK_BACKWARDS_LEFT,  1);
}

void app_main(void)
{
    gpio_init_pins();
    motors_all_stop();
    ESP_LOGI(TAG, "Car controller ready");

    while (1) {
        bool fwd   = (gpio_get_level(BUTTON_FRONTWARDS) == 0);
        bool bwd   = (gpio_get_level(BUTTON_BACKWARDS)  == 0);
        bool left  = (gpio_get_level(BUTTON_LEFT)        == 0);
        bool right = (gpio_get_level(BUTTON_RIGHT)       == 0);

        if (fwd && !bwd) {
            if      (left  && !right) motors_forward_left();
            else if (right && !left)  motors_forward_right();
            else                      motors_forward();
        } else if (bwd && !fwd) {
            if      (left  && !right) motors_backward_left();
            else if (right && !left)  motors_backward_right();
            else                      motors_backward();
        } else if (left && !right) {
            motors_turn_left();
        } else if (right && !left) {
            motors_turn_right();
        } else {
            motors_all_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz polling + debounce
    }
}
