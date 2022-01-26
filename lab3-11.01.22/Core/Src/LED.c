#include <LED.h>
#include "main.h"


extern TIM_HandleTypeDef htim4;

const char * const led_names[] = {
    [GREEN] = "GREEN",
    [YELLOW] = "YELLOW",
    [RED] = "RED",
};


typedef void (* abstract_LED_setter)(uint16_t);

static void set_green_LED(uint16_t power) { htim4.Instance->CCR2 = power; }

static void set_yellow_LED(uint16_t power) { htim4.Instance->CCR3 = power; }

static void set_red_LED(uint16_t power) { htim4.Instance->CCR4 = power; }

static const abstract_LED_setter led_setters[] = {
    [GREEN] = set_green_LED,
    [YELLOW] = set_yellow_LED,
    [RED] = set_red_LED,
};

void led_set_brightness(enum LED led, uint8_t power) {
    if (power > 100) power = 100;
    led_setters[led]((uint16_t) power * 10);
}
