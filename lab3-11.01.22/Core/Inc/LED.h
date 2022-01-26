#pragma once

#include <stdint.h>


enum LED {
    GREEN = 0,
    YELLOW = 1,
    RED = 2,
};

struct GarlandMode {
    enum LED color;
    uint8_t brightness;
};

extern const char * const led_names[];

void led_set_brightness(enum LED led, uint8_t power);

static void led_mode_enable(struct GarlandMode mode) { led_set_brightness(mode.color, mode.brightness); }

static void led_mode_disable(struct GarlandMode mode) { led_set_brightness(mode.color, 0); }
