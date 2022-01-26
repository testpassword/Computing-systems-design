#include <stdbool.h>
#include "gpio.h"

bool is_btn_pressed() {
	 if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15) == 0)
		 while (1) {
			 HAL_Delay(10);
			 if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15) == 1)
				 return true;
		 }
	 return false;
 }


void reset_LEDs() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
}
