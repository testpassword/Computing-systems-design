#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <UART.h>

#include "main.h"


extern UART_HandleTypeDef huart6;


static inline bool uart_is_ready() { return HAL_UART_GetState(&huart6) == HAL_UART_STATE_READY; }

void print(const char * content) {
	while (!uart_is_ready());
	HAL_UART_Transmit_IT(&huart6, (void *) content, strlen(content));
}

void println(const char * message) {
	print(message);
	print("\r\n");
}

void print_format(const char * format, ...) {
	static char buffer[1024];
	while (!uart_is_ready());
	va_list ap;
	va_start(ap, format);
	vsnprintf(buffer, sizeof(buffer), format, ap);
	va_end(ap);
	print(buffer);
}
