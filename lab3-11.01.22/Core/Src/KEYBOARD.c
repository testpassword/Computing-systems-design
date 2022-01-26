#include <KEYBOARD.h>
#include <stdbool.h>
#include <stddef.h>
#include <UART.h>


#define BUFFER_CAPACITY (32)
#define INC_BUFFER_IDX(__idx) do { (__idx) = ((__idx) + 1) % (BUFFER_CAPACITY); } while (0)
#define KB_I2C_ADDRESS (0xE2)
#define KB_I2C_READ_ADDRESS ((KB_I2C_ADDRESS) | 1)
#define KB_I2C_WRITE_ADDRESS ((KB_I2C_ADDRESS) & ~1)
#define KB_INPUT_REG (0x0)
#define KB_OUTPUT_REG (0x1)
#define KB_CONFIG_REG (0x3)
#define KB_KEY_DEBOUNCE_TIME (50)


static struct kb_event buffer[BUFFER_CAPACITY] = { 0 };
static size_t buffer_start_idx = 0;
static size_t buffer_end_idx = 0;


static void kb_event_push(struct kb_event event) {
    buffer[buffer_end_idx] = event;
    INC_BUFFER_IDX(buffer_end_idx);
}

bool kb_event_has() {
	const uint32_t priMask = __get_PRIMASK();
	__disable_irq();
	const bool ret = buffer_start_idx != buffer_end_idx;
	__set_PRIMASK(priMask);
	return ret;
}

struct kb_event kb_event_pop() {
	const uint32_t priMask = __get_PRIMASK();
	__disable_irq();
	const struct kb_event evt = buffer[buffer_start_idx];
	INC_BUFFER_IDX(buffer_start_idx);
	__set_PRIMASK(priMask);
	return evt;
}

void kb_init(I2C_HandleTypeDef * i2c) {
    static uint8_t output = 0x0;
    HAL_I2C_Mem_Write(i2c, KB_I2C_WRITE_ADDRESS, KB_OUTPUT_REG, 1, &output, 1, 100);
}

static void kb_write_config(I2C_HandleTypeDef * i2c, uint8_t data) {
    static uint8_t buf;
    buf = data;
    HAL_I2C_Mem_Write_IT(i2c, KB_I2C_WRITE_ADDRESS, KB_CONFIG_REG, 1, &buf, 1);
}

static void kb_select_row(I2C_HandleTypeDef * i2c, uint8_t row) {
	kb_write_config(i2c, ~((uint8_t) (1 << row)));
}

static void kb_read_input(I2C_HandleTypeDef * i2c, uint8_t * data) {
	HAL_I2C_Mem_Read_IT(i2c, KB_I2C_READ_ADDRESS, KB_INPUT_REG, 1, data, 1); }

void kb_scan_step(I2C_HandleTypeDef * i2c) {
    static uint8_t reg_buffer = ~0;
    static uint8_t row = 0;
    static bool read = false;
    static bool input_keys[12] = { 0 };
    static bool output_keys[12] = { 0 };
    static uint32_t key_time[12] = { 0 };
    if (HAL_I2C_GetState(i2c) != HAL_I2C_STATE_READY) return;
    if (!read) {
        for (uint8_t i = 0, mask = 0x10; i < 3; ++i, mask <<= 1)
            if ((reg_buffer & mask) == 0)
            	input_keys[row * 3 + i] = true;
        row = (row + 1) % 4;
        if (row == 0) {
        	uint8_t count = 0;
        	for (int i = 0; i < 12; ++i)
        		if (input_keys[i])
        			++count;
        	if (count > 2)
        		for (int i = 0; i < 12; ++i)
        			input_keys[i] = false;
            for (int i = 0; i < 12; ++i) {
                const uint32_t t = HAL_GetTick();
                if (output_keys[i] == input_keys[i]) key_time[i] = 0;
                else if (key_time[i] == 0) key_time[i] = t;
                else if (t - key_time[i] >= KB_KEY_DEBOUNCE_TIME) {
                    output_keys[i] = !output_keys[i];
                    if (output_keys[i]) kb_event_push((struct kb_event) { .type = KB_EVENT_TYPE_PRESS, .key = i });
                    else kb_event_push((struct kb_event) { .type = KB_EVENT_TYPE_RELEASE, .key = i });
                }
            }
            memset(input_keys, 0, sizeof(input_keys));
        }
        kb_select_row(i2c, row);
    } else kb_read_input(i2c, &reg_buffer);
    read = !read;
}
