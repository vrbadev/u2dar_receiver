#ifndef AHT20_H_
#define AHT20_H_

#include <stdint.h>
#include "../rpi5_periph/i2c.h"

#define AHT20_ADDR 0x38
#define AHT20_CMD_INIT 0xBE
#define AHT20_CMD_MEAS 0xAC

typedef struct {
    i2c_handle_t* i2c_handle;
	float temp;
	float hum;
} aht20_t;

void aht20_init(aht20_t* handle);
void aht20_read(aht20_t* handle);
void aht20_trigger(aht20_t* handle);

#endif
