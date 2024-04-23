#include "aht20.h"

static uint8_t i2c_buff[8];

void aht20_init(aht20_t* handle)
{
    i2c_set_addr(handle->i2c_handle, AHT20_ADDR);

    i2c_buff[0] = 0x08;
    i2c_buff[1] = 0x00;

    i2c_write_reg(handle->i2c_handle, AHT20_CMD_INIT, i2c_buff, 2);
}

void aht20_read(aht20_t* handle)
{
    i2c_set_addr(handle->i2c_handle, AHT20_ADDR);

    i2c_read_reg(handle->i2c_handle, AHT20_CMD_MEAS, i2c_buff, 6);
    uint32_t val = i2c_buff[1];
    val <<= 8;
    val |= i2c_buff[2];
    val <<= 4;
    val |= i2c_buff[3] >> 4;
    handle->hum = (((float) val) * 100) / 0x100000;
    val = i2c_buff[3] & 0x0F;
    val <<= 8;
    val |= i2c_buff[4];
    val <<= 8;
    val |= i2c_buff[5];
    handle->temp = ((((float) val) * 200) / 0x100000) - 50;
}

void aht20_trigger(aht20_t* handle)
{
    i2c_set_addr(handle->i2c_handle, AHT20_ADDR);

    i2c_buff[0] = 0x33;
    i2c_buff[1] = 0x00;

    i2c_write_reg(handle->i2c_handle, AHT20_CMD_MEAS, i2c_buff, 2);
}
