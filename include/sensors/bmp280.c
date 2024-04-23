#include "bmp280.h"

static uint8_t i2c_buff[8];

static void bmp280_write8(bmp280_t* handle, uint8_t reg, uint8_t data)
{
    i2c_set_addr(handle->i2c_handle, BMP280_ADDR);
    i2c_write_reg_u8(handle->i2c_handle, reg, data);
}

static uint8_t bmp280_read8(bmp280_t* handle, uint8_t reg)
{
    i2c_set_addr(handle->i2c_handle, BMP280_ADDR);
    i2c_read_reg(handle->i2c_handle, reg, i2c_buff, 1);
	return i2c_buff[0];
}

static uint16_t bmp280_read16(bmp280_t* handle, uint8_t reg)
{
    i2c_set_addr(handle->i2c_handle, BMP280_ADDR);
    i2c_read_reg(handle->i2c_handle, reg, i2c_buff, 2);
	return (((uint16_t) i2c_buff[1]) << 8) | ((uint16_t) i2c_buff[0]);
}

static uint32_t bmp280_read24(bmp280_t* handle, uint8_t reg)
{
    i2c_set_addr(handle->i2c_handle, BMP280_ADDR);
    i2c_read_reg(handle->i2c_handle, reg, i2c_buff, 3);
	return (((uint32_t) i2c_buff[0]) << 16) | (((uint32_t) i2c_buff[1]) << 8) | ((uint32_t) i2c_buff[2]);
}

void bmp280_init(bmp280_t* bmp280)
{
	//uint8_t chipid = bmp280_read8(BMP280_REGISTER_CHIPID);
	//printf("BMP280 chip id = 0x%02X\n", chipid);

	bmp280_write8(bmp280, BMP280_REGISTER_CONFIG, (STANDBY_MS_1 << 5) | (FILTER_X16 << 2));
	bmp280_write8(bmp280, BMP280_REGISTER_CONTROL, (SAMPLING_X16 << 5) | (SAMPLING_X16 << 2) | MODE_NORMAL);

	bmp280->dig_T1 = bmp280_read16(bmp280, BMP280_REGISTER_DIG_T1);
	bmp280->dig_T2 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_T2);
	bmp280->dig_T3 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_T3);
	bmp280->dig_P1 = bmp280_read16(bmp280, BMP280_REGISTER_DIG_P1);
	bmp280->dig_P2 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P2);
	bmp280->dig_P3 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P3);
	bmp280->dig_P4 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P4);
	bmp280->dig_P5 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P5);
	bmp280->dig_P6 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P6);
	bmp280->dig_P7 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P7);
	bmp280->dig_P8 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P8);
	bmp280->dig_P9 = (int16_t) bmp280_read16(bmp280, BMP280_REGISTER_DIG_P9);
}

void bmp280_read_temperature(bmp280_t* bmp280)
{
	int32_t adc_T = (int32_t) bmp280_read24(bmp280, BMP280_REGISTER_TEMPDATA);
	adc_T >>= 4;

	int32_t var1 = ((((adc_T >> 3) - ((int32_t) bmp280->dig_T1 << 1))) * ((int32_t) bmp280->dig_T2)) >> 11;
	int32_t var2 = (((((adc_T >> 4) - ((int32_t) bmp280->dig_T1)) * ((adc_T >> 4) - ((int32_t) bmp280->dig_T1))) >> 12) * ((int32_t) bmp280->dig_T3)) >> 14;

	bmp280->t_fine = var1 + var2;

	float T = (bmp280->t_fine * 5 + 128) >> 8;
	bmp280->temp = T / 100;
}

void bmp280_read_pressure(bmp280_t* bmp280)
{
  int64_t var1, var2, p;

  int32_t adc_P = (int32_t) bmp280_read24(bmp280, BMP280_REGISTER_PRESSUREDATA);
  adc_P >>= 4;

  var1 = ((int64_t) bmp280->t_fine) - 128000;
  var2 = var1 * var1 * (int64_t) bmp280->dig_P6;
  var2 = var2 + ((var1 * (int64_t) bmp280->dig_P5) << 17);
  var2 = var2 + (((int64_t) bmp280->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t) bmp280->dig_P3) >> 8) + ((var1 * (int64_t) bmp280->dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t) bmp280->dig_P1) >> 33;

  if (var1 == 0) {
    return; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t) bmp280->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t) bmp280->dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t) bmp280->dig_P7) << 4);
  bmp280->pres = (float)p / 256;
}

void bmp280_read_all(bmp280_t* bmp280)
{
    bmp280_read_temperature(bmp280);
    bmp280_read_pressure(bmp280);
}
