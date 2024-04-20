#ifndef MAX9939_H_
#define MAX9939_H_

#include <stdint.h>
#include <stdbool.h>
#include "../rpi5_periph/spi.h"


#define MAX9939_SHDN (0x01 << 7)
#define MAX9939_MEAS (0x01 << 6)

#define MAX9939_VOS_NEG (0x01 << 5)
#define MAX9939_VOS_0_0MV (0x00 << 1)
#define MAX9939_VOS_1_3MV (0x01 << 1)
#define MAX9939_VOS_2_5MV (0x02 << 1)
#define MAX9939_VOS_3_8MV (0x03 << 1)
#define MAX9939_VOS_4_9MV (0x04 << 1)
#define MAX9939_VOS_6_1MV (0x05 << 1)
#define MAX9939_VOS_7_3MV (0x06 << 1)
#define MAX9939_VOS_8_4MV (0x07 << 1)
#define MAX9939_VOS_10_6MV (0x08 << 1)
#define MAX9939_VOS_11_7MV (0x09 << 1)
#define MAX9939_VOS_12_7MV (0x0A << 1)
#define MAX9939_VOS_13_7MV (0x0B << 1)
#define MAX9939_VOS_14_7MV (0x0C << 1)
#define MAX9939_VOS_15_7MV (0x0D << 1)
#define MAX9939_VOS_16_7MV (0x0E << 1)
#define MAX9939_VOS_17_6MV (0x0F << 1)

#define MAX9939_GAIN_0_25X ((0x09 << 1) | 0x01)
#define MAX9939_GAIN_1X ((0x00 << 1) | 0x01)
#define MAX9939_GAIN_10X ((0x01 << 1) | 0x01)
#define MAX9939_GAIN_20X ((0x02 << 1) | 0x01)
#define MAX9939_GAIN_30X ((0x03 << 1) | 0x01)
#define MAX9939_GAIN_40X ((0x04 << 1) | 0x01)
#define MAX9939_GAIN_60X ((0x05 << 1) | 0x01)
#define MAX9939_GAIN_80X ((0x06 << 1) | 0x01)
#define MAX9939_GAIN_120X ((0x07 << 1) | 0x01)
#define MAX9939_GAIN_157X ((0x08 << 1) | 0x01)


const float max9939_gains_values[] = { 
    0.25f,
    1.0f,
    10.0f,
    20.0f,
    30.0f,
    40.0f,
    60.0f,
    80.0f,
    120.0f,
    157.0f
};
const uint8_t max9939_gains_bytes[] = {
        MAX9939_GAIN_0_25X, 
        MAX9939_GAIN_1X,
        MAX9939_GAIN_10X,
        MAX9939_GAIN_20X,
        MAX9939_GAIN_30X,
        MAX9939_GAIN_40X,
        MAX9939_GAIN_60X,
        MAX9939_GAIN_80X,
        MAX9939_GAIN_120X,
        MAX9939_GAIN_157X
};

const float max9939_offsets_values[] = {  
    -17.6f,
    -16.7f,
    -15.7f,
    -14.7f,
    -13.7f,
    -12.7f,
    -11.7f,
    -10.6f,
    -8.4f,
    -7.3f,
    -6.1f,
    -4.9f,
    -3.8f,
    -2.5f,
    -1.3f,
    0.0f,
    1.3f,
    2.5f,
    3.8f,
    4.9f,
    6.1f,
    7.3f,
    8.4f,
    10.6f,
    11.7f,
    12.7f,
    13.7f,
    14.7f,
    15.7f,
    16.7f,
    17.6f
};

const uint8_t max9939_offsets_bytes[] = {
    MAX9939_VOS_NEG | MAX9939_VOS_17_6MV,
    MAX9939_VOS_NEG | MAX9939_VOS_16_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_15_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_14_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_13_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_12_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_11_7MV,
    MAX9939_VOS_NEG | MAX9939_VOS_10_6MV,
    MAX9939_VOS_NEG | MAX9939_VOS_8_4MV,
    MAX9939_VOS_NEG | MAX9939_VOS_7_3MV,
    MAX9939_VOS_NEG | MAX9939_VOS_6_1MV,
    MAX9939_VOS_NEG | MAX9939_VOS_4_9MV,
    MAX9939_VOS_NEG | MAX9939_VOS_3_8MV,
    MAX9939_VOS_NEG | MAX9939_VOS_2_5MV,
    MAX9939_VOS_NEG | MAX9939_VOS_1_3MV,
    MAX9939_VOS_0_0MV,
    MAX9939_VOS_1_3MV,
    MAX9939_VOS_2_5MV,
    MAX9939_VOS_3_8MV,
    MAX9939_VOS_4_9MV,
    MAX9939_VOS_6_1MV,
    MAX9939_VOS_7_3MV,
    MAX9939_VOS_8_4MV,
    MAX9939_VOS_10_6MV,
    MAX9939_VOS_11_7MV,
    MAX9939_VOS_12_7MV,
    MAX9939_VOS_13_7MV,
    MAX9939_VOS_14_7MV,
    MAX9939_VOS_15_7MV,
    MAX9939_VOS_16_7MV,
    MAX9939_VOS_17_6MV
};

float max9939_set_gain(spi_handle_t* spi, float gain, bool shdn, bool meas);
float max9939_set_offset(spi_handle_t* spi, float offset, bool shdn, bool meas);

#endif
