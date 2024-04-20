#include "max9939.h"

#include <float.h>
#include <math.h>

static uint8_t reverse_bits(uint8_t byte)
{
    uint8_t res = 0x00;
    for (int i = 0; i < 8; i++) {
        res |= ((byte >> i) & 0x01) << (7 - i);
    }
    return res;
}

float max9939_set_gain(spi_handle_t* spi, float gain, bool shdn, bool meas)
{
    uint8_t byte = shdn ? MAX9939_SHDN : 0x00;
    byte |= meas ? MAX9939_MEAS : 0x00;

    int sel_i;
    float min_diff = FLT_MAX;
    float diff;
    for (int i = 0; i < sizeof(max9939_gains_values) / sizeof(float); i++) {
        diff = fabsf(max9939_gains_values[i] - gain);
        if (diff < min_diff) {
            min_diff = diff;
            sel_i = i;
        }
    }

    byte |= max9939_gains_bytes[sel_i];
    byte = reverse_bits(byte);
    spi_write(spi, &byte, 1);
    spi_write(spi, &byte, 1);

    return max9939_gains_values[sel_i];
}

float max9939_set_offset(spi_handle_t* spi, float offset, bool shdn, bool meas)
{
    uint8_t byte = shdn ? MAX9939_SHDN : 0x00;
    byte |= meas ? MAX9939_MEAS : 0x00;

    int sel_i;
    float min_diff = FLT_MAX;
    float diff;
    for (int i = 0; i < sizeof(max9939_offsets_values) / sizeof(float); i++) {
        diff = fabsf(max9939_offsets_values[i] - offset);
        if (diff < min_diff) {
            min_diff = diff;
            sel_i = i;
        }
    }

    byte |= max9939_offsets_bytes[sel_i];
    byte = reverse_bits(byte);
    spi_write(spi, &byte, 1);
    spi_write(spi, &byte, 1);

    return max9939_offsets_values[sel_i];
}
