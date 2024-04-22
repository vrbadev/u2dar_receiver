#ifndef MAX262_H_
#define MAX262_H_

#include <stdint.h>
#include "../rpi5_periph/rp1.h"

typedef struct {
    rp1_t* rp1_handle;
    uint32_t gpios_a[4];
    uint32_t gpios_d[2];
    uint32_t gpio_nwr;
    uint32_t gpios_clk[2];
} max262_t;

int max262_init(max262_t* handle);
void max262_deinit(max262_t* handle);

#endif
