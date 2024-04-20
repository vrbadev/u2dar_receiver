#ifndef RP1_H_
#define RP1_H_

// ref: https://datasheets.raspberrypi.com/rp1/rp1-peripherals.pdf

#include <stdlib.h>
#include <stdint.h>


// RP1 device memory
#define RP1_BASE 0x1F00000000
#define RP1_SPAN 0x0000400000

#define RP1_GET_ADDR(rp1, offset) (((uint64_t) (rp1).base_ptr) + (offset))
#define BIT(pos) (1 << (pos))
#define GENMASK(last_bit, first_bit) (((1 << ((last_bit) - (first_bit) + 1)) - 1) << (first_bit))

// GPIO interface
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/pinctrl/pinctrl-rp1.c
#define RP1_GPIO_IO_BANK0_BASE 0xD0000
#define RP1_GPIO_PADS_BANK0_BASE 0xF0000

typedef struct {
    uint32_t status;
    uint32_t ctrl;
} rp1_gpio_io_bank_gpio_t;

typedef struct {
    uint32_t inte;
    uint32_t intf;
    uint32_t ints;
} rp1_gpio_io_bank_int_t;

typedef struct {
    rp1_gpio_io_bank_gpio_t gpio[27];
    uint32_t intr;
    rp1_gpio_io_bank_int_t proc[2]; 
    rp1_gpio_io_bank_int_t pcie; 
} rp1_gpio_io_bank_t;

typedef struct {
    uint32_t voltage_select;
    uint32_t gpio[27];
} rp1_gpio_pads_bank_t;


// UART interface
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/tty/serial/amba-pl011.c

#define RP1_UART0_BASE 0x30000
#define RP1_UART1_BASE 0x34000
#define RP1_UART2_BASE 0x38000
#define RP1_UART3_BASE 0x3C000
#define RP1_UART4_BASE 0x40000
#define RP1_UART5_BASE 0x44000


// RIO (Registered IO) interface
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/pinctrl/pinctrl-rp1.c
#define RP1_SYS_RIO0_BASE 0xE0000

#define RP1_SYS_RIO0_XOR_OFFSET 0x1000
#define RP1_SYS_RIO0_SET_OFFSET 0x2000
#define RP1_SYS_RIO0_CLR_OFFSET 0x3000

typedef struct {
    uint32_t out;
    uint32_t oe;
    uint32_t in;
    uint32_t in_sync;
} rp1_sys_rio_t;


// PWM interface
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/pwm/pwm-rp1.c
#define RP1_PWM0_BASE 0x98000
#define RP1_PWM1_BASE 0x9C000

typedef struct {
    uint32_t ctrl;
    uint32_t range;
    uint32_t phase;
    uint32_t duty;
} rp1_pwm_chan_t;

typedef struct {
    uint32_t global_ctrl;
    uint32_t fifo_ctrl;
    uint32_t common_range;
    uint32_t common_duty;
    uint32_t duty_fifo;
    rp1_pwm_chan_t chan[4];
    uint32_t intr;
    uint32_t inte;
    uint32_t intf;
    uint32_t ints;
} rp1_pwm_t;


// I2C interface registers
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/i2c/busses/i2c-designware-platdrv.c
#define RP1_I2C0_BASE 0x70000
#define RP1_I2C1_BASE 0x74000
#define RP1_I2C2_BASE 0x78000
#define RP1_I2C3_BASE 0x7C000
#define RP1_I2C4_BASE 0x80000
#define RP1_I2C5_BASE 0x84000
#define RP1_I2C6_BASE 0x88000


// SPI interface registers
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/drivers/spi/spi-dw-mmio.c
#define RP1_SPI8_BASE 0x4C000
#define RP1_SPI0_BASE 0x50000
#define RP1_SPI1_BASE 0x54000
#define RP1_SPI2_BASE 0x58000
#define RP1_SPI3_BASE 0x5C000
#define RP1_SPI4_BASE 0x60000
#define RP1_SPI5_BASE 0x64000
#define RP1_SPI6_BASE 0x68000 
#define RP1_SPI7_BASE 0x6C000

typedef struct {
    uint32_t ctrlr[2];
    uint32_t ssienr;
    uint32_t mwcr;
    uint32_t ser;
    uint32_t baudr;
    uint32_t txftlr;
    uint32_t rxftlr;
    uint32_t sr;
    uint32_t imr;
    uint32_t isr;
    uint32_t risr;
    uint32_t txoicr;
    uint32_t rxoicr;
    uint32_t rxuicr;
    uint32_t msticr;
    uint32_t icr;
    uint32_t dmacr;
    uint32_t dmatdlr;
    uint32_t dmardlr;
    uint32_t idr;
    uint32_t version;
    uint32_t dr;
    uint32_t _reserved[3];
    uint32_t sample_dly;
    uint32_t cs_override;
} rp1_spi_t;


// I2S interface registers
// ref: https://github.com/raspberrypi/linux/blob/rpi-6.1.y/sound/soc/dwc/dwc-i2s.c
#define RP1_I2S0_BASE 0xA0000
#define RP1_I2S1_BASE 0xA4000
#define RP1_I2S2_BASE 0xA8000


// TICKS interface registers
#define RP1_TICKS_BASE 0x174000

typedef struct {
    uint32_t ctrl;
    uint32_t cycles;
    uint32_t count;
} rp1_ticks_regs_t;

typedef struct {
    rp1_ticks_regs_t timer;
    rp1_ticks_regs_t watchdog;
    rp1_ticks_regs_t proc;
    rp1_ticks_regs_t dma[2];
    rp1_ticks_regs_t io_bank[3];
} rp1_ticks_t;


// CLK peripheral
#define RP1_CLK_BASE 0x18000


typedef struct {
    void* base_ptr;
    int devmem_fd;
    rp1_gpio_io_bank_t* gpio_io_bank0;
    rp1_gpio_pads_bank_t* gpio_pads_bank0;
    rp1_sys_rio_t* sys_rio0;
    rp1_sys_rio_t* sys_rio0_xor;
    rp1_sys_rio_t* sys_rio0_set;
    rp1_sys_rio_t* sys_rio0_clr;
    rp1_pwm_t* pwm0;
    rp1_pwm_t* pwm1;
    rp1_spi_t* spi0;
    rp1_spi_t* spi1;
    rp1_spi_t* spi2;
    rp1_spi_t* spi3;
    rp1_spi_t* spi4;
    rp1_spi_t* spi5;
    rp1_spi_t* spi6;
    rp1_spi_t* spi7;
    rp1_spi_t* spi8;
} rp1_t;

rp1_t rp1_init(void);
void rp1_deinit(rp1_t* rp1);

// some auxiliary functions
void rp1_gpio_funcsel(rp1_t* rp1, uint32_t pin, uint32_t altfun);

void rp1_sys_rio_config_output(rp1_t* rp1, uint32_t pin);
void rp1_sys_rio_out_set(rp1_t* rp1, uint32_t pin);
void rp1_sys_rio_out_xor(rp1_t* rp1, uint32_t pin);
void rp1_sys_rio_out_clr(rp1_t* rp1, uint32_t pin);
uint32_t rp1_sys_rio_in_get(rp1_t* rp1, uint32_t pin);

void rp1_pwm_chan_enable(rp1_pwm_t* pwm, uint32_t ch, uint32_t en);
void rp1_pwm_chan_config(rp1_pwm_t* pwm, uint32_t ch, uint32_t mode, uint32_t range, uint32_t phase, uint32_t duty);

#endif
