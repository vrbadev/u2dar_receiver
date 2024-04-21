#include "rp1.h"

#include <unistd.h>
#include <fcntl.h> 
#include <sys/mman.h> 


int rp1_init(rp1_t* rp1)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    rp1->base_ptr = (void*) mmap(NULL, RP1_SPAN, PROT_READ | PROT_WRITE, MAP_SHARED, fd, RP1_BASE);

    if (rp1->base_ptr != MAP_FAILED) {
        rp1->devmem_fd = fd;
        rp1->gpio_io_bank0 = (rp1_gpio_io_bank_t*) RP1_GET_ADDR(rp1, RP1_GPIO_IO_BANK0_BASE);
        rp1->gpio_pads_bank0 = (rp1_gpio_pads_bank_t*) RP1_GET_ADDR(rp1, RP1_GPIO_PADS_BANK0_BASE);
        rp1->sys_rio0 = (rp1_sys_rio_t*) RP1_GET_ADDR(rp1, RP1_SYS_RIO0_BASE);
        rp1->sys_rio0_xor = (rp1_sys_rio_t*) RP1_GET_ADDR(rp1, RP1_SYS_RIO0_BASE + RP1_SYS_RIO0_XOR_OFFSET);
        rp1->sys_rio0_set = (rp1_sys_rio_t*) RP1_GET_ADDR(rp1, RP1_SYS_RIO0_BASE + RP1_SYS_RIO0_SET_OFFSET);
        rp1->sys_rio0_clr = (rp1_sys_rio_t*) RP1_GET_ADDR(rp1, RP1_SYS_RIO0_BASE + RP1_SYS_RIO0_CLR_OFFSET);
        rp1->pwm0 = (rp1_pwm_t*) RP1_GET_ADDR(rp1, RP1_PWM0_BASE);
        rp1->pwm1 = (rp1_pwm_t*) RP1_GET_ADDR(rp1, RP1_PWM1_BASE);
        rp1->spi0 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI0_BASE);
        rp1->spi1 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI1_BASE);
        rp1->spi2 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI2_BASE);
        rp1->spi3 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI3_BASE);
        rp1->spi4 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI4_BASE);
        rp1->spi5 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI5_BASE);
        rp1->spi6 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI6_BASE);
        rp1->spi7 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI7_BASE);
        rp1->spi8 = (rp1_spi_t*) RP1_GET_ADDR(rp1, RP1_SPI8_BASE);
    } else {
        rp1->base_ptr = NULL;
        rp1->devmem_fd = -1;
        return 1;
    }
    return 0;
}

void rp1_deinit(rp1_t* rp1)
{
    if (rp1->base_ptr != NULL) {
        munmap(rp1->base_ptr, RP1_SPAN);
        close(rp1->devmem_fd);
        rp1->base_ptr = NULL;
        rp1->devmem_fd= -1;
    }
}

void rp1_gpio_funcsel(rp1_t* rp1, uint32_t pin, uint32_t altfun) 
{
    rp1->gpio_io_bank0->gpio[pin].ctrl = altfun & 0x1F;
}

void rp1_gpio_config_pullup(rp1_t* rp1, uint32_t pin) 
{
    rp1->gpio_pads_bank0->gpio[pin] &= ~BIT(2);
    rp1->gpio_pads_bank0->gpio[pin] |= BIT(3);
}

void rp1_gpio_config_pulldown(rp1_t* rp1, uint32_t pin) 
{
    rp1->gpio_pads_bank0->gpio[pin] &= ~BIT(3);
    rp1->gpio_pads_bank0->gpio[pin] |= BIT(2);
}

void rp1_gpio_config_nopull(rp1_t* rp1, uint32_t pin) 
{
    rp1->gpio_pads_bank0->gpio[pin] &= ~BIT(2);
    rp1->gpio_pads_bank0->gpio[pin] &= ~BIT(3);
}


void rp1_gpio_config_input(rp1_t* rp1, uint32_t pin)
{
    rp1->gpio_pads_bank0->gpio[pin] |= BIT(6);
}

void rp1_sys_rio_config_output(rp1_t* rp1, uint32_t pin)
{
    rp1->gpio_pads_bank0->gpio[pin] &= ~BIT(6);
    rp1->sys_rio0->oe = BIT(pin);
    rp1->sys_rio0_set->oe = BIT(pin);
    rp1->sys_rio0_clr->oe = BIT(pin);
    rp1->sys_rio0_xor->oe = BIT(pin);
}

void rp1_sys_rio_out_set(rp1_t* rp1, uint32_t pin)
{
    rp1->sys_rio0_set->out = BIT(pin);
}

void rp1_sys_rio_out_xor(rp1_t* rp1, uint32_t pin)
{
    rp1->sys_rio0_xor->out = BIT(pin);
}

void rp1_sys_rio_out_clr(rp1_t* rp1, uint32_t pin)
{
    rp1->sys_rio0_clr->out = BIT(pin);
}

uint32_t rp1_sys_rio_in_get(rp1_t* rp1, uint32_t pin)
{
    return rp1->sys_rio0->in & BIT(pin);
}

void rp1_pwm_chan_enable(rp1_pwm_t* pwm, uint32_t ch, uint32_t en) 
{
    if (en) {
        pwm->global_ctrl |= BIT(ch);
    } else {
        pwm->global_ctrl &= ~BIT(ch);
    }
    pwm->global_ctrl |= BIT(31); // update bit
}

void rp1_pwm_chan_config(rp1_pwm_t* pwm, uint32_t ch, uint32_t mode, uint32_t range, uint32_t phase, uint32_t duty)
{
    pwm->chan[ch].ctrl = mode;
    pwm->chan[ch].range = range;
    pwm->chan[ch].phase = phase;
    pwm->chan[ch].duty = duty;
}
