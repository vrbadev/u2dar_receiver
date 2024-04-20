#ifndef __SPI_H__
#define __SPI_H__

#include <stdint.h>
#include <linux/spi/spidev.h>

typedef struct {
    const char* dev_path; // eg. "/dev/spidev0.0"
    int handle;
} spi_handle_t;


int spi_init(spi_handle_t* spi_handle);
int spi_deinit(spi_handle_t* spi_handle);
int spi_set_mode(spi_handle_t* spi_handle, unsigned int mode);
int spi_set_bits_per_word(spi_handle_t* spi_handle, unsigned char bits_per_word);
int spi_set_speed(spi_handle_t* spi_handle, unsigned int speed);
int spi_write(spi_handle_t* spi_handle, unsigned char* data, unsigned int len);
int spi_read(spi_handle_t* spi_handle, unsigned char* data, unsigned int len);

#endif 