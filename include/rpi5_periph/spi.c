#include <sys/ioctl.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "spi.h"


int spi_init(spi_handle_t* spi_handle)
{
	if ((spi_handle->handle = open(spi_handle->dev_path, O_RDWR)) < 0) {
		fprintf(stderr, "Failed to open the SPI device '%s'!\r\n", spi_handle->dev_path);
		return -1;
	}
	return 0;
}

int spi_deinit(spi_handle_t* spi_handle)
{
	if (spi_handle->handle) {
		close(spi_handle->handle);
		spi_handle->handle = 0;
	}
	return 0;
}

int spi_set_mode(spi_handle_t* spi_handle, unsigned int mode)
{
	if (!spi_handle->handle) {
		return -1;
	}

	if (ioctl(spi_handle->handle, SPI_IOC_WR_MODE32, &mode) < 0) {
		fprintf(stderr, "Failed to write SPI mode.\r\n");
		return -2;
	}

	return 0;
}

int spi_set_bits_per_word(spi_handle_t* spi_handle, unsigned char bits_per_word)
{
	if (!spi_handle->handle) {
		return -1;
	}

	if (ioctl(spi_handle->handle, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
		fprintf(stderr, "Failed to write SPI bits per word.\r\n");
		return -2;
	}

	return 0;
}


int spi_set_speed(spi_handle_t* spi_handle, unsigned int speed)
{
	if (!spi_handle->handle) {
		return -1;
	}

	if (ioctl(spi_handle->handle, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
		fprintf(stderr, "Failed to write SPI speed.\r\n");
		return -2;
	}

	return 0;
}


int spi_write(spi_handle_t* spi_handle, unsigned char* data, unsigned int len)
{
	if (!spi_handle->handle) {
		return -1;
	}

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = (unsigned long) data;
    tr.len = len;
    tr.cs_change = 1;
    tr.delay_usecs = 0;

	if (ioctl(spi_handle->handle, SPI_IOC_MESSAGE(1), &tr) < 0) {
		fprintf(stderr, "Failed to write SPI data.\r\n");
		return -2;
	}
	return 0;
}


int spi_read(spi_handle_t* spi_handle, unsigned char* data, unsigned int len)
{
	if (!spi_handle->handle) {
		return -1;
	}

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.rx_buf = (unsigned long) data;
    tr.len = len;
    tr.cs_change = 1;
    tr.delay_usecs = 0;

	if (ioctl(spi_handle->handle, SPI_IOC_MESSAGE(1), &tr) < 0) {
		fprintf(stderr, "Failed to read SPI data.\r\n");
		return -2;
	}
	return 0;
}
