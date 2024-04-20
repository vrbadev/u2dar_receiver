#ifndef RP1_SPI_
#define RP1_SPI_

#include "rp1.h"

// adapted from:
// https://github.com/praktronics/rpi5-rp1-spi


#define DW_PSSI_CTRLR0_DFS_MASK     0b00000000000000000000000000001111 //GENMASK(3, 0)
#define DW_PSSI_CTRLR0_DFS32_MASK   0b00000000000111110000000000000000 //GENMASK(20, 16)

#define DW_PSSI_CTRLR0_FRF_MASK     0b00000000000000000000000000110000 //GENMASK(5, 4)
#define DW_SPI_CTRLR0_FRF_MOTO_SPI 0x0
#define DW_SPI_CTRLR0_FRF_TI_SSP 0x1
#define DW_SPI_CTRLR0_FRF_NS_MICROWIRE 0x2
#define DW_SPI_CTRLR0_FRF_RESV 0x3

#define DW_PSSI_CTRLR0_MODE_MASK    0b00000000000000000000000011000000 //GENMASK(7, 6)
#define DW_PSSI_CTRLR0_SCPHA        0b00000000000000000000000001000000 // BIT(6)
#define DW_PSSI_CTRLR0_SCPOL        0b00000000000000000000000010000000 //BIT(7)

#define DW_PSSI_CTRLR0_TMOD_MASK    0b00000000000000000000001100000000// GENMASK(9, 8)
#define DW_SPI_CTRLR0_TMOD_TR 0x0        /* xmit & recv */
#define DW_SPI_CTRLR0_TMOD_TO 0x1        /* xmit only */
#define DW_SPI_CTRLR0_TMOD_RO 0x2        /* recv only */
#define DW_SPI_CTRLR0_TMOD_EPROMREAD 0x3 /* eeprom read mode */

#define DW_PSSI_CTRLR0_SLV_OE       0b00000000000000000000010000000000 // BIT(10)
#define DW_PSSI_CTRLR0_SRL          0b00000000000000000000100000000000 // BIT(11)
#define DW_PSSI_CTRLR0_CFS          0b00000000000000000001000000000000 // BIT(12)

/* Bit fields in CTRLR1 */
#define DW_SPI_NDF_MASK             0b00000000000000001111111111111111 // GENMASK(15, 0) // Number of data frames

/* Bit fields in SR, 7 bits */
#define DW_SPI_SR_MASK              0b00000000000000000000000001111111 // GENMASK(6, 0)
#define DW_SPI_SR_BUSY              0b00000000000000000000000000000001 // BIT(0)
#define DW_SPI_SR_TF_NOT_FULL       0b00000000000000000000000000000010 // BIT(1)
#define DW_SPI_SR_TF_EMPT           0b00000000000000000000000000000100 // BIT(2)
#define DW_SPI_SR_RF_NOT_EMPT       0b00000000000000000000000000001000 // BIT(3)
#define DW_SPI_SR_RF_FULL           0b00000000000000000000000000010000 // BIT(4)
#define DW_SPI_SR_TX_ERR            0b00000000000000000000000000100000 // BIT(5)
#define DW_SPI_SR_DCOL              0b00000000000000000000000001000000 // BIT(6)

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define DW_SPI_INT_MASK             0b00000000000000000000000000111111 // GENMASK(5, 0)
#define DW_SPI_INT_TXEI             0b00000000000000000000000000000001 // BIT(0) TX FIFO empty
#define DW_SPI_INT_TXOI             0b00000000000000000000000000000010 // BIT(1) TX FIFO overflow
#define DW_SPI_INT_RXUI             0b00000000000000000000000000000100 // BIT(2) RX FIFO underflow
#define DW_SPI_INT_RXOI             0b00000000000000000000000000001000 // BIT(3) RX FIFO overflow
#define DW_SPI_INT_RXFI             0b00000000000000000000000000010000 // BIT(4) RX FIFO full
#define DW_SPI_INT_MSTI             0b00000000000000000000000000100000 // BIT(5) Multi-Master contention

/* Bit fields in DMACR */
#define DW_SPI_DMACR_RDMAE          0b00000000000000000000000000000001 // BIT(0)
#define DW_SPI_DMACR_TDMAE          0b00000000000000000000000000000010 // BIT(1)

typedef struct {
    rp1_spi_t* regbase;
    char *txdata;
    char *rxdata;
    uint8_t txcount;
} rp1_spi_instance_t;

typedef enum {
    SPI_OK = 0,
    SPI_ERROR = 1,
    SPI_BUSY = 2,
    SPI_TIMEOUT = 3,
    SPI_INVALID = 4
} spi_status_t;


rp1_spi_instance_t rp1_spi_init(rp1_spi_t* regbase);
void rp1_spi_set_freq(rp1_spi_instance_t* spi, uint32_t freq);
void rp1_spi_set_mode(rp1_spi_instance_t* spi, uint32_t mode);
spi_status_t rp1_spi_write_8_blocking(rp1_spi_instance_t *spi, uint8_t data);
spi_status_t rp1_spi_read_8_n_blocking(rp1_spi_instance_t *spi, uint8_t *data, uint32_t len, uint32_t timeout);
spi_status_t rp1_spi_read_32_n(rp1_spi_instance_t *spi, uint32_t *data, uint32_t len, uint32_t timeout);
spi_status_t rp1_spi_purge_rx_fifo(rp1_spi_instance_t *spi, int* dwordspurged);

#endif
