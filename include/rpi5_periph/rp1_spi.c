#include "rp1_spi.h"


rp1_spi_instance_t rp1_spi_init(rp1_spi_t* regbase)
{
    rp1_spi_instance_t spi;
    spi.regbase = regbase;
    spi.txdata = (char *)0x0;
    spi.rxdata = (char *)0x0;
    spi.txcount = 0x0;
    return spi;
}


/// @brief Writes 8 bits of data to the SPI bus, blocking until it can write and until the write is complete
/// @param spi SPI instance
/// @param data 8 bits of data to write (unsigned char)
/// @return SPI_OK if successful - note it does not return if busy - it waits until the SPI is not busy
spi_status_t rp1_spi_write_8_blocking(rp1_spi_instance_t *spi, uint8_t data)
{

    // wait until the spi is not busy
    while((spi->regbase->sr) & DW_SPI_SR_BUSY)
    {
        ;
    }

    spi->txdata = &data;
    
    // spin until we can write to the fifo
    while(!((spi->regbase->sr) & DW_SPI_SR_TF_NOT_FULL))
    {
       ;
    }

    // set the CS pin
    (spi->regbase->ser) = 1 <<0;

    // put the data into the fifo
    *(volatile uint8_t *)(spi->regbase->dr) = data;

    // we now need to pull exactly one byte out of the fifo which would
    // have been clocked in when we wrote the data    
    
    while( (!(spi->regbase->sr) & DW_SPI_SR_RF_NOT_EMPT) || ((spi->regbase->sr) & DW_SPI_SR_BUSY))   // check if there is data to read (check status register for Read Fifo Not Empty)
    {
        ;
    }
    /*uint8_t discard = */*(volatile uint8_t *)(spi->regbase->dr);
    //printf("write_8 - discarded: %d\n", discard);

    return SPI_OK;
}


/// @brief Reads a number of 8-bit bytes from the SPI bus, blocking until the read is complete
/// @param spi SPI instance
/// @param data buffer to read into
/// @param len number of bytes to read
/// @param timeout timeout in ms - not yet implemented
/// @return 
spi_status_t rp1_spi_read_8_n_blocking(rp1_spi_instance_t *spi, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if (spi->txcount != 0)
        return SPI_BUSY;
    if (len == 0)
        return SPI_INVALID;
   spi->txcount = len;

    // how this works
    // 1. We stuff the TX FIFO with dummy data (zeros, but can be anything you want) until it is full,
    //    or we have stuffed the number of bytes we want to read (we write in order to generate the 
    //    clock pulses to the slave, which sends us data to read)
    // 2. We then set the CS pin to active transmission
    // 3. We then continue to feed the TX FIFO with dummy data until we have sent all the bytes 
    //    we want to send, if we didn't manage them all in step 1
    // 4. As we are feeding the TX FIFO with dummy data, we are clocking out data from the slave
    //    which we read from the RX fifo and discard
    // 5. Once we have sent all the data, we then read the RX FIFO until it is empty, and store the data
    //    in the buffer we were passed
    // 6. The CS pin is turned off by the hardware when the last bit is clocked out

    // pre-stuff the TX buffer with dummy data
    while(((spi->regbase->sr) & DW_SPI_SR_TF_NOT_FULL) && (spi->txcount > 0))
    {
        *(volatile uint8_t *)(spi->regbase->dr) = (uint8_t)0x00;
        spi->txcount--;
    }
    
    // set the CS pin - since we have pre-stuffed data, the clock should start here
    // note the behaviour of te CS pin (active low, or high) is determined by the hardware
    // and the GPIO / PAD settings, but default is active low
    (spi->regbase->ser) = 1 << 0;  // TODO - fix this to use the correct CS pin
    
    int inbyte = 0;
    // keep loading data into the tx fifo and also see if we have anything to read in the rx fifo
    while(((spi->regbase->sr) & DW_SPI_SR_TF_NOT_FULL) && (spi->txcount > 0))
    {
        *(volatile uint8_t *)(spi->regbase->dr) = (uint8_t)0x00;
        spi->txcount--;
        // check if there is data to read (check status register for Read Fifo Not Empty)
        if((spi->regbase->sr) & DW_SPI_SR_RF_NOT_EMPT)
        {
            data[inbyte] = *(volatile uint8_t *)(spi->regbase->dr);
            inbyte++;
        }
    }

    // read the remaining bytes from the buffer
    while(inbyte < len)
    {
        // check if there is data to read (check status register for Read Fifo Not Empty)
        if((spi->regbase->sr) & DW_SPI_SR_RF_NOT_EMPT)
        {
            data[inbyte] = *(volatile uint8_t *)(spi->regbase->dr);
            inbyte++;
        }
    }

    return SPI_OK;
}

spi_status_t rp1_spi_read_32_n(rp1_spi_instance_t *spi, uint32_t *data, uint32_t len, uint32_t timeout)
{
    if (spi->txcount != 0)
        return SPI_BUSY;
    if (len == 0)
        return SPI_INVALID;

    //todo: implement timeout

   
    spi->txcount = len;

    // set the frame size to 32 bits
    (spi->regbase->ctrlr[0]) = (((spi->regbase->ctrlr[0])) | DW_PSSI_CTRLR0_DFS32_MASK | DW_PSSI_CTRLR0_DFS_MASK);

    // pre-stuff the TX buffer with dummy data
    while(((spi->regbase->sr) & DW_SPI_SR_TF_NOT_FULL) && (spi->txcount > 0))
    {
        (spi->regbase->dr) = (uint32_t)0x00;
        spi->txcount--;
    }


    // set the CS pin
    (spi->regbase->ser) = 1 <<0;
    
    int indw = 0;
    // now load the dummy data into the TX FIFO to start the clock
    while(((spi->regbase->sr) & DW_SPI_SR_TF_NOT_FULL) && (spi->txcount > 0))
    {
        (spi->regbase->dr) = (uint32_t)0x00;
        spi->txcount--;
        // check if there is data to read (check status register for Read Fifo Not Empty)
        if((spi->regbase->sr) & DW_SPI_SR_RF_NOT_EMPT)
        {
            data[indw] = (spi->regbase->dr);
            indw++;
        }
    
    }

    // read the remaining dwords from the buffer
    while(indw < len)
    {
        // check if there is data to read (check status register for Read Fifo Not Empty)
        if((spi->regbase->sr) & DW_SPI_SR_RF_NOT_EMPT)
        {
            data[indw] = (spi->regbase->dr);
            indw++;
        }
    }

    // turn off the CS pin
    (spi->regbase->ser) = 0x00;

    return SPI_OK;
}

spi_status_t rp1_spi_purge_rx_fifo(rp1_spi_instance_t *spi, int* dwordspurged)
{
    if (spi->txcount != 0)
        return SPI_BUSY;

    int readcount = 0;
    uint32_t temp;

    // read the remaining dwords from the buffer
    while(((spi->regbase->sr)) & DW_SPI_SR_RF_NOT_EMPT)
    {
        // check if there is data to read (check status register for Read Fifo Not Empty)
        temp = (spi->regbase->dr);
        readcount++;
    }

    *dwordspurged = readcount;

    //printf("purge: discarded %d\n", temp);

    return SPI_OK;
}
