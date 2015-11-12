/* mbed Microcontroller Library
 * Copyright (c) 2013 Nordic Semiconductor
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//#include <math.h>
#include "mbed-drivers/mbed_assert.h"
#include "mbed-hal/spi_api.h"
#include "cmsis.h"
#include "pinmap.h"
#include "mbed-drivers/mbed_error.h"
#include <string.h>

#define SPIS_MESSAGE_SIZE 1
volatile uint8_t m_tx_buf[SPIS_MESSAGE_SIZE] = {0};
volatile uint8_t m_rx_buf[SPIS_MESSAGE_SIZE] = {0};

// nRF51822's I2C_0 and SPI_0 (I2C_1, SPI_1 and SPIS1) share the same address.
// They can't be used at the same time. So we use two global variable to track the usage.
// See nRF51822 address information at nRF51822_PS v2.0.pdf - Table 15 Peripheral instance reference
extern volatile i2c_spi_peripheral_t i2c0_spi0_peripheral; // from i2c_api.c
extern volatile i2c_spi_peripheral_t i2c1_spi1_peripheral;

static void spi_enable_vector_interrupt(spi_t *obj, uint32_t handler, uint8_t enable);
static void spi_master_enable_interrupt(spi_t *obj, uint8_t enable);
static uint32_t spi_master_write_asynch(spi_t *obj, uint32_t TxLimit);
static uint32_t spi_master_read_asynch(spi_t *obj);
static uint32_t spi_event_check(spi_t *obj);
static void spi_buffer_tx_write(spi_t *obj);
static void spi_buffer_rx_read(spi_t *obj);
static void spi_buffer_set(spi_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length);

#define max(a,b)\
    ((a)>(b)?(a):(b))
#define min(a,b)\
    ((a)<(b)?(a):(b))

void spi_init(spi_t *obj, PinName mosi, PinName miso, PinName sclk)
{
    SPIName spi = SPI_0;

    memset(obj,0,sizeof(spi_t));

    if (i2c0_spi0_peripheral.usage == I2C_SPI_PERIPHERAL_FOR_SPI &&
            i2c0_spi0_peripheral.sda_mosi == (uint8_t)mosi &&
            i2c0_spi0_peripheral.scl_miso == (uint8_t)miso &&
            i2c0_spi0_peripheral.sclk     == (uint8_t)sclk) {
        // The SPI with the same pins is already initialized
        spi = SPI_0;
        obj->spi.peripheral = 0x1;
    } else if (i2c1_spi1_peripheral.usage == I2C_SPI_PERIPHERAL_FOR_SPI &&
            i2c1_spi1_peripheral.sda_mosi == (uint8_t)mosi &&
            i2c1_spi1_peripheral.scl_miso == (uint8_t)miso &&
            i2c1_spi1_peripheral.sclk     == (uint8_t)sclk) {
        // The SPI with the same pins is already initialized
        spi = SPI_1;
        obj->spi.peripheral = 0x2;
    } else if (i2c1_spi1_peripheral.usage == 0) {
        i2c1_spi1_peripheral.usage = I2C_SPI_PERIPHERAL_FOR_SPI;
        i2c1_spi1_peripheral.sda_mosi = (uint8_t)mosi;
        i2c1_spi1_peripheral.scl_miso = (uint8_t)miso;
        i2c1_spi1_peripheral.sclk     = (uint8_t)sclk;

        spi = SPI_1;
        obj->spi.peripheral = 0x2;
    } else if (i2c0_spi0_peripheral.usage == 0) {
        i2c0_spi0_peripheral.usage = I2C_SPI_PERIPHERAL_FOR_SPI;
        i2c0_spi0_peripheral.sda_mosi = (uint8_t)mosi;
        i2c0_spi0_peripheral.scl_miso = (uint8_t)miso;
        i2c0_spi0_peripheral.sclk     = (uint8_t)sclk;

        spi = SPI_0;
        obj->spi.peripheral = 0x1;
    } else {
        // No available peripheral
        error("No available SPI");
    }

    obj->spi.spi  = (NRF_SPI_Type *)spi;
    obj->spi.spis = (NRF_SPIS_Type *)NC;

    //master
    obj->spi.spi->POWER = 0;
    obj->spi.spi->POWER = 1;

    //NRF_GPIO->DIR |= (1<<mosi);
    NRF_GPIO->PIN_CNF[mosi] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    obj->spi.spi->PSELMOSI = mosi;

    NRF_GPIO->PIN_CNF[sclk] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    obj->spi.spi->PSELSCK = sclk;

    //NRF_GPIO->DIR &= ~(1<<miso);
    NRF_GPIO->PIN_CNF[miso] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);

    obj->spi.spi->PSELMISO = miso;

    obj->spi.spi->EVENTS_READY = 0U;

    spi_format(obj, 8, 0, SPI_MSB);  // 8 bits, mode 0, master
    spi_frequency(obj, 1000000);
}

void spi_free(spi_t *obj)
{
    (void) obj;
}

static inline void spi_disable(spi_t *obj)
{
    obj->spi.spi->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
}

static inline void spi_enable(spi_t *obj)
{
    obj->spi.spi->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
}

void spi_format(spi_t *obj, int bits, int mode, spi_bitorder_t order)
{
    uint32_t config_mode = 0;
    spi_disable(obj);

    if (bits != 8) {
        error("Only 8bits SPI supported");
    }

    switch (mode) {
        case 0:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case 1:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
            break;
        case 2:
            config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        case 3:
            config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
            break;
        default:
            error("SPI format error");
            break;
    }

    obj->spi.spi->CONFIG = (config_mode | (((order == SPI_MSB) ? SPI_CONFIG_ORDER_MsbFirst : SPI_CONFIG_ORDER_LsbFirst) << SPI_CONFIG_ORDER_Pos));

    spi_enable(obj);
}

void spi_frequency(spi_t *obj, int hz)
{
    if ((int)obj->spi.spi==NC) {
        return;
    }
    spi_disable(obj);

    if (hz<250000) { //125Kbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_K125;
    } else if (hz<500000) { //250Kbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_K250;
    } else if (hz<1000000) { //500Kbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_K500;
    } else if (hz<2000000) { //1Mbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_M1;
    } else if (hz<4000000) { //2Mbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_M2;
    } else if (hz<8000000) { //4Mbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_M4;
    } else { //8Mbps
        obj->spi.spi->FREQUENCY = (uint32_t) SPI_FREQUENCY_FREQUENCY_M8;
    }

    spi_enable(obj);
}

static inline int spi_readable(spi_t *obj)
{
    return (obj->spi.spi->EVENTS_READY == 1);
}

static inline int spi_writeable(spi_t *obj)
{
    return (obj->spi.spi->EVENTS_READY == 0);
}

static inline int spi_read(spi_t *obj)
{
    while (!spi_readable(obj)) {
    }

    obj->spi.spi->EVENTS_READY = 0;
    return (int)obj->spi.spi->RXD;
}

int spi_master_write(spi_t *obj, int value)
{
    while (!spi_writeable(obj)) {
    }
    obj->spi.spi->TXD = (uint32_t)value;
    return spi_read(obj);
}

//static inline int spis_writeable(spi_t *obj) {
//    return (obj->spi.spis->EVENTS_ACQUIRED==1);
//}

int spi_slave_receive(spi_t *obj)
{
    return obj->spi.spis->EVENTS_END;
}

int spi_slave_read(spi_t *obj)
{
    (void) obj;
    return m_rx_buf[0];
}

void spi_slave_write(spi_t *obj, int value)
{
    m_tx_buf[0]                = value & 0xFF;
    obj->spi.spis->TASKS_RELEASE   = 1;
    obj->spi.spis->EVENTS_ACQUIRED = 0;
    obj->spi.spis->EVENTS_END      = 0;
}

static void spi_enable_vector_interrupt(spi_t *obj, uint32_t handler, uint8_t enable)
{
    IRQn_Type spi_irq[] = { SPI0_TWI0_IRQn, SPI1_TWI1_IRQn };

    int instance = obj->spi.peripheral - 1;

    if (enable) {
        NVIC_SetVector(spi_irq[instance], handler);
        NVIC_EnableIRQ(spi_irq[instance]);
    } else {
        NVIC_SetVector(spi_irq[instance], handler);
        NVIC_DisableIRQ(spi_irq[instance]);
    }
}

static void spi_master_enable_interrupt(spi_t *obj, uint8_t enable)
{
    if (enable) {
        obj->spi.spi->INTENSET = SPI_INTENSET_READY_Msk;
    } else {
        obj->spi.spi->INTENCLR = SPI_INTENSET_READY_Msk;
    }
}


void spi_master_transfer(spi_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length, uint32_t handler, uint32_t event, DMAUsage hint)
{
    (void)hint;
    (void) event;
    MBED_ASSERT(hint == DMA_USAGE_NEVER); // only IRQ supported
    spi_buffer_set(obj, tx, tx_length, rx, rx_length);

    // use IRQ
    obj->spi.spi->INTENSET = SPI_INTENSET_READY_Msk;
    spi_enable_vector_interrupt(obj, handler, 1);
}

static uint32_t spi_event_check(spi_t *obj)
{
    uint32_t event = 0;
    // The transfer is only complete if the TX buffer has been sent, the RX buffer has been filled, and there are no
    // values in the TX FIFO or RX FIFO
    if ((obj->rx_buff.pos >= obj->rx_buff.length) &&
        (obj->tx_buff.pos >= obj->tx_buff.length)) {
        event |= SPI_EVENT_COMPLETE;
        event |= SPI_EVENT_INTERNAL_TRANSFER_COMPLETE; // process another transactions
    }

    return event;
}

// Write a value from the TX buffer to the TX FIFO
static void spi_buffer_tx_write(spi_t *obj)
{
    int data = SPI_FILL_WORD;

    if ((obj->tx_buff.buffer) && (obj->tx_buff.pos < obj->tx_buff.length)) {
        uint8_t *tx = (uint8_t *)(obj->tx_buff.buffer);
        data = tx[obj->tx_buff.pos];
    }
    // Increment the buffer position
    obj->tx_buff.pos++;
    // Send the data
    obj->spi.spi->TXD = (uint32_t)data;
}

static void spi_buffer_rx_read(spi_t *obj)
{
    // Read a word from the RX FIFO
    int data = (int)((obj->spi.spi->RXD)&0xFF);

    // Disregard the word if the rx buffer is full or not present
    if (obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length) {
        // store the word to the rx buffer
        uint8_t *rx = (uint8_t *)(obj->rx_buff.buffer);
        rx[obj->rx_buff.pos] = data;
    }
    // advance the buffer position
    obj->rx_buff.pos++;
}

/**
 * Send words from the SPI TX buffer until the send limit is reached or the TX FIFO is full
 * TxLimit is provided to ensure that the number of SPI frames (words) in flight can be managed.
 * @param[in] obj     The SPI object on which to operate
 * @param[in] TxLimit The maximum number of words to send
 * @return The number of SPI frames that have been transfered
 */
static uint32_t spi_master_write_asynch(spi_t *obj, uint32_t TxLimit)
{
    uint32_t ndata = 0;
    // Determine the number of frames to send
    uint32_t txRemaining = obj->tx_buff.length - min(obj->tx_buff.pos, obj->tx_buff.length);
    uint32_t rxRemaining = obj->rx_buff.length - min(obj->tx_buff.pos, obj->rx_buff.length);
    uint32_t maxTx = max(txRemaining, rxRemaining);
    maxTx = min(maxTx, TxLimit);
    // Send words until the FIFO is full or the send limit is reached
    if (ndata < maxTx) {
        spi_buffer_tx_write(obj);
        ndata++;
    }
    //Return the number of frames that have been sent
    return ndata;
}

/**
 * Read SPI frames out of the RX FIFO
 * Continues reading frames out of the RX FIFO until the following condition is met:
 * o There are no more frames in the FIFO
 * OR BOTH OF:
 * o At least as many frames as the TX buffer have been received
 * o At least as many frames as the RX buffer have been received
 * This way, RX overflows are not generated when the TX buffer size exceeds the RX buffer size
 * @param[in] obj The SPI object on which to operate
 * @return Returns the number of frames extracted from the RX FIFO
 */
static uint32_t spi_master_read_asynch(spi_t *obj)
{
    uint32_t ndata = 0;
    // Calculate the maximum number of frames to receive
    uint32_t txRemaining = obj->tx_buff.length - min(obj->rx_buff.pos, obj->tx_buff.length);
    uint32_t rxRemaining = obj->rx_buff.length - min(obj->rx_buff.pos, obj->rx_buff.length);
    uint32_t maxRx = max(txRemaining, rxRemaining);
    // Receive frames until the maximum is reached or the RX FIFO is empty
    if (ndata < maxRx) {
        spi_buffer_rx_read(obj);
        ndata++;
    }
    // Return the number of frames received
    return ndata;
}

/**
 * Abort an SPI transfer
 * This is a helper function for event handling. When any of the events listed occurs, the HAL will abort any ongoing
 * transfers
 * @param[in] obj The SPI peripheral to stop
 */
void spi_abort_asynch(spi_t *obj)
{
    spi_enable_vector_interrupt(obj, 0, 0);
}

/**
 * Handle the SPI interrupt
 * Read frames until the RX FIFO is empty.  Write at most as many frames as were read.  This way,
 * it is unlikely that the RX FIFO will overflow.
 * @param[in] obj The SPI peripheral that generated the interrupt
 * @return
 */
uint32_t spi_irq_handler_asynch(spi_t *obj)
{
    uint32_t result = 0;
    // Read frames until the RX FIFO is empty
    uint32_t r = spi_master_read_asynch(obj);
    // Write at most the same number of frames as were received
    spi_master_write_asynch(obj, r);

    obj->spi.spi->EVENTS_READY = 0;

    // Check for SPI events
    uint32_t event = spi_event_check(obj);
    if (event) {
        result = event;
        if (event & SPI_EVENT_COMPLETE) {
            // adjust buffer positions
            obj->tx_buff.pos = obj->tx_buff.length;
            obj->rx_buff.pos = obj->rx_buff.length;
        }
    }

    if (result) {
        spi_master_enable_interrupt(obj, 0);
    }
    return result;

}

uint8_t spi_active(spi_t *obj)
{
    /* Check if rx or tx buffers are set and check if any more bytes need to be transferred. */
    if ((obj->rx_buff.buffer && obj->rx_buff.pos < obj->rx_buff.length)
            || (obj->tx_buff.buffer && obj->tx_buff.pos < obj->tx_buff.length) ){
        return 1;
    } else  {
        // interrupts are disabled, all transaction have been completed
        // TODO: checking rx fifo, it reports data eventhough RFDF is not set
        return 0;
    }

}

static void spi_buffer_set(spi_t *obj, void *tx, size_t tx_length, void *rx, size_t rx_length)
{
    obj->tx_buff.buffer = tx;
    obj->tx_buff.length = tx_length;
    obj->tx_buff.pos = 0;
    obj->tx_buff.width = 0;
    obj->rx_buff.buffer = rx;
    obj->rx_buff.length = rx_length;
    obj->rx_buff.pos = 0;
    obj->rx_buff.width = 0;
}
