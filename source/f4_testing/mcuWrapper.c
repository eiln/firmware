
/*******************************************************************************
Copyright (c) 2020 - Analog Devices Inc. All Rights Reserved.
This software is proprietary & confidential to Analog Devices, Inc.
and its licensor.
******************************************************************************
* @file:    mcuWrapper.c
* @brief:   BMS SPI driver functions
* @version: $Revision$
* @date:    $Date$
* Developed by: ADIBMS Software team, Bangalore, India
*****************************************************************************/
/*! \addtogroup MCU DRIVER
*  @{
*/

/*! @addtogroup Mcu Driver
*  @{
*/

#include "main.h"
#include "adBms6830GenericType.h"
#include "adBms6830Data.h"
#include "common/phal_F4_F7/spi/spi.h"
#include "adBms6830ParseCreate.h"

#define WAKEUP_DELAY 1                          /* BMS ic wakeup delay  */

/**
 *******************************************************************************
 * Function: Delay_ms
 * @brief Delay mili second
 *
 * @details This function insert delay in ms.
 *
 * Parameters:
 * @param [in]  delay   Delay_ms
 *
 * @return None
 *
 *******************************************************************************
*/
void Delay_ms(uint32_t delay)
{
    uint32_t start = tick_ms;
    while (tick_ms - start < delay);
    //wait_ms((int)delay);
}

/**
 *******************************************************************************
 * Function: adBmsCsLow
 * @brief Select chip select low
 *
 * @details This function does spi chip select low.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsLow()
{
  #if 0
  spi.lock();
  chip_select = 0;
  #else
  eth_spi_config.periph->CR1 |= SPI_CR1_SPE;
  PHAL_writeGPIO(SPI_CS_PORT, SPI_CS_PIN, 0);
  #endif
}

/**
 *******************************************************************************
 * Function: adBmsCsHigh
 * @brief Select chip select High
 *
 * @details This function does spi chip select high.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsHigh()
{
  #if 0
  chip_select = 1;
  spi.unlock();
  #else
  eth_spi_config.periph->CR1 &= ~SPI_CR1_SPE;
  PHAL_writeGPIO(SPI_CS_PORT, SPI_CS_PIN, 1);
  #endif
}

/**
 *******************************************************************************
 * Function: adBmsWakeupIc
 * @brief Wakeup bms ic using chip select
 *
 * @details This function wakeup thr bms ic using chip select.
 *
 * @param [in]  total_ic    Total_ic
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsWakeupIc(uint8_t total_ic)
{
  for (uint8_t ic = 0; ic < total_ic; ic++)
  {
    adBmsCsLow();
    Delay_ms(WAKEUP_DELAY);
    adBmsCsHigh();
    Delay_ms(WAKEUP_DELAY);
  }
}

#define printf debug_printf

static char ascii(char s)
{
    if (s < 0x20)
        return '.';
    if (s > 0x7E)
        return '.';
    return s;
}

void hexdump(const void *d, size_t len)
{
    uint8_t *data;
    size_t i, off;
    data = (uint8_t *)d;
    for (off = 0; off < len; off += 16) {
        printf("%08lx  ", off);
        for (i = 0; i < 16; i++) {
            if ((i + off) >= len)
                printf("   ");
            else
                printf("%02x ", data[off + i]);
        }

        printf(" ");
        for (i = 0; i < 16; i++) {
            if ((i + off) >= len)
                printf(" ");
            else
                printf("%c", ascii(data[off + i]));
        }
        printf("\n");
    }
}

/**
 *******************************************************************************
 * Function: spiWriteBytes
 * @brief Writes an array of bytes out of the SPI port.
 *
 * @details This function wakeup bms ic in IsoSpi mode send dumy byte data in spi line..
 *
 * @param [in]  size            Numberof bytes to be send on the SPI line
 *
 * @param [in]  *tx_Data    Tx data pointer
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteBytes
(
uint16_t size,                     /*Option: Number of bytes to be written on the SPI port*/
uint8_t *tx_data                       /*Array of bytes to be written on the SPI port*/
)
{
  #if 0
  uint8_t rx_data[size];
  spi.write((char *)tx_data, size ,(char *)rx_data, size);
  #else
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, size, 0, NULL);
  PHAL_SPI_transfer_noDMA(&eth_spi_config, tx_data, size, 0, NULL);
  #endif
}

/**
 *******************************************************************************
 * Function: spiWriteReadBytes
 * @brief Writes and read a set number of bytes using the SPI port.
 *
 * @details This function writes and read a set number of bytes using the SPI port.
 *
 * @param [in]  *tx_data    Tx data pointer
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size            Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteReadBytes
(
uint8_t *tx_data,                   /*array of data to be written on SPI port*/
uint8_t *rx_data,                   /*Input: array that will store the data read by the SPI port*/
uint16_t size                           /*Option: number of bytes*/
)
{
  //TODO what the FUCK is this?
  #if 0
  uint16_t data_size = (4 + size);
  uint8_t cmd[data_size];
  memcpy(&cmd[0], &tx_data[0], 4); /* dst, src, size */
  spi.write((char *)cmd, data_size ,(char *)cmd, data_size);
  memcpy(&rx_data[0], &cmd[4], size); /* dst, src, size */
  #else
  #if 0
  uint16_t data_size = (4 + size);
  uint8_t cmd[data_size];
  PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, data_size, data_size, cmd);
  memcpy(&rx_data[0], &cmd[4], size); /* dst, src, size */
  #endif
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, size, 0, NULL);
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, 4, size, rx_data);
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, size, size, rx_data)
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, 4, size, tx_data);
  //memcpy(&rx_data[0], &tx_data[4], size); /* dst, src, size */
  uint16_t data_size = (4 + size);
  uint8_t cmd[data_size];
  hexdump(tx_data, 4);
  memcpy(&cmd[0], &tx_data[0], 4); /* dst, src, size */
  PHAL_SPI_transfer_noDMA(&eth_spi_config, cmd, data_size, data_size, cmd);
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, 4, size, rx_data);
  hexdump(cmd, data_size);
  memcpy(&rx_data[0], &cmd[4], size); /* dst, src, size */
  hexdump(rx_data, size);
  //memcpy(&rx_data[0], &cmd[4], size); /* dst, src, size */
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, 4, 0, NULL);
  //PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, NULL, 0, size, rx_data);
  #endif
}

/**
 *******************************************************************************
 * Function: spiReadBytes
 * @brief Read number of bytes using the SPI port.
 *
 * @details This function Read a set number of bytes using the SPI port.
 *
 * @param [in]  size            Data size
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @return None
 *
 *******************************************************************************
*/
void spiReadBytes(uint16_t size, uint8_t *rx_data)
{
  // TODO chnage to 0xff?
  #if 0
  uint8_t tx_data[size];
  for(uint8_t i=0; i < size; i++)
  {
    tx_data[i] = 0xFF;
  }
  spi.write((char *)tx_data, size ,(char *)rx_data, size);
  #endif
  #if 0
  uint8_t tx_data[size];
  for(uint8_t i=0; i < size; i++)
  {
    tx_data[i] = 0xFF;
  }
  PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, tx_data, size, size, rx_data);
  #endif
  PHAL_SPI_transfer_noDMA_DAQW5500Only2(&eth_spi_config, NULL, 0, size, rx_data);
}

#if 0
/**
 *******************************************************************************
 * Function: startTimer()
 * @brief Start timer
 *
 * @details This function start the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void startTimer()
{
  timer.start();
}

/**
 *******************************************************************************
 * Function: stopTimer()
 * @brief Stop timer
 *
 * @details This function stop the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void stopTimer()
{
  timer.stop();
}

/**
 *******************************************************************************
 * Function: getTimCount()
 * @brief Get Timer Count Value
 *
 * @details This function return the timer count value.
 *
 * @return tim_count
 *
 *******************************************************************************
*/
uint32_t getTimCount()
{
  uint32_t count = 0;
  count = timer.read_us();
  timer.reset();
  return(count);
}
#endif
#if 0

#define SPI_TIME_OUT HAL_MAX_DELAY              /* SPI Time out delay   */
#define UART_TIME_OUT HAL_MAX_DELAY             /* UART Time out delay  */
#define I2C_TIME_OUT HAL_MAX_DELAY              /* I2C Time out delay   */

SPI_HandleTypeDef *hspi         = &hspi1;       /* MUC SPI Handler      */
UART_HandleTypeDef *huart       = &huart5;      /* MUC UART Handler     */
I2C_HandleTypeDef *hi2c         = &hi2c1;       /* MUC I2C Handler      */
TIM_HandleTypeDef *htim         = &htim2;       /* Mcu TIM handler */


/**
 *******************************************************************************
 * Function: Delay_ms
 * @brief Delay mili second
 *
 * @details This function insert delay in ms.
 *
 * Parameters:
 * @param [in]  delay   Delay_ms
 *
 * @return None
 *
 *******************************************************************************
*/
void Delay_ms(uint32_t delay)
{
  HAL_Delay(delay);
}

/**
 *******************************************************************************
 * Function: adBmsCsLow
 * @brief Select chip select low
 *
 * @details This function does spi chip select low.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsLow()
{
  HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_RESET);
}

/**
 *******************************************************************************
 * Function: adBmsCsHigh
 * @brief Select chip select High
 *
 * @details This function does spi chip select high.
 *
 * @return None
 *
 *******************************************************************************
*/
void adBmsCsHigh()
{
  HAL_GPIO_WritePin(GPIO_PORT, CS_PIN, GPIO_PIN_SET);
}

/**
 *******************************************************************************
 * Function: spiWriteBytes
 * @brief Writes an array of bytes out of the SPI port.
 *
 * @details This function wakeup bms ic in IsoSpi mode send dumy byte data in spi line..
 *
 * @param [in]  size            Numberof bytes to be send on the SPI line
 *
 * @param [in]  *tx_Data    Tx data pointer
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteBytes
(
uint16_t size,                     /*Option: Number of bytes to be written on the SPI port*/
uint8_t *tx_Data                       /*Array of bytes to be written on the SPI port*/
)
{
  HAL_SPI_Transmit(hspi, tx_Data, size, SPI_TIME_OUT); /* SPI1 , data, size, timeout */
}

/**
 *******************************************************************************
 * Function: spiWriteReadBytes
 * @brief Writes and read a set number of bytes using the SPI port.
 *
 * @details This function writes and read a set number of bytes using the SPI port.
 *
 * @param [in]  *tx_data    Tx data pointer
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @param [in]  size            Data size
 *
 * @return None
 *
 *******************************************************************************
*/
void spiWriteReadBytes
(
uint8_t *tx_data,                   /*array of data to be written on SPI port*/
uint8_t *rx_data,                   /*Input: array that will store the data read by the SPI port*/
uint16_t size                           /*Option: number of bytes*/
)
{
  HAL_SPI_Transmit(hspi, tx_data, 4, SPI_TIME_OUT);
  HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT);
}

/**
 *******************************************************************************
 * Function: spiReadBytes
 * @brief Read number of bytes using the SPI port.
 *
 * @details This function Read a set number of bytes using the SPI port.
 *
 * @param [in]  size            Data size
 *
 * @param [in]  *rx_data    Rx data pointer
 *
 * @return None
 *
 *******************************************************************************
*/
void spiReadBytes(uint16_t size, uint8_t *rx_data)
{
  HAL_SPI_Receive(hspi, rx_data, size, SPI_TIME_OUT);
}

/**
 *******************************************************************************
 * Function: startTimer()
 * @brief Start timer
 *
 * @details This function start the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void startTimer()
{
  HAL_TIM_Base_Start(htim);
}

/**
 *******************************************************************************
 * Function: stopTimer()
 * @brief Stop timer
 *
 * @details This function stop the timer.
 *
 * @return None
 *
 *******************************************************************************
*/
void stopTimer()
{
  HAL_TIM_Base_Stop(htim);
}

/**
 *******************************************************************************
 * Function: getTimCount()
 * @brief Get Timer Count Value
 *
 * @details This function return the timer count value.
 *
 * @return tim_count
 *
 *******************************************************************************
*/
uint32_t getTimCount()
{
  uint32_t count = 0;
  count = __HAL_TIM_GetCounter(htim);
  __HAL_TIM_SetCounter(htim, 0);
  return(count);
}

#endif
