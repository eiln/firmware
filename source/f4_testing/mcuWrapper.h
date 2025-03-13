
#ifndef __MCU_WRAPPER__
#define __MCU_WRAPPER__

#include "adBms6830Data.h"

void Delay_ms(uint32_t delay);
void adBmsCsLow();
void adBmsCsHigh();
void adBmsWakeupIc(uint8_t total_ic);
void spiWriteBytes(uint16_t size, uint8_t *tx_data);
void spiReadBytes(uint16_t size, uint8_t *rx_data);
void spiWriteReadBytes
(
uint8_t *tx_data,                   /*array of data to be written on SPI port*/
uint8_t *rx_data,                   /*Input: array that will store the data read by the SPI port*/
uint16_t size                           /*Option: number of bytes*/
);

#endif // __MCU_WRAPPER__
