/**
 * @file adc.h
 * @author Chris McGalliard - port of L4 HAL by Luke Oxley (lcoxley@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2023-09-17
 */

#ifndef _PHAL_ADC_H
#define _PHAL_ADC_H

#include <stdbool.h>

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#include "stm32g474xx.h"
#else
#error "Please define a MCU arch"
#endif

typedef enum {
    ADC_RES_12_BIT = 0b00,
    ADC_RES_10_BIT = 0b01,
    ADC_RES_8_BIT = 0b10,
    ADC_RES_6_BIT = 0b11
} ADCResolution_t;

typedef enum {
    ADC_CLK_PRESC_2 = 0b00,
    ADC_CLK_PRESC_4 = 0b01,
    ADC_CLK_PRESC_6 = 0b10,
    ADC_CLK_PRESC_8 = 0b11,
} ADCClkPrescaler_t;

typedef enum {
    ADC_DMA_OFF      = 0b00,
    ADC_DMA_ONE_SHOT = 0b01,
    ADC_DMA_CIRCULAR = 0b11
} ADCDMAMode_t;

typedef enum {
    ADC_DATA_ALIGN_RIGHT = 0b0,
    ADC_DATA_ALIGN_LEFT = 0b1
} ADCDataAlign_t;

typedef struct {
    ADCClkPrescaler_t clock_prescaler;
    ADCResolution_t resolution;
    ADCDataAlign_t data_align;
    bool cont_conv_mode;
    ADCDMAMode_t dma_mode;
    uint8_t adc_number;
    ADC_TypeDef *periph;
} ADCInitConfig_t;

typedef enum {
    ADC_CHN_SMP_CYCLES_3    = 0b000,
    ADC_CHN_SMP_CYCLES_15   = 0b001,
    ADC_CHN_SMP_CYCLES_28   = 0b010,
    ADC_CHN_SMP_CYCLES_56   = 0b011,
    ADC_CHN_SMP_CYCLES_84   = 0b100,
    ADC_CHN_SMP_CYCLES_112  = 0b101,
    ADC_CHN_SMP_CYCLES_144  = 0b110,
    ADC_CHN_SMP_CYCLES_480  = 0b111,
} ADCChannelSampleCycles_t;

typedef enum {
    ADC_CHANNEL_1 = 1,
    ADC_CHANNEL_2 = 2,
    ADC_CHANNEL_3 = 3,
    ADC_CHANNEL_4 = 4,
} ADCChannel_t;

typedef struct {
    ADC_TypeDef *periph;
    ADCChannel_t channel; // not the GPIO channel, use the ADC channel
    uint32_t rank;    // order at which the channels will be polled, starting at 0
    ADCChannelSampleCycles_t sampling_time;
} ADCChannelConfig_t;

// TODO DMA CONFIGS FOR ADC2/3/4/5
#define ADC1_DMA_CONT_CONFIG(mem_addr_, tx_size_, priority_)         \
    {.periph_addr=(uint32_t) &(ADC1->DR), .mem_addr=mem_addr_,       \
     .tx_size=tx_size_, .increment=true, .circular=true,             \
     .dir=0b0, .mem_inc=true, .periph_inc=false, .mem_to_mem=false,  \
     .priority=priority_, .mem_size=0b01, .periph_size=0b01,         \
     .tx_isr_en=false, .dma_chan_request=0b0000, .channel_idx=1,     \
     .periph=DMA1, .channel=DMA1_Channel1}

#define ADC1_CH1_GPIO_Port (GPIOA)
#define ADC1_CH1_Pin       (0)
#define ADC1_CH2_GPIO_Port (GPIOA)
#define ADC1_CH2_Pin       (1)
#define ADC1_CH3_GPIO_Port (GPIOA)
#define ADC1_CH3_Pin       (2)
#define ADC1_CH4_GPIO_Port (GPIOA)
#define ADC1_CH4_Pin       (3)

/**
 * @brief Initializes the ADC, requires GPIO config prior
 *
 * @param adc ADC handle
 * @param config ADC initial config settings
 * @param channels List of channel configurations
 * @param num_channels Number of channels in the channel configuration list
**/
bool PHAL_initADC(ADCInitConfig_t* config, ADCChannelConfig_t channels[], uint8_t num_channels);

/**
 * @brief Starts the ADC conversions, requires PHAL_initADC to be called prior
 *
 * @param adc ADC handle
**/
bool PHAL_startADC(ADCInitConfig_t* config);
/**
 * @brief Stops the ADC conversions, requires PHAL_initADC to be called prior
 *
 * @param adc ADC handle
**/
bool PHAL_stopADC(ADCInitConfig_t* config);

/**
 * @brief Reads the ADC data register
 *
 * @param adc ADC handle
 * @return contents of the data register
**/
uint16_t PHAL_readADC(ADCInitConfig_t* config);

#endif
