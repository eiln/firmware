/**
 * @file adc.c
 * @author Eilen Yoon - Port of F4 HAL by Aditya Anand, Chris McGalliard
 * @brief
 * @version 0.1
 * @date 2023-09-17
 */

#include "common/phal_G4/adc/adc.h"

#include <stdint.h>

static bool PHAL_configureADCChannels(ADCInitConfig_t* config, ADCChannelConfig_t channels[], uint8_t num_channels)
{
    ADC_TypeDef *adc = config->periph;

    // Clear all SQR ranks
    adc->SQR1 = 0;
    adc->SQR2 = 0;
    adc->SQR3 = 0;
    adc->SQR4 = 0;

    // Set number of channels
    adc->SQR1 |= (num_channels - 1) << ADC_SQR1_L_Pos;

    // Configure sequence and sample time ---
    for (uint8_t i = 0; i < num_channels; i++) {
        uint8_t ch = channels[i].channel;
        uint8_t rank = channels[i].rank;
        if (rank <= 0) return false;
        rank = rank - 1; // PHAL does 1-based start

        // Set rank for channel
        if (rank < 4) {
            adc->SQR1 &= ~(0b111111 << (6 + rank * 6));
            adc->SQR1 |= ((uint32_t)ch << (6 + rank * 6));
        } else if (rank < 9) {
            adc->SQR2 &= ~(0b111111 << ((rank - 4) * 6));
            adc->SQR2 |= ((uint32_t)ch << ((rank - 4) * 6));
        } else if (rank < 14) {
            adc->SQR3 &= ~(0b111111 << ((rank - 9) * 6));
            adc->SQR3 |= ((uint32_t)ch << ((rank - 9) * 6));
        } else if (rank < 16) {
            ADC1->SQR4 &= ~(0b111111 << ((rank - 14) * 6));
            ADC1->SQR4 |= ((uint32_t)ch << ((rank - 14) * 6));
        } else {
            return false;
        }

        // Set sampling time for channel
        if (ch <= 9) {
            adc->SMPR1 &= ~(0b111 << (ch * 3));
            adc->SMPR1 |=  (channels[i].sampling_time << (ch * 3));
        } else {
            uint8_t ch2 = ch - 10;
            adc->SMPR2 &= ~(0b111 << (ch2 * 3));
            adc->SMPR2 |=  (channels[i].sampling_time << (ch2 * 3));
        }
    }

    return true;
}

bool PHAL_initADC(ADCInitConfig_t* config, ADCChannelConfig_t channels[], uint8_t num_channels)
{
    if (num_channels >= 16) return false;

    ADC_TypeDef *adc = config->periph;
    if (adc == ADC1 || adc == ADC2)
    {
        // Enable clock to the selected peripheral
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

        #define ADC_CKMODE_DIV1  (0x1UL << ADC_CCR_CKMODE_Pos)
        ADC12_COMMON->CCR &= ~ADC_CCR_CKMODE;
        ADC12_COMMON->CCR |= ADC_CKMODE_DIV1;
    }
    else if (adc == ADC3 || adc == ADC4 || adc == ADC5)
    {
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC345EN;

        ADC345_COMMON->CCR &= ~(ADC_CCR_PRESC_Msk);
        ADC345_COMMON->CCR |= (config->clock_prescaler << ADC_CCR_PRESC_Pos) & ADC_CCR_PRESC_Msk;
    }
    else
    {
        return false;
    }

    adc->CR &= ~ADC_CR_DEEPPWD;
    adc->CR |= ADC_CR_ADVREGEN;
    for (volatile int i = 0; i < 1000; ++i); // Short delay

    // 1. Ensure ADC is disabled before calibration
    if (adc->CR & ADC_CR_ADEN) {
        adc->CR |= ADC_CR_ADDIS; // Disable ADC if it was enabled
        while (adc->CR & ADC_CR_ADEN); // Wait until disabled
    }

    // Calibrate ADC
    adc->CR |= ADC_CR_ADCAL;
    while (adc->CR & ADC_CR_ADCAL); // Wait for calibration to finish

    // Set conversion mode on regular channels
    // adc->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_DISCEN);
    // config->cont_conv_mode ? (adc->CFGR |= (ADC_CFGR_CONT)) : (adc->CFGR |= (ADC_CFGR_DISCEN));

    // Set resolution
    adc->CFGR &= ~(ADC_CFGR_RES);
    adc->CFGR |= (config->resolution << ADC_CFGR_RES_Pos) & ADC_CFGR_RES_Msk;  // 12-bit resolution

    // Set data alignment
    adc->CFGR &= ~(ADC_CFGR_ALIGN);
    adc->CFGR |= (config->data_align << ADC_CFGR_ALIGN_Pos) & ADC_CFGR_ALIGN_Msk;

    if (!PHAL_configureADCChannels(config, channels, num_channels)) return false;

#if 0
    // DMA configuration
    while (adc->CR & ADC_CR_ADSTART || adc->CR & ADC_CR_JADSTART);
    if (config->dma_mode != ADC_DMA_OFF)
    {
        adc->CFGR |= ADC_CFGR_DMAEN;
        adc->CFGR |= ((config->dma_mode == ADC_DMA_CIRCULAR) << ADC_CFGR_DMACFG_Pos) & ADC_CFGR_DMACFG_Msk; // Circular or one shot
    }
    else
    {
        // Disable ADC DMA Mode
        adc->CFGR &= ~(ADC_CFGR_DMAEN);
    }
#endif

    // Enable ADC
    adc->ISR |= ADC_ISR_ADRDY;  // Clear ready flag
    adc->CR |= ADC_CR_ADEN;     // Enable ADC
    while (!(adc->ISR & ADC_ISR_ADRDY)); // Wait until ready

    return true;
}

uint16_t PHAL_readADC(ADCInitConfig_t* config)
{
    ADC_TypeDef *adc = config->periph;
    adc->CR |= ADC_CR_ADSTART; // Start conversion
    while (!(adc->ISR & ADC_ISR_EOC)); // Wait for end of conversion
    return (uint16_t)adc->DR; // Read result
}

bool PHAL_startADC(ADCInitConfig_t* config)
{
    #if 0
    ADC_TypeDef *adc = config->periph;
    adc->CR |= ADC_CR_ADSTART;
    adc->CFGR2 |= ADC_CFGR2_SWTRIG;
    #endif
    return true;
}

bool PHAL_stopADC(ADCInitConfig_t* config)
{
    ADC_TypeDef *adc = config->periph;
    if (adc->CR & ADC_CR_ADSTART)
    {
        adc->CR |= ADC_CR_ADSTP;
    }
    adc->CR &= ~ADC_CR_ADSTART;
    return true;
}
