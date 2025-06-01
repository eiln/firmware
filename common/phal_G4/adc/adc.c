/**
 * @file adc.c
 * @author Eilen Yoon - Port of F4 HAL by Aditya Anand, Chris McGalliard
 * @brief
 * @version 0.1
 * @date 2023-09-17
 */

#include "common/phal_G4/adc/adc.h"

bool PHAL_initADC(ADCInitConfig_t* config, ADCChannelConfig_t channels[], uint8_t num_channels)
{
    if (num_channels >= 16) return false;

    ADC_TypeDef *adc = config->periph;
    if (adc == ADC1 || adc == ADC2)
    {
        // Enable clock to the selected peripheral
        RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;

        // Set prescaler (todo maintain acceptable bounds)
        ADC12_COMMON->CCR &= ~(ADC_CCR_PRESC_Msk);
        ADC12_COMMON->CCR |= (config->clock_prescaler << ADC_CCR_PRESC_Pos) & ADC_CCR_PRESC_Msk;
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

    // Set conversion mode on regular channels
    adc->CFGR &= ~(ADC_CFGR_CONT | ADC_CFGR_DISCEN);
    config->cont_conv_mode ? (adc->CFGR |= (ADC_CFGR_CONT)) : (adc->CFGR |= (ADC_CFGR_DISCEN));

    // Set resolution
    adc->CFGR &= ~(ADC_CFGR_RES);
    adc->CFGR |= (config->resolution << ADC_CFGR_RES_Pos) & ADC_CFGR_RES_Msk;

    // Set data alignment
    adc->CFGR &= ~(ADC_CFGR_ALIGN);
    adc->CFGR |= (config->data_align << ADC_CFGR_ALIGN_Pos) & ADC_CFGR_ALIGN_Msk;

    // Regular channel sequence length
    adc->SQR1 &= ~(ADC_SQR1_L);
    adc->SQR1 |= ((num_channels - 1) << ADC_SQR1_L_Pos) & ADC_SQR1_L_Msk;

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

    // Channel configuration
    for (int i = 0; i < num_channels; i++)
    {
        // Configure sample time: https://controllerstech.com/adc-conversion-time-frequency-calculation-in-stm32/
        if (channels[i].channel < 10)
        {
            adc->SMPR1 &= ~(1 << (ADC_SMPR1_SMP0_Pos * channels[i].channel));
            adc->SMPR1 |= (channels[i].sampling_time & ADC_SMPR1_SMP0_Msk) << (ADC_SMPR1_SMP0_Pos * channels[i].channel);
        }
        else if (channels[i].channel >= 10 && channels[i].channel < 19)
        {
            adc->SMPR2 &= ~(1 << (ADC_SMPR2_SMP10_Pos * (channels[i].channel - 10)));
            adc->SMPR2 |= (channels[i].sampling_time & ADC_SMPR2_SMP10_Msk) << (ADC_SMPR2_SMP10_Pos * (channels[i].channel - 10));
        }

        // Sequence rank
        if (channels[i].rank < 4)
        {
            adc->SQR1 &= ~(0b111 << ((channels[i].rank + 1) * 6));
            adc->SQR1 |= ((channels[i].channel & 0b111) << ((channels[i].rank + 1) * 6));
        }
        else if (channels[i].rank < 9)
        {
            adc->SQR2 &= ~(0b111 << ((channels[i].rank - 4) * 6));
            adc->SQR2 |= ((channels[i].channel & 0b111) << ((channels[i].rank - 4) * 6));
        }
        else if (channels[i].rank < 16)
        {
            adc->SQR3 &= ~(0b111 << ((channels[i].rank - 9) * 6));
            adc->SQR3 |= ((channels[i].channel & 0b111) << ((channels[i].rank - 9) * 6));
        }
    }

    adc->CR |= ADC_CR_ADEN;

    return true;
}

bool PHAL_startADC(ADC_TypeDef* adc)
{
    adc->CR |= ADC_CR_ADSTART;
    return true;
}

bool PHAL_stopADC(ADC_TypeDef* adc)
{
    if (adc->CR & ADC_CR_ADSTART)
    {
        adc->CR |= ADC_CR_ADSTP;
    }
    adc->CR &= ~ADC_CR_ADSTART;
    return true;
}

uint16_t PHAL_readADC(ADC_TypeDef* adc)
{
    return (uint16_t) (adc->DR & ADC_DR_RDATA_Msk);
}
