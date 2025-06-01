/**
 * @file dma.c
 * @author Eileen Yoon - Port of F4 DMA library by Aditya Anand, Chris McGalliard
 * @brief Basic DMA Peripheral HAL library for setting up DMA transfers
 * @version 0.1
 * @date 2023-08-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "common/phal_G4/dma/dma.h"

bool PHAL_initDMA(dma_init_t* dma) {
    // Check we aren't going to break the peripheral
    if (dma->mem_to_mem && dma->circular) {
        return false;
    } else if (dma->dir > 1) {
        return false;
    } else if (dma->priority > 3) {
        return false;
    } else if (dma->mem_size > 2 || dma->periph_size > 2) {
        return false;
    }

    // Enable clock in RCC
    if (dma->periph == DMA1) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    } else if (dma->periph == DMA2) {
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    } else {
        return false;
    }

    // Ensure the stream is disabled, must be in order to configure the DMA control registers
    dma->channel->CCR &= ~(DMA_CCR_EN);
    while (dma->channel->CCR & DMA_CCR_EN);

    // Clear any stream dedicated status flags that may have been set previously
    dma->periph->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTEIF1;

    // Set peripheral port register address
    dma->channel->CPAR = dma->periph_addr;

    // Set memory address
    dma->channel->CMAR = dma->mem_addr;

    // Reset preconfigured CR values
    dma->channel->CCR = 0;
    // Set channel, priority, memory data size
    dma->channel->CCR |= (dma->mem_size   << DMA_CCR_MSIZE_Pos) |
                         (dma->priority   << DMA_CCR_PL_Pos)    |
                         (dma->mem_inc    << DMA_CCR_MINC_Pos)  |
                         (dma->periph_inc << DMA_CCR_PINC_Pos)  |
                         (dma->circular   << DMA_CCR_CIRC_Pos)  |
                         (dma->dir        << DMA_CCR_DIR_Pos)   |
                         (dma->tx_isr_en  << DMA_CCR_TEIE_Pos)  |
                         (dma->tx_isr_en  << DMA_CCR_TCIE_Pos);

    // Set stream memory configuration
    PHAL_DMA_setTxferLength(dma, dma->tx_size);

    return true;
}

void PHAL_startTxfer(dma_init_t* dma) {
    // Stream enable starts txfer
    dma->channel->CCR |= DMA_CCR_EN;
}

void PHAL_stopTxfer(dma_init_t* dma) {
    // Stream disable stops txfer
    dma->channel->CCR &= ~DMA_CCR_EN;
}

void PHAL_reEnable(dma_init_t* dma) {
    // Clear any stream dedicated status flags that may have been set previously
    dma->periph->IFCR = ((uint32_t)DMA_ISR_HTIF1 << (dma->channel_idx & 0x1FU));
    dma->channel->CCR |= DMA_CCR_EN;
}

void PHAL_DMA_setMemAddress(dma_init_t* dma, const uint32_t address)
{
    dma->channel->CMAR = address;
}

void PHAL_DMA_setTxferLength(dma_init_t* dma, const uint32_t length)
{
    dma->channel->CNDTR = length; // Set number of data to transfer
}
