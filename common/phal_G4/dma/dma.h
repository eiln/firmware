/**
 * @file dma.h
 * @author Chris McGalliard - Port of L4 HAL by Dawson Moore (moore800@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2023-08-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef _DMA_H_
#define _DMA_H_

#include <stdbool.h>
#include <stdint.h>

#if defined(STM32G474xx)
#include "stm32g4xx.h"
#else
#error "Please define a MCU arch"
#endif

typedef enum {
    DMA_SIZE_8BIT  = 0,
    DMA_SIZE_16BIT = 1,
    DMA_SIZE_32BIT = 2
} dma_size_t;

typedef struct {
    uint32_t    periph_addr;
    uint32_t    mem_addr;
    uint16_t    tx_size;
    uint8_t     mem_size;

    bool        increment;
    bool        circular;
    uint8_t     dir;
    bool        mem_inc;
    bool        periph_inc;
    bool        mem_to_mem;
    uint8_t     priority;
    uint8_t     periph_size;
    bool        tx_isr_en;
    uint8_t     dma_chan_request;
    uint8_t     channel_idx;

    DMA_TypeDef* periph;
    DMA_Channel_TypeDef* channel; // Example DMA1_Stream0 or DMA2_Stream7
} dma_init_t;

/*
 * @brief Initialize DMA peripheral to set m2m, p2p, or p2m with set size
 *        and length of txfer
 *
 * @param dma -> Address of initialization structure
 * @return true -> Successful init (no clashing params)
 * @return false -> Init not complete (parameters clash)
 */
bool PHAL_initDMA(dma_init_t* dma);

/*
 * @brief Start txfer after sucessful DMA peripheral initialization
 *
 * @param dma -> Address of initialization structure
 */
void PHAL_startTxfer(dma_init_t* dma);

/*
 * @brief Stop txfer
 *
 * @param dma -> Address of initialization structure
 */
void PHAL_stopTxfer(dma_init_t* dma);

/*
 * @brief Re-enable DMA txfer after error ISR fires
 *
 * @param dma -> Address of initialization structure
 */
void PHAL_reEnable(dma_init_t* dma);

/*
 * @brief Set memory address for DMA transfer. In Mem to Mem this acts as the source address
 *
 * @param dma -> Address of initialization structure
 */
void PHAL_DMA_setMemAddress(dma_init_t* dma, const uint32_t address);

/*
 * @brief Set transfer length for DMA transaction
 *
 * @param dma -> Address of initialization structure
 */
void PHAL_DMA_setTxferLength(dma_init_t* dma, const uint32_t length);

#endif