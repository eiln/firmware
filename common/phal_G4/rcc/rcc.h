/**
 * @file rcc.h
 * @author Eileen Yoon - Port of L4 RCC by Adam Busch (busch8@purdue.edu)
 * @brief RCC Configuration Driver for STM32F4 Devices
 * @version 0.1
 * @date 2023-08-16
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef __PHAL_G4_RCC_H__
#define __PHAL_G4_RCC_H__

#include "common/phal_G4/phal_g4.h"

#define RCC_144_MHZ (144000000)
#define RCC_16_MHZ (16000000)
#define RCC_8_MHZ (8000000)

#define RCC_TARGET_HZ (RCC_144_MHZ)

typedef struct {
	bool use_hse;
	uint32_t target_hz;
	uint32_t input_hz; // HSE hz
	bool hse_crystal;  // if HSE is crystal
} rcc_config_t;

bool PHAL_configureClockRates(const rcc_config_t *config);

#endif // __PHAL_G4_RCC_H__
