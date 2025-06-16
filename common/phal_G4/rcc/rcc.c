/**
 * @file rcc.h
 * @author Eileen Yoon - Port of L4 RCC by Adam Busch (busch8@purdue.edu)
 * @brief RCC Configuration Driver for STM32F4 Devices
 * @version 0.1
 * @date 2025-06-15
 */

#include "common/phal_G4/rcc/rcc.h"

static bool PHAL_configurePLLRates(const rcc_config_t *config);
static bool PHAL_configurePLLSystemClock(const rcc_config_t *config);
static bool PHAL_configureHSISystemClock(const rcc_config_t *config);
static bool PHAL_configureHSESystemClock(const rcc_config_t *config);
static bool PHAL_configureAPBClocks(const rcc_config_t *config);

bool PHAL_configureClockRates(const rcc_config_t *config)
{
	if (config->target_hz != RCC_144_MHZ || (config->use_hse && !((config->input_hz == RCC_16_MHZ) || (config->input_hz == RCC_8_MHZ)))) // lol
	{
		return false;
	}

	if (!config->use_hse && (config->input_hz != RCC_16_MHZ)) {
		return false; // HSI16
	}

	if (config->use_hse) {
		if (!PHAL_configureHSESystemClock(config)) {
			return false;
		}
	} else {
		if (!PHAL_configureHSISystemClock(config)) {
			return false;
		}
	}

	if (!PHAL_configurePLLRates(config)) {
		return false;
	}
	if (!PHAL_configurePLLSystemClock(config)) {
		return false;
	}

	if (!PHAL_configureAPBClocks(config)) {
		return false;
	}

	// if (SysTick_Config(SystemCoreClock / 1000)) {
	// 	return false;
	// }
	// NVIC_SetPriority(SysTick_IRQn, 0x0F); // TODO move to common

	return true;
}

static bool PHAL_configurePLLRates(const rcc_config_t *config)
{
	// Turn off and wait for PLL to disable
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY))
		; // Wait for PLL to turn off
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLM_Msk);

	if (config->use_hse) {
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
		while (!(RCC->CR & RCC_CR_HSERDY))
			;
	} else {
		RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // Select HSI source
		while (!(RCC->CR & RCC_CR_HSIRDY))		// Wait for HSI to enable
			;
	}

	// 16 Mhz -> 144 MHz
	uint32_t pllm = 1;	// PLLM
	uint32_t plln = 18; // PLLN
	uint32_t pllq = 6;	// PLLQ
	uint32_t pllr = 2;	// PLLR

	switch (config->input_hz) {
	case RCC_8_MHZ:
		plln = 36; // 2x
		break;
	case RCC_16_MHZ:
		plln = 18;
		break;
	default:
		return false;
	}

	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk);
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLQEN | RCC_PLLCFGR_PLLREN | RCC_PLLCFGR_PLLPEN);
	RCC->PLLCFGR |= ((pllm) << RCC_PLLCFGR_PLLM_Pos) & RCC_PLLCFGR_PLLM_Msk; // Set PLLM
	RCC->PLLCFGR |= ((plln) << RCC_PLLCFGR_PLLN_Pos) & RCC_PLLCFGR_PLLN_Msk; // Set PLLN
	RCC->PLLCFGR |= ((pllq) << RCC_PLLCFGR_PLLQ_Pos) & RCC_PLLCFGR_PLLQ_Msk; // Set PLLQ
	RCC->PLLCFGR |= ((pllr) << RCC_PLLCFGR_PLLR_Pos) & RCC_PLLCFGR_PLLR_Msk; // Set PLLR
	__DSB();

	return true;
}

static bool PHAL_configurePLLSystemClock(const rcc_config_t *config)
{
	RCC->CR |= RCC_CR_PLLON; // Enable PLL
	while (!(RCC->CR & RCC_CR_PLLRDY))
		; // Wait for PLL to turn on
	__DSB();

	// Flash latency adjustment, see ST RM 0090 Pg. 80, ST RM 0431 Pg. 69
	uint32_t flash_acr_temp = FLASH->ACR;
	flash_acr_temp &= ~(FLASH_ACR_LATENCY_Msk);

	uint32_t system_clock_target_hz = config->target_hz;
	if (system_clock_target_hz >= 210000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_7WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 180000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_6WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 150000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_5WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 120000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_4WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 90000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_3WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 60000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_2WS << FLASH_ACR_LATENCY_Pos;
	else if (system_clock_target_hz >= 30000000)
		flash_acr_temp |= FLASH_ACR_LATENCY_1WS << FLASH_ACR_LATENCY_Pos;
	else
		flash_acr_temp |= FLASH_ACR_LATENCY_0WS << FLASH_ACR_LATENCY_Pos;
	FLASH->ACR = flash_acr_temp;

	__DSB(); // Wait for explicit memory accesses to finish
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL)
		;
	__DSB(); // Wait for explicit memory accesses to finish

	SystemCoreClockUpdate(); // Must be called each time the core clock HCLK
							 // changes
	return true;
}

static bool PHAL_configureHSISystemClock(const rcc_config_t *config)
{
	// Turn on and wait for HSI to enable
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY))
		;

	// Flash latency adjustment, see ST RM 0090 Pg. 80
	uint32_t flash_acr_temp = FLASH->ACR;
	flash_acr_temp &= ~(FLASH_ACR_LATENCY_Msk);
	flash_acr_temp |= FLASH_ACR_LATENCY_0WS << FLASH_ACR_LATENCY_Pos;
	FLASH->ACR = flash_acr_temp;

	__DSB();
	RCC->CFGR |= RCC_CFGR_SW_HSI; // Set system clock switch register to HSI
	while ((RCC->CFGR & RCC_CFGR_SWS_HSI) != RCC_CFGR_SWS_HSI)
		;
	__DSB();

	SystemCoreClockUpdate(); // Must be called each time the core clock HCLK
							 // changes
	return true;			 // Return true upon completion
}

static bool PHAL_configureHSESystemClock(const rcc_config_t *config)
{
	// 1. Enable PWR clock if needed
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	// 2. Set Voltage Scaling if using high-speed clocks (e.g., for 170 MHz)
	PWR->CR1 |= PWR_CR1_VOS_1; // VOS = 1
	while ((PWR->SR2 & PWR_SR2_VOSF))
		; // Wait until voltage scaling is ready

	/* Turn on and wait for HSE to enable */
	if (config->hse_crystal) {
		RCC->CR &= ~RCC_CR_HSEBYP; // Clear bypass bit
		RCC->CR |= (RCC_CR_HSEON); // no HSEBYP
	} else {
		RCC->CR |= (RCC_CR_HSEON | RCC_CR_HSEBYP);
	}
	while (!(RCC->CR & RCC_CR_HSERDY))
		;
	__DSB();

	// Flash latency adjustment, see ST RM 0090 Pg. 80
	uint32_t flash_acr_temp = FLASH->ACR;
	flash_acr_temp &= ~(FLASH_ACR_LATENCY_Msk);
	flash_acr_temp |= FLASH_ACR_LATENCY_0WS << FLASH_ACR_LATENCY_Pos;
	FLASH->ACR = flash_acr_temp;

	__DSB();
	// Clear and set SW bits together:
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSE;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) { /* wait */
	}
	__DSB(); // Wait for explicit memory accesses to finish

	/* Turn off HSI to save power */
	__DSB();
	RCC->CR &= ~(RCC_CR_HSION);
	while ((RCC->CR & RCC_CR_HSION))
		;
	__DSB();

	SystemCoreClockUpdate();
	return true;
}

static bool PHAL_configureAPBClocks(const rcc_config_t *config)
{
	// Set AHB, APB1, and APB2 prescalers, defualt to 1
	RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB1 prescaler = 1
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB2 prescaler = 1
	return true;
}
