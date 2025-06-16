/**
 * @file can.c
 * @author Eileen Yoon - Port of L4 HAL by Adam Busch (busch8@purdue.edu)
 * @brief Basic CAN Peripheral HAL library for setting up CAN peripheral and
 * sending messages
 * @version 0.1
 * @date 2023-09-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "common/phal_G4/fdcan/fdcan.h"
#include "common/phal_G4/rcc/rcc.h"

#define FDCAN_TX_FIFO_OPERATION ((uint32_t)0x00000000U)		 /*!< FIFO mode  */
#define FDCAN_TX_QUEUE_OPERATION ((uint32_t)FDCAN_TXBC_TFQM) /*!< Queue mode */

#define FDCAN_RXGFC_ANFS_ACCEPT_FIFO0 (0x0U << FDCAN_RXGFC_ANFS_Pos)
#define FDCAN_RXGFC_ANFE_ACCEPT_FIFO0 (0x0U << FDCAN_RXGFC_ANFE_Pos)

bool phal_fdcan_init(FDCAN_GlobalTypeDef *Instance, uint32_t bitrate)
{
	// Enable FDCAN clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

	// Select 48 MHz for FDCAN kernel clock (e.g., from PLLQ)
	RCC->CCIPR &= ~RCC_CCIPR_FDCANSEL; // Clear bits
	RCC->CCIPR |= RCC_CCIPR_FDCANSEL_0;

	// Exit sleep mode
	Instance->CCCR &= ~FDCAN_CCCR_CSR;
	while ((Instance->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA) {
		;
	}

	// Enter INIT mode
	Instance->CCCR |= FDCAN_CCCR_INIT;
	while ((Instance->CCCR & FDCAN_CCCR_INIT) == 0) {
		;
	}

	// Enable configuration changes
	Instance->CCCR |= FDCAN_CCCR_CCE;
	/* Check FDCAN instance */
	if (Instance == FDCAN1) {
		/* Configure Clock divider */
		FDCAN_CONFIG->CKDIV = 1;
	}

	// Disable FD and BRS for classic CAN
	Instance->CCCR &= ~(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE);
	Instance->CCCR |= FDCAN_CCCR_NISO;

	Instance->CCCR &= ~(FDCAN_CCCR_DAR);  // clear no automatic retransmission
	Instance->CCCR &= ~(FDCAN_CCCR_TXP);  // clear transmit pause
	Instance->CCCR &= ~(FDCAN_CCCR_PXHD); // clear protocol exception

	// clear existing modes
	Instance->CCCR &= ~(FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);
	Instance->TEST &= ~(FDCAN_TEST_LBCK);

	// Setup bit timing (example for 1 Mbps, adjust for your clock)
	uint32_t tseg1 = 1;
	uint32_t tseg2 = 1;
	uint32_t sjw = 1;
	uint32_t prescaler = 64;

	Instance->NBTP = ((sjw - 1) << FDCAN_NBTP_NSJW_Pos) | ((tseg1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((tseg2 - 1) << FDCAN_NBTP_NTSEG2_Pos)
					 | ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos);

	/* Select between Tx FIFO and Tx Queue operation modes */
	Instance->TXBC |= FDCAN_TX_QUEUE_OPERATION;
	/* Calculate each RAM block address */
	// FDCAN_CalcultateRamBlockAddresses(hfdcan);
	Instance->RXGFC = (0 << FDCAN_RXGFC_LSS_Pos) |	  // 0 standard filters
					  (0 << FDCAN_RXGFC_LSE_Pos) |	  // 0 extended filters
					  FDCAN_RXGFC_ANFS_ACCEPT_FIFO0 | // Accept all non-matching STD IDs into FIFO0
					  FDCAN_RXGFC_ANFE_ACCEPT_FIFO0;  // Accept all non-matching EXT IDs into FIFO0

	// Leave INIT mode - keep CCE set during transition
	Instance->CCCR &= ~FDCAN_CCCR_INIT;
	while (Instance->CCCR & FDCAN_CCCR_INIT) {
		;
	}

	// Disable config changes (clear CCE)
	Instance->CCCR &= ~FDCAN_CCCR_CCE;

	return true;
}

#if 0
static void FDCAN_CalcultateRamBlockAddresses(FDCAN_HandleTypeDef *hfdcan)
{
	uint32_t RAMcounter;
	uint32_t SramCanInstanceBase = SRAMCAN_BASE;
#if defined(FDCAN2)

	if (hfdcan->Instance == FDCAN2)
	{
		SramCanInstanceBase += SRAMCAN_SIZE;
	}
#endif /* FDCAN2 */
#if defined(FDCAN3)
	if (hfdcan->Instance == FDCAN3)
	{
		SramCanInstanceBase += SRAMCAN_SIZE * 2U;
	}
#endif /* FDCAN3 */

	/* Standard filter list start address */
	hfdcan->msgRam.StandardFilterSA = SramCanInstanceBase + SRAMCAN_FLSSA;

	/* Standard filter elements number */
	MODIFY_REG(hfdcan->Instance->RXGFC, FDCAN_RXGFC_LSS, (hfdcan->Init.StdFiltersNbr << FDCAN_RXGFC_LSS_Pos));

	/* Extended filter list start address */
	hfdcan->msgRam.ExtendedFilterSA = SramCanInstanceBase + SRAMCAN_FLESA;

	/* Extended filter elements number */
	MODIFY_REG(hfdcan->Instance->RXGFC, FDCAN_RXGFC_LSE, (hfdcan->Init.ExtFiltersNbr << FDCAN_RXGFC_LSE_Pos));

	/* Rx FIFO 0 start address */
	hfdcan->msgRam.RxFIFO0SA = SramCanInstanceBase + SRAMCAN_RF0SA;

	/* Rx FIFO 1 start address */
	hfdcan->msgRam.RxFIFO1SA = SramCanInstanceBase + SRAMCAN_RF1SA;

	/* Tx event FIFO start address */
	hfdcan->msgRam.TxEventFIFOSA = SramCanInstanceBase + SRAMCAN_TEFSA;

	/* Tx FIFO/queue start address */
	hfdcan->msgRam.TxFIFOQSA = SramCanInstanceBase + SRAMCAN_TFQSA;

	/* Flush the allocated Message RAM area */
	for (RAMcounter = SramCanInstanceBase; RAMcounter < (SramCanInstanceBase + SRAMCAN_SIZE); RAMcounter += 4U)
	{
		*(uint32_t *)(RAMcounter) = 0x00000000U;
	}
}
#endif

#if 0
extern uint32_t APB1ClockRateHz;

bool phal_fdcan_init(MY_FDCAN_HandleTypeDef *hfdcan)
{
		uint32_t tickstart;

		RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;

		// Exit sleep mode: clear CSR bit
		hfdcan->Instance->CCCR &= ~FDCAN_CCCR_CSR;

		tickstart = MY_HAL_GetTick();
		while ((hfdcan->Instance->CCCR & FDCAN_CCCR_CSA) != 0)
		{
				if ((MY_HAL_GetTick() - tickstart) > FDCAN_TIMEOUT_VALUE)
				{
						return false;
				}
		}

		// Request initialization: set INIT bit
		hfdcan->Instance->CCCR |= FDCAN_CCCR_INIT;

		tickstart = MY_HAL_GetTick();
		while ((hfdcan->Instance->CCCR & FDCAN_CCCR_INIT) == 0)
		{
				if ((MY_HAL_GetTick() - tickstart) > FDCAN_TIMEOUT_VALUE)
				{
						return false;
				}
		}

		// Enable configuration changes: set CCE bit
		hfdcan->Instance->CCCR |= FDCAN_CCCR_CCE;

		// Clock divider config (only FDCAN1)
		if (hfdcan->Instance == FDCAN1)
		{
				FDCAN_CONFIG->CKDIV = hfdcan->Init.ClockDivider;
		}

		// Auto retransmission: clear or set DAR bit
		if (hfdcan->Init.AutoRetransmission == ENABLE)
		{
				hfdcan->Instance->CCCR &= ~FDCAN_CCCR_DAR;
		}
		else
		{
				hfdcan->Instance->CCCR |= FDCAN_CCCR_DAR;
		}

		// Transmit pause: set or clear TXP bit
		if (hfdcan->Init.TransmitPause == ENABLE)
		{
				hfdcan->Instance->CCCR |= FDCAN_CCCR_TXP;
		}
		else
		{
				hfdcan->Instance->CCCR &= ~FDCAN_CCCR_TXP;
		}

		// Protocol exception: clear or set PXHD bit
		if (hfdcan->Init.ProtocolException == ENABLE)
		{
				hfdcan->Instance->CCCR &= ~FDCAN_CCCR_PXHD;
		}
		else
		{
				hfdcan->Instance->CCCR |= FDCAN_CCCR_PXHD;
		}

		// Set Frame Format bits: clear relevant bits then set
		hfdcan->Instance->CCCR &= ~FDCAN_CCCR_FDOE; // Assuming FDOE mask here (replace with actual)
		hfdcan->Instance->CCCR |= hfdcan->Init.FrameFormat;

		// Clear test, monitor and ASM bits
		hfdcan->Instance->CCCR &= ~(FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);

		// Clear loopback test bit in TEST register
		hfdcan->Instance->TEST &= ~FDCAN_TEST_LBCK;

		// Set Nominal Bit Timing Register (NBTP)
		hfdcan->Instance->NBTP =
				((hfdcan->Init.NominalSyncJumpWidth - 1U) << FDCAN_NBTP_NSJW_Pos) |
				((hfdcan->Init.NominalTimeSeg1 - 1U) << FDCAN_NBTP_NTSEG1_Pos) |
				((hfdcan->Init.NominalTimeSeg2 - 1U) << FDCAN_NBTP_NTSEG2_Pos) |
				((hfdcan->Init.NominalPrescaler - 1U) << FDCAN_NBTP_NBRP_Pos);

		// Set Tx FIFO/Queue Mode bits in TXBC register
		hfdcan->Instance->TXBC |= hfdcan->Init.TxFifoQueueMode;

		// Calculate RAM block addresses (user provided function)
		FDCAN_CalculateRamBlockAddresses(hfdcan);

		hfdcan->LatestTxFifoQRequest = 0;

		return true;
}
#endif
