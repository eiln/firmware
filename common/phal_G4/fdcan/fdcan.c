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

#define FDCAN_MSG_RAM_START  SRAMCAN_BASE
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
	Instance->TOCC = 0x00000000;

	Instance->NBTP = ((16 - 1) << FDCAN_NBTP_NSJW_Pos) |    // SJW = 16
	((140 - 1) << FDCAN_NBTP_NTSEG1_Pos) | // TSEG1 = 140
	((51  - 1) << FDCAN_NBTP_NTSEG2_Pos) | // TSEG2 = 51
	((1   - 1) << FDCAN_NBTP_NBRP_Pos);    // Prescaler = 1
	//
	Instance->TXBC = (2 << 0)  // TFQS = 2 means 3 elements (0-based count)
                 | (1 << 7);  // TFQM = 1: FIFO mode
	/* Calculate each RAM block address */
	// FDCAN_CalcultateRamBlockAddresses(hfdcan);
	Instance->RXGFC = (0 << FDCAN_RXGFC_LSS_Pos) |	  // 0 standard filters
					  (0 << FDCAN_RXGFC_LSE_Pos) |	  // 0 extended filters
					  FDCAN_RXGFC_ANFS_ACCEPT_FIFO0 | // Accept all non-matching STD IDs into FIFO0
					  FDCAN_RXGFC_ANFE_ACCEPT_FIFO0;  // Accept all non-matching EXT IDs into FIFO0

// Enable interrupt lines and interrupts *before* clearing CCE
Instance->ILE |= FDCAN_ILE_EINT0;
Instance->IE  |= FDCAN_IE_RF0NE | FDCAN_IE_RF1NE;

// Leave init mode
Instance->CCCR &= ~FDCAN_CCCR_INIT;
while (Instance->CCCR & FDCAN_CCCR_INIT);

// Disable config changes (clear CCE)
Instance->CCCR &= ~FDCAN_CCCR_CCE;

	NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	NVIC_SetPriority(FDCAN1_IT0_IRQn, 7);

	return true;
}

void FDCAN1_IT0_IRQHandler(void) {
    uint32_t ir = FDCAN1->IR;  // Read interrupt register

    if (ir & FDCAN_IR_RF0N) {
        // Handle RX FIFO0 new message interrupt
        FDCAN1->IR = FDCAN_IR_RF0N; // Clear the interrupt flag
		asm("bkpt");
        // Read message from RX FIFO0 here
    }
	if (ir & FDCAN_IR_RF1N) {
        FDCAN1->IR = FDCAN_IR_RF1N;
		asm("bkpt");
    }

    // Handle other interrupts as needed
	asm("bkpt");
}

void fdcan_test_send(FDCAN_GlobalTypeDef *Instance) {
    // Wait until there is space in the Tx FIFO/Queue (TFQF = FIFO full flag, bit 24)
    while (Instance->TXFQS & FDCAN_TXFQS_TFQF) {
        // Wait while FIFO is full
    }

    // Get the next free buffer index to write (TFQPI bits [13:8])
    uint32_t buf_idx = (Instance->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    // Setup standard ID (11-bit) and 8-byte data payload
    uint32_t id = 0x123;
    uint8_t data[8] = { 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H' };

    // Pointer to the Tx buffer in message RAM
    volatile uint32_t *tx_buf = (volatile uint32_t *)(FDCAN_MSG_RAM_START + 0x100 * buf_idx);

    // T0 register: bits
    // Bits 28:18 = Standard ID (11 bits)
    // Bit 30 = IDE (0 for standard)
    // Bit 31 = RTR (0 for data frame)
    tx_buf[0] = (id << 18) & 0x1FFC0000;

    // T1 register:
    // Bits 15:12 = DLC (Data length code, 8 bytes = 8)
    // Bit 10 = ESI (Error State Indicator, 0)
    // Bits 9:8 = RTR, XTD (0 for data frame, standard ID)
    tx_buf[1] = (8 << 16);  // DLC in bits 19:16 (not 15:12) on STM32, check datasheet

    // Copy 8 bytes data into Tx buffer payload (4 bytes each uint32_t)
    tx_buf[2] = ((uint32_t*)data)[0];
    tx_buf[3] = ((uint32_t*)data)[1];

    // Request transmission by setting the request bit for this buffer index
    Instance->TXBAR = (1 << buf_idx);

    // Optionally wait until transmission is complete for this buffer
    while (Instance->TXBRP & (1 << buf_idx)) {
        // wait for buffer to be transmitted
    }
}
