/**
 * @file main.c
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief CAN Bootloader:
 *        - Double bank flash buffer + CRC
 *        - Download/Upload firmware over buffered CAN TP
 *        - Load/store backup firmware
 * @version 0.1
 * @date 2024-11-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#if defined(STM32L496xx) || defined(STM32L432xx)
#include "common/phal_L4/can/can.h"
#include "common/phal_L4/gpio/gpio.h"
#include "common/phal_L4/rcc/rcc.h"
#endif
#if defined(STM32F407xx) || defined(STM32F732xx)
#include "common/phal_F4_F7/can/can.h"
#include "common/phal_F4_F7/gpio/gpio.h"
#include "common/phal_F4_F7/rcc/rcc.h"
#endif

/* Module Includes */
#include "can_parse.h"
#include "node_defs.h"
#include "bootloader.h"


/* PER HAL Initilization Structures */
GPIOInitConfig_t gpio_config[] = {
    CAN_RX_GPIO_CONFIG,
    CAN_TX_GPIO_CONFIG,
};

extern uint32_t APB1ClockRateHz;
extern uint32_t APB2ClockRateHz;
extern uint32_t AHBClockRateHz;
extern uint32_t PLLClockRateHz;

#define TargetCoreClockrateHz 16000000
ClockRateConfig_t clock_config = {
    .system_source              =SYSTEM_CLOCK_SRC_HSI,
    .system_clock_target_hz     =TargetCoreClockrateHz,
    .ahb_clock_target_hz        =(TargetCoreClockrateHz / 1),
    .apb1_clock_target_hz       =(TargetCoreClockrateHz / (1)),
    .apb2_clock_target_hz       =(TargetCoreClockrateHz / (1)),
};

void HardFault_Handler();
void canTxSendToBack(CanMsgTypeDef_t *msg);
static bool BL_InProgress(void);
static void BL_CANPoll(void);

q_handle_t q_tx_can;
q_handle_t q_rx_can;

#define BL_BACKDOOR_PERIOD 3000   // Allow 3s of bootloader mode at the start
#define CAN_TX_BLOCK_TIMEOUT (30 * 16000) // Clock rate 16MHz, 15ms * 16000 cyc / ms
static volatile uint32_t bootloader_ms;   // Systick

int main(void)
{
    /* Data Struct init */
    qConstruct(&q_tx_can, sizeof(CanMsgTypeDef_t));
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));

#ifdef HSI_TRIM_BL_NODE
    PHAL_trimHSI(HSI_TRIM_BL_NODE);
#endif
    if (0 != PHAL_configureClockRates(&clock_config))
        HardFault_Handler();

    if (1 != PHAL_initGPIO(gpio_config, sizeof(gpio_config)/sizeof(GPIOInitConfig_t)))
        HardFault_Handler();

    // Init bare minimum peripherals (systick, can1, crc)
    SysTick_Config(SystemCoreClock / 1000);
    NVIC_EnableIRQ(SysTick_IRQn);

    if (1 != PHAL_initCAN(CAN1, false, VCAN_BPS))
        HardFault_Handler();

    initCANParse(&q_rx_can);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);

    // Boot immediately if verified firmware is found
    //BL_checkAndBoot(false); // Just kidding, disabling backdoor bypass for now

    // If verified firmware not found, signal that we're in bootloader mode
    BL_sendSuccess(0, BL_MAGIC_BOOTLOADER);
    // Then enter backdoor period (CAN loop) for 3s
    uint32_t start_ms = bootloader_ms;
    while (bootloader_ms - start_ms < BL_BACKDOOR_PERIOD || BL_InProgress())
    {
        BL_CANPoll();
    }

    // Now try booting unverified firmware
    BL_checkAndBoot(false);

    while (1) // Infinite bootloader poll loop
    {
        BL_CANPoll();
    }
}

static bool BL_InProgress(void)
{
    return BL_flashStarted() || !qIsEmpty(&q_rx_can) || !qIsEmpty(&q_tx_can);
}

static void BL_CANPoll(void)
{
    while (!qIsEmpty(&q_rx_can))
        canRxUpdate();
}

// Override CAN TX method to block (instead of queueing and distributing across multiple mailboxes) to guarantee on-time and in-order transmission of frames
// Bootloader messages have highest priority so this should not fail
void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    uint32_t t = 0;
    while (!PHAL_txMailboxFree(CAN1, 0) && (t++ < CAN_TX_BLOCK_TIMEOUT));
    if (t < CAN_TX_BLOCK_TIMEOUT) PHAL_txCANMessage(msg, 0);
    // TODO: count errors?
}

void CAN1_RX0_IRQHandler()
{
    if (CAN1->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
        CAN1->RF0R &= ~(CAN_RF0R_FOVR0);

    if (CAN1->RF0R & CAN_RF0R_FULL0) // FIFO Full
        CAN1->RF0R &= ~(CAN_RF0R_FULL0);

    if (CAN1->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;
        rx.Bus = CAN1;

        // Get either StdId or ExtId
        if (CAN_RI0R_IDE & CAN1->sFIFOMailBox[0].RIR)
        {
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & CAN1->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & CAN1->sFIFOMailBox[0].RIR) >> CAN_TI0R_STID_Pos;
        }

        rx.DLC = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 0) & 0xFF;
        rx.Data[1] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 8) & 0xFF;
        rx.Data[2] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (CAN1->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 0) & 0xFF;
        rx.Data[5] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 8) & 0xFF;
        rx.Data[6] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (CAN1->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        CAN1->RF0R     |= (CAN_RF0R_RFOM0);

        qSendToBack(&q_rx_can, &rx); // Add to queue (qSendToBack is interrupt safe)
    }
}

void SysTick_Handler(void)
{
    bootloader_ms++;
}

void HardFault_Handler()
{
    NVIC_SystemReset();
    while(1)
        ;
}
