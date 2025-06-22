#include "common/daq/can_parse_base.h"
/**
 * q_tx_can_0 -> hlp [0,1] -> mailbox 1
 * q_tx_can_1 -> hlp [2,3] -> mailbox 2
 * q_tx_can_2 -> hlp [4,5] -> mailbox 3
*/

can_stats_t can_stats = {0};
uint32_t can_mbx_last_send_time[NUM_CAN_PERIPHERALS][CAN_TX_MAILBOX_CNT] = {0};
volatile uint32_t last_can_rx_time_ms = 0;

#ifndef AUTOCAN_USE_FREERTOS
q_handle_t q_tx_can[NUM_CAN_PERIPHERALS][CAN_TX_MAILBOX_CNT];
q_handle_t q_rx_can;
#else
defineStaticQueue(q_rx_can, CanMsgTypeDef_t, 2048); // CAN messages RX'd to DAQ
defineStaticQueue(q_can1_tx, CanMsgTypeDef_t, 2048); // CAN messages RX'd to DAQ
defineStaticQueue(q_can2_tx, CanMsgTypeDef_t, 2048); // CAN messages RX'd to DAQ
QueueHandle_t *q_can_tx[NUM_CAN_PERIPHERALS] = {&q_can1_tx, &q_can2_tx};
#endif

void initCANParseBase(void)
{
    #ifndef AUTOCAN_USE_FREERTOS
    for (uint8_t can_periph = 0; can_periph < NUM_CAN_PERIPHERALS; can_periph++)
    {
      for (uint8_t mbx = 0; mbx < CAN_TX_MAILBOX_CNT; mbx++)
      {
        qConstruct(&q_tx_can[can_periph][mbx], sizeof(CanMsgTypeDef_t));
      }
    }
    qConstruct(&q_rx_can, sizeof(CanMsgTypeDef_t));
    #else
    q_rx_can = createStaticQueue(q_rx_can, CanMsgTypeDef_t, 2048);
    q_can1_tx = createStaticQueue(q_can1_tx, CanMsgTypeDef_t, 2048);
    q_can2_tx = createStaticQueue(q_can2_tx, CanMsgTypeDef_t, 2048);
    #endif
}

void canTxSendToBack(CanMsgTypeDef_t *msg)
{
    q_handle_t *qh;
    uint8_t mailbox;
    uint8_t peripheral_idx = (msg->Bus == CAN1) ? CAN1_IDX : CAN2_IDX;
    if (msg->IDE == 1)
    {
        // extended id, check hlp
        switch((msg->ExtId >> 26) & 0b111)
        {
            case 0:
            case 1:
                mailbox = CAN_MAILBOX_HIGH_PRIO;
                break;
            case 2:
            case 3:
                mailbox = CAN_MAILBOX_MED_PRIO;
                break;
            default:
                mailbox = CAN_MAILBOX_LOW_PRIO;
                break;
        }
        qh = &q_tx_can[peripheral_idx][mailbox];
    }
    else
    {
        qh = &q_tx_can[peripheral_idx][CAN_MAILBOX_HIGH_PRIO]; // IDE = 0 doesn't have an HLP
    }
    if (qSendToBack(qh, msg) != SUCCESS_G)
    {
        can_stats.can_peripheral_stats[peripheral_idx].tx_of++;
    }
}

void __attribute__((weak)) canTxUpdate(void)
{
    CanMsgTypeDef_t tx_msg;
    for (uint8_t i = 0; i < CAN_TX_MAILBOX_CNT; ++i)
    {
        // Handle CAN1
        if (PHAL_txMailboxFree(CAN1, i))
        {
            if (qReceive(&q_tx_can[CAN1_IDX][i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can_mbx_last_send_time[CAN1_IDX][i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can_mbx_last_send_time[CAN1_IDX][i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN1, i); // aborts tx and empties the mailbox
            can_stats.can_peripheral_stats[CAN1_IDX].tx_fail++;
        }
#ifdef CAN2
        // Handle CAN2
        if(PHAL_txMailboxFree(CAN2, i))
        {
            if (qReceive(&q_tx_can[CAN2_IDX][i], &tx_msg) == SUCCESS_G)    // Check queue for items and take if there is one
            {
                PHAL_txCANMessage(&tx_msg, i);
                can_mbx_last_send_time[CAN2_IDX][i] = sched.os_ticks;
            }
        }
        else if (sched.os_ticks - can_mbx_last_send_time[CAN2_IDX][i] > CAN_TX_TIMEOUT_MS)
        {
            PHAL_txCANAbort(CAN2, i); // aborts tx and empties the mailbox
            can_stats.can_peripheral_stats[CAN2_IDX].tx_fail++;
        }
#endif
    }
}

void canParseIRQHandler(CAN_TypeDef *can_h)
{
    #ifdef AUTOCAN_USE_FREERTOS
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    #endif

    can_peripheral_stats_t *rx_stats = (can_h == CAN1) ? (&can_stats.can_peripheral_stats[CAN1_IDX]) : (&can_stats.can_peripheral_stats[CAN2_IDX]);
    if (can_h->RF0R & CAN_RF0R_FOVR0) // FIFO Overrun
    {
        can_h->RF0R |= CAN_RF0R_FOVR0;
        rx_stats->rx_overrun++;
    }

    if (can_h->RF0R & CAN_RF0R_FULL0) // FIFO Full
        can_h->RF0R |= CAN_RF0R_FULL0;

    if (can_h->RF0R & CAN_RF0R_FMP0_Msk) // Release message pending
    {
        CanMsgTypeDef_t rx;
        rx.Bus = can_h;

        // Get either StdId or ExtId
        rx.IDE = CAN_RI0R_IDE & can_h->sFIFOMailBox[0].RIR;
        if (rx.IDE)
        {
          rx.ExtId = ((CAN_RI0R_EXID | CAN_RI0R_STID) & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_EXID_Pos;
        }
        else
        {
          rx.StdId = (CAN_RI0R_STID & can_h->sFIFOMailBox[0].RIR) >> CAN_RI0R_STID_Pos;
          rx.ExtId = rx.StdId; // for can_parse (assumes all are ExtId)
        }

        rx.DLC = (CAN_RDT0R_DLC & can_h->sFIFOMailBox[0].RDTR) >> CAN_RDT0R_DLC_Pos;

        rx.Data[0] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 0)  & 0xFF;
        rx.Data[1] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 8)  & 0xFF;
        rx.Data[2] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        rx.Data[3] = (uint8_t) (can_h->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        rx.Data[4] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 0)  & 0xFF;
        rx.Data[5] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 8)  & 0xFF;
        rx.Data[6] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        rx.Data[7] = (uint8_t) (can_h->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

        can_h->RF0R |= (CAN_RF0R_RFOM0);

        #ifndef AUTOCAN_USE_FREERTOS
        if (qSendToBack(&q_rx_can, &rx) != SUCCESS_G) {
            can_stats.rx_of++;
        }
        #else
        if (xQueueSendToBack(q_rx_can, &rx, (TickType_t)10) != pdPASS) {
            can_stats.rx_of++;
        }
        #endif
    }

    #ifdef AUTOCAN_USE_FREERTOS
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    #endif
}

void canRxUpdate(void)
{
    CanMsgTypeDef_t rx_msg;
    #ifndef AUTOCAN_USE_FREERTOS
    while (qReceive(&q_rx_can, &rx_msg) == SUCCESS_G)
    #else
    while (xQueueReceive(q_can_rx, &rx_msg, portMAX_DELAY) == pdPASS)
    #endif
    {
        #ifndef AUTOCAN_USE_FREERTOS
        last_can_rx_time_ms = sched.os_ticks;
        #else
        last_can_rx_time_ms = getTick();
    #endif
        /* BEGIN AUTO CASES */
        handle_rx_autocase(&rx_msg);
        /* END AUTO CASES */
    }

    /* BEGIN AUTO STALE CHECKS */
    handle_rx_stale();
    /* END AUTO STALE CHECKS */
}

static bool initCANFilter()
{
    uint32_t timeout = 0;
    CAN1->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    while(!(CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout >= PHAL_CAN_INIT_TIMEOUT)
         return false;
    CAN1->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN1->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN1->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale

#ifdef CAN2
    CAN2->MCR |= CAN_MCR_INRQ;                // Enter back into INIT state (required for changing scale)
    while(!(CAN2->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
         ;
    if (timeout == PHAL_CAN_INIT_TIMEOUT)
         return false;
    CAN2->FMR  |= CAN_FMR_FINIT;              // Enter init mode for filter banks
    CAN2->FM1R |= 0x07FFFFFF;                 // Set banks 0-27 to id mode
    CAN2->FS1R |= 0x07FFFFFF;                 // Set banks 0-27 to 32-bit scale
#endif /* CAN2 */

    /* BEGIN AUTO FILTER */
    set_rx_filter();
    /* END AUTO FILTER */

    CAN1->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)
    // Enter back into NORMAL mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while((CAN1->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
        ;
    if (timeout >= PHAL_CAN_INIT_TIMEOUT)
        return false;

#ifdef CAN2
    CAN2->FMR  &= ~CAN_FMR_FINIT;             // Enable Filters (exit filter init mode)
    // Enter back into NORMAL mode
    CAN2->MCR &= ~CAN_MCR_INRQ;
    while((CAN2->MSR & CAN_MSR_INAK) && ++timeout < PHAL_CAN_INIT_TIMEOUT)
        ;
    if (timeout >= PHAL_CAN_INIT_TIMEOUT)
         return false;
#endif /* CAN2 */

    return true;
}

void initCANParse(void)
{
    initCANParseBase();
    initCANFilter();
}
