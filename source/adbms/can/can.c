
#include "common/phal_F4_F7/can/can.h"
#include "common/freertos/freertos.h"
#include "can.h"
#include "can_parse.h"

#define CAN_TX_BLOCK_TIMEOUT (30 * 16000) // clock rate 16MHz, 15ms * 16000 cyc / ms

can_stats_t can_stats[NUM_CAN_PERIPHERALS] = {0};

#ifdef CAN_PARSE_USE_PQUEUE
#else
defineStaticQueue(q_can1_tx, CanMsgTypeDef_t, 2048); // CAN messages RX'd to DAQ
defineStaticQueue(q_can2_tx, CanMsgTypeDef_t, 2048); // CAN messages RX'd to DAQ
QueueHandle_t *q_can_tx[NUM_CAN_PERIPHERALS] = {&q_can1_tx, &q_can2_tx};
#endif

void canTxSendToBack(CanMsgTypeDef_t *tx_msg)
{
	uint8_t peripheral_idx = (tx_msg->Bus == CAN1) ? CAN1_IDX : CAN2_IDX;
#ifdef CAN_PARSE_USE_PQUEUE
	if (qSendToBack(q_can_tx[peripheral_idx], tx_msg) != SUCCESS_G) {
		can_stats.can_peripheral_stats[peripheral_idx].tx_of++;
	}
#else
	if (xQueueSendToBack(*q_can_tx[peripheral_idx], tx_msg, (TickType_t)10) != pdPASS) {
		can_stats[peripheral_idx].tx_of++;
	}
#endif
}

static void can_tx_send(CanMsgTypeDef_t *tx_msg)
{
	uint8_t peripheral_idx = (tx_msg->Bus == CAN1) ? CAN1_IDX : CAN2_IDX;
	uint32_t t = 0;
	/* Don't use multiple mailboxes to guarantee in-order transmission */
	while (!PHAL_txMailboxFree(tx_msg->Bus, 0) && (t++ < CAN_TX_BLOCK_TIMEOUT))
		;
	if (t < CAN_TX_BLOCK_TIMEOUT)
		PHAL_txCANMessage(tx_msg, 0);
	else
		can_stats[peripheral_idx].tx_fail++;
}

void canTxUpdate(void)
{
	CanMsgTypeDef_t tx_msg;
	for (uint8_t peripheral_idx = 0; peripheral_idx < NUM_CAN_PERIPHERALS; peripheral_idx++) {
		while (xQueueReceive(*q_can_tx[peripheral_idx], &tx_msg, portMAX_DELAY) == pdPASS) {
			can_tx_send(&tx_msg);
		}
	}
}
