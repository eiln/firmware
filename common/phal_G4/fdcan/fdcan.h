/**
 * @file can.h
 * @author Adam Busch (busch8@purdue.edu)
 * @brief
 * @version 0.1
 * @date 2021-03-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef PHAL_FDCAN_H_
#define PHAL_FDCAN_H_

#include "common/phal_G4/phal_g4.h"

bool phal_fdcan_init(FDCAN_GlobalTypeDef *Instance, uint32_t bitrate);
void fdcan_test_send(FDCAN_GlobalTypeDef *Instance);

#define GPIO_AF9_FDCAN1 (9)

#define GPIO_INIT_FDCAN1_RX_PB8 GPIO_INIT_AF(GPIOB, 8, GPIO_AF9_FDCAN1, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_OPEN_DRAIN, GPIO_INPUT_PULL_UP)
#define GPIO_INIT_FDCAN1_TX_PB9 GPIO_INIT_AF(GPIOB, 9, GPIO_AF9_FDCAN1, GPIO_OUTPUT_ULTRA_SPEED, GPIO_OUTPUT_PUSH_PULL, GPIO_INPUT_OPEN_DRAIN)

#endif // PHAL_FDCAN_H_
