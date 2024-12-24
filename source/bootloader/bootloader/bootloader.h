/**
 * @file bootloader.h
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief CAN Bootloader:
 *  - A/B partition (seamless) updates for OTA
 *  - Load/store locked backup firmware in partition C
 *  - Download/Upload firmware over buffered CAN-TP (WIP, kinda)
 *
 * @version 0.1
 * @date 2024-11-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

#include "inttypes.h"
#include "stdbool.h"
#include "can_parse.h"
#include "node_defs.h"
#include "common/bootloader/bootloader.h"

/* Magic sent IFF bootloader bootloader code is running */
#define BL_MAGIC_BOOTLOADER   0xFEE2DEAD

void BL_checkAndBoot(bool initial);
bool BL_flashStarted(void);

#endif // __BOOTLOADER_H__
