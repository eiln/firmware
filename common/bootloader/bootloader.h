/**
 * @file bootloader.h
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief CAN Bootloader:
 *        - A/B double bank flash buffer + CRC checksum
 *        - Download/Upload firmware over buffered CAN-TP (WIP)
 *        - Load/store backup firmware
 * @version 0.1
 * @date 2024-12-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __BOOTLOADER_COMMON_H__
#define __BOOTLOADER_COMMON_H__

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#if defined(STM32F407xx)
#include "stm32f4xx.h"
#include "stm32f407xx.h"
#elif defined(STM32F732xx)
#include "stm32f7xx.h"
#include "stm32f732xx.h"
#else
#error "Please define a MCU arch"
#endif

/* F4:
 * 0x08000000 ]  16K [Bootloader code]
 * 0x08004000 ]  16K [Metadata region/boot manager]
 * 0x08008000 ]  16K [Metadata for backup firmware]
 * 0x08008000 ] 256K [Bank A: Application]
 * 0x08040000 ] 256K [Bank B: Buffer]
 * 0x08080000 ] 256K [Bank C: Backup firmware]
 */
#define MAX_FIRMWARE_SIZE        0x40000

#define BL_ADDRESS_BOOTLOADER 0x08000000 // 0: Bootloader (16K, sector 0)
#define BL_ADDRESS_META_1     0x08004000 // 1: Metadata (16K, sector 1)
#define BL_ADDRESS_META_C     0x08008000 // 2: Metadata (16K, sector 2)
#define BL_ADDRESS_BANK_A     0x08040000 // 3: Bank A: Application (256K, sector 6..7)
#define BL_ADDRESS_BANK_B     0x08080000 // 4: Bank B: Temporary buffer (256K, sector 8..9)
#define BL_ADDRESS_BANK_C     0x080c0000 // 5: Bank C: Backup firmware (256K, sector 10..11)

#define BL_METADATA_MAGIC     0xFEE1DEAD

// NOR Flash so we can only flip from 0b1111 -> 0b0000
#define BL_FIRMWARE_NOT_VERIFIED (0xffffffff)
#define BL_FIRMWARE_VERIFIED     (0x00000000)

#define BL_FLAG_BANK_A  0xAAAA
#define BL_FLAG_BANK_B  0xBBBB
#define BL_FLAG_BANK_C  0xCCCC

typedef struct {
    uint32_t magic;    // Magic number to verify bootloader exists
    uint32_t addr;     // Address of the bank (A/B/Backup)
    uint32_t words;    // Words (u32) in firmware
    uint32_t crc;      // CRC of the firmware
    uint32_t flags;    // Unused, potentially checksum of meta itself
    uint32_t verified; // App flips flash to 0b00 if verified during application
} __attribute__((__packed__, aligned(sizeof(uint32_t)))) bl_metadata_t;
#define BL_METADATA_WC ((sizeof(bl_metadata_t)) / (sizeof(uint32_t)))
#define BL_METADATA_VERIFIED_ADDR ((BL_ADDRESS_META_1) + (((BL_METADATA_WC) - 1) * sizeof(uint32_t))) // Last member
static_assert(sizeof(bl_metadata_t) == sizeof(uint32_t) * BL_METADATA_WC);
static_assert((BL_METADATA_VERIFIED_ADDR) == 0x08004014);

bool BL_isBootloaderLoaded(void);
bool BL_metaSanityCheck(bl_metadata_t *meta);
void BL_markFirmwareVerified(void);
bool BL_processCommand(uint8_t cmd, uint64_t data);
bool BL_setMetadata(uint32_t addr, uint32_t words, uint32_t crc);
bool BL_memcpyFlashBuffer(uint32_t addr_dst, uint32_t addr_src, uint32_t words, uint32_t crc);
void BL_sendStatusMessage(uint8_t cmd, uint8_t err, uint64_t data);

#define BL_sendError(cmd, err) BL_sendStatusMessage(cmd, err, 0)
#define BL_sendErrorVal(cmd, err, val) BL_sendStatusMessage(cmd, err, val)
#define BL_sendSuccess(cmd, val) BL_sendStatusMessage(cmd, BLERROR_NONE, val)

typedef enum
{
    BLERROR_NONE = 0,
    BLERROR_CRC = 1,
    BLERROR_FLASH = 2,
    BLERROR_SIZE = 3,
    BLERROR_META = 4,
    BLERROR_UNKNOWN = 5,
} BLError_t;

/* Bootloader range: 0x10 - 0x1f */
#define UDS_CMD_BL_QUERY      0x10
#define UDS_CMD_BL_START      0x11
#define UDS_CMD_BL_DATA       0x12
#define UDS_CMD_BL_CRC        0x13
#define UDS_CMD_BL_CONFIGURE  0x14
#define UDS_CMD_BL_DOWNLOAD   0x15
#define UDS_CMD_BL_MAX        0x1f

#endif // __BOOTLOADER_COMMON_H__
