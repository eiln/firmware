/**
 * @file bootloader.c
 * @author Eileen Yoon (eyn@purdue.edu)
 * @brief CAN Bootloader:
 *  - A/B partition (seamless) updates for OTA
 *  - Load/store locked backup firmware in partition C
 *  - Download/Upload firmware over buffered CAN-TP (WIP, kinda)
 *
 * @version 0.1
 * @date 2024-12-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#if defined(STM32F407xx) || defined(STM32F732xx)
#include "common/phal_F4_F7/flash/flash.h"
#include "common/phal_F4_F7/crc/crc.h"
#else
#error "Unsupported MCU arch"
#endif

#include "common/uds/uds.h"
#include "bootloader.h"

static bool bl_cmd_inprogress = false;
static bool bl_flash_inprogress = false;

static uint32_t firmware_wc_total = 0;
extern char _eboot_flash;

// For readability
#define BLCMD_QUERY      UDS_CMD_BL_QUERY
#define BLCMD_START      UDS_CMD_BL_START
#define BLCMD_DATA       UDS_CMD_BL_DATA
#define BLCMD_CRC        UDS_CMD_BL_CRC
#define BLCMD_CONFIGURE  UDS_CMD_BL_CONFIGURE
#define BLCMD_DOWNLOAD   UDS_CMD_BL_DOWNLOAD

/* F4:
 * 0x08000000 ]  16K [Bootloader code]
 * 0x08004000 ]  16K [Metadata region/boot manager]
 * 0x08008000 ] 256K [Bank A: Application]
 * 0x08040000 ] 256K [Bank B: Buffer]
 * 0x08080000 ] 256K [Bank C: Backup firmware]
 */
/* We want this to be unbrickable.
 *
 * Since F4 (our version) doesn't have hardware flash dual bank, we have to copy over the bank manually oursevles.
 * We always write to B as the temporary buffer. We do NOT
 */
// Running from either bootloader (0) or app (A) so we can never write directly to A

bool BL_isBootloaderLoaded(void)
{
    // Check whether we currently have a bootloader flashed at all
    // or if we just have the raw application starting at start of flash
    // If there's no bootloader we'd be overwriting the app's own code
    // when we bit flip the verified flag in the metadata section
    uint32_t app_start = (uint32_t)((void *) &_eboot_flash);
    return (app_start == BL_ADDRESS_BANK_A); // linker hack
}

bool BL_metaSanityCheck(bl_metadata_t *meta)
{
    // TODO compute CRC of meta itself
    return (meta->magic == BL_METADATA_MAGIC) && meta->crc && meta->crc != 0xffffffff &&
        meta->words && meta->words != 0xffffffff && (meta->words << 2) < MAX_FIRMWARE_SIZE &&
        meta->addr >= FLASH_BASE && meta->addr <= FLASH_END &&
        (meta->addr == BL_ADDRESS_BANK_A || meta->addr == BL_ADDRESS_BANK_B || meta->addr == BL_ADDRESS_BANK_C);
}

void BL_markFirmwareVerified(void)
{
    if (BL_isBootloaderLoaded())
    {
        bl_metadata_t meta;
        PHAL_flashReadU32_Buffered(BL_ADDRESS_META_1, (uint32_t)&meta, BL_METADATA_WC);
        if ((meta.verified == BL_FIRMWARE_NOT_VERIFIED) && BL_metaSanityCheck(&meta))
        {
            // Flip flash bits from 0b11 -> 0b00 during application code if the current application code is good (meta af)
            PHAL_flashWriteU32(BL_METADATA_VERIFIED_ADDR, BL_FIRMWARE_VERIFIED);
        }
    }
}

void BL_sendStatusMessage(uint8_t cmd, uint8_t err, uint64_t data)
{
    uint64_t payload = ((uint64_t)data) << 16 | ((uint64_t)err & 0xff) << 8 | (cmd & 0xff);
    uds_frame_send(payload);
}

static inline uint32_t BL_getMetaAddr(uint32_t flags)
{
    return (flags == BL_FLAG_BANK_C) ? BL_ADDRESS_META_C : BL_ADDRESS_META_1;
}

static void BL_processCommand_Query(uint64_t data)
{
    uint32_t flags = data & 0xffff;
    bl_metadata_t meta;

    switch (flags)
    {
        case 0: // ping
            BL_sendSuccess(BLCMD_QUERY, BL_isBootloaderLoaded() ? BL_METADATA_MAGIC : 0xDEADBEEF); // pong
        break;
        case BL_FLAG_BANK_B:
        case BL_FLAG_BANK_C:
            PHAL_flashReadU32_Buffered(BL_getMetaAddr(flags), (uint32_t)&meta, BL_METADATA_WC);
            BL_sendSuccess(BLCMD_QUERY, meta.addr);
            BL_sendSuccess(BLCMD_QUERY, meta.words);
            BL_sendSuccess(BLCMD_QUERY, meta.crc);
            BL_sendSuccess(BLCMD_QUERY, meta.verified);
        break;
        default:
            BL_sendSuccess(BLCMD_QUERY, BLERROR_UNKNOWN);
        break;
    }
}

static bool BL_processCommand_Start(uint64_t data)
{
    uint32_t words = (data & 0xffff);
    uint32_t flags = (data >> 16) & 0xffff; // Unused for now

    firmware_wc_total = 0;

    // TODO store CRC + size at the end of firmware
    if (!words || ((words << 2) >= MAX_FIRMWARE_SIZE))
    {
        BL_sendError(BLCMD_START, BLERROR_SIZE);
        return false;
    }

    if (PHAL_flashErase((uint32_t *)BL_ADDRESS_BANK_B, words) != FLASH_OK)
    {
        BL_sendError(BLCMD_START, BLERROR_FLASH);
        return false;
    }

    firmware_wc_total = words;
    bl_flash_inprogress = true;
    BL_sendSuccess(BLCMD_START, firmware_wc_total);
    return true;
}

void BL_processCommand_Data(uint64_t data)
{
    uint32_t index = data & 0xffff;
    uint32_t payload = (data >> 16) & 0xffffffff;

    uint32_t buffer_addr = BL_ADDRESS_BANK_B + index * sizeof(uint32_t);
    if (PHAL_flashWriteU32(buffer_addr, payload) != FLASH_OK)
    {
        BL_sendErrorVal(BLCMD_DATA, BLERROR_FLASH, index);
    }
    // BL_sendSuccess(BLCMD_DATA, 6); // Don't send ack msg, too slow
}

static bool _BL_setMetadata(uint32_t addr, uint32_t words, uint32_t crc, uint32_t meta_addr)
{
    bl_metadata_t meta = {
        .magic = BL_METADATA_MAGIC,
        .addr = addr,
        .words = words,
        .crc = crc,
        .flags = 0, // Unused for now, potentially CRC of meta itself
        .verified = BL_FIRMWARE_NOT_VERIFIED,
    };
    return BL_memcpyFlashBuffer(meta_addr, (uint32_t)&meta, BL_METADATA_WC, 0);
}

bool BL_setMetadata(uint32_t addr, uint32_t words, uint32_t crc)
{
    return _BL_setMetadata(addr, words, crc, BL_ADDRESS_META_1);
}

static bool BL_setMetadataC(uint32_t addr, uint32_t words, uint32_t crc)
{
    return _BL_setMetadata(addr, words, crc, BL_ADDRESS_META_C);
}

bool BL_memcpyFlashBuffer(uint32_t addr_dst, uint32_t addr_src, uint32_t words, uint32_t crc)
{
    if (PHAL_flashErase((uint32_t *)addr_dst, words) != FLASH_OK)
        return false;

    for (uint32_t i = 0; i < words; i++)
    {
        uint32_t offset = i * sizeof(uint32_t);
        if (PHAL_flashWriteU32(addr_dst + offset, *(__IO uint32_t*)(addr_src + offset)) != FLASH_OK)
            return false;
    }

    return crc ? PHAL_CRC32_Calculate((uint32_t *)addr_dst, words) == crc : true;
}

static bool BL_processCommand_CRC(uint64_t data)
{
    uint32_t crc_app = data & 0xffffffff;
    uint32_t flags = (data >> 32) & 0xffff;

    uint32_t words = firmware_wc_total;
    if (!words)
    {
        BL_sendError(BLCMD_CRC, BLERROR_SIZE);
        return false;
    }

    uint32_t crc_flash = PHAL_CRC32_Calculate((uint32_t *)BL_ADDRESS_BANK_B, words);
    if (crc_flash != crc_app)
    {
        BL_sendErrorVal(BLCMD_CRC, BLERROR_CRC, crc_flash);
        return false;
    }

    switch (flags)
    {
        case BL_FLAG_BANK_B:
            if (BL_setMetadata(BL_ADDRESS_BANK_B, words, crc_app))
            {
                BL_sendSuccess(BLCMD_CRC, crc_flash);
                bl_flash_inprogress = false;
                return true;
            }
            BL_sendError(BLCMD_CRC, BLERROR_FLASH);
        break;
        case BL_FLAG_BANK_C:
            if (BL_memcpyFlashBuffer(BL_ADDRESS_BANK_C, BL_ADDRESS_BANK_B, words, crc_flash) &&
                BL_setMetadataC(BL_ADDRESS_BANK_C, words, crc_flash)) // TODO CRC of meta itself
            {
                BL_sendSuccess(BLCMD_CRC, crc_flash);
                bl_flash_inprogress = false;
                return true;
            }
            BL_sendError(BLCMD_CRC, BLERROR_FLASH);
        break;
        default:
            BL_sendErrorVal(BLCMD_CRC, BLERROR_UNKNOWN, flags);
        break;
    }

    return false;
}

// If configuring to boot C next, copy C to B, then configure to boot from B on next boot
// This way we don't touch A, and C is read-only
static bool BL_processCommand_Configure(uint64_t data)
{
    uint32_t flags = data & 0xffff;
    bl_metadata_t meta;

    switch (flags)
    {
        case BL_FLAG_BANK_C:
            /* If we lose power during the write from C -> B,
             * we're not fucked because we still have C. */
            PHAL_flashReadU32_Buffered(BL_ADDRESS_META_C, (uint32_t)&meta, BL_METADATA_WC);
            if (BL_metaSanityCheck(&meta) &&
                BL_memcpyFlashBuffer(BL_ADDRESS_BANK_B, BL_ADDRESS_BANK_C, meta.words, meta.crc) &&
                BL_setMetadata(BL_ADDRESS_BANK_B, meta.words, meta.crc))
            {

                BL_sendSuccess(BLCMD_CONFIGURE, meta.crc);
                return true;
            }
            BL_sendError(BLCMD_CONFIGURE, BLERROR_FLASH);
        break;
        default:
            BL_sendErrorVal(BLCMD_CONFIGURE, BLERROR_UNKNOWN, flags);
        break;
    }

    return false;
}

static bool BL_processCommand_Download(uint64_t data)
{
    uint32_t flags = data & 0xffff;
    if (!(flags == BL_FLAG_BANK_A || flags == BL_FLAG_BANK_B || flags == BL_FLAG_BANK_C))
    {
        BL_sendErrorVal(BLCMD_DOWNLOAD, BLERROR_META, flags);
        return false;
    }

    bl_metadata_t meta;
    uint32_t meta_addr = BL_getMetaAddr(flags);
    PHAL_flashReadU32_Buffered(meta_addr, (uint32_t)&meta, BL_METADATA_WC);
    if (BL_metaSanityCheck(&meta))
    {
        uint64_t data = (uint64_t)(meta.words & 0xffff) | (uint64_t)meta.crc << 16;
        BL_sendSuccess(BLCMD_DOWNLOAD, data);
        for (uint32_t i = 0; i < meta.words; i++)
        {
            // TODO index and proper protocol
            BL_sendSuccess(BLCMD_DOWNLOAD, PHAL_flashReadU32(meta.addr + i * sizeof(uint32_t)));
        }
        /* BL_send blocks so no need to lock */
        return true;
    }

    BL_sendError(BLCMD_DOWNLOAD, BLERROR_META);
    return false;
}

bool BL_processCommand(uint8_t cmd, uint64_t data)
{
    bl_cmd_inprogress = true;

    switch (cmd)
    {
        case BLCMD_QUERY:
            BL_processCommand_Query(data);
            break;
        case BLCMD_START:
            BL_processCommand_Start(data);
            break;
        case BLCMD_DATA:
            BL_processCommand_Data(data);
            break;
        case BLCMD_CRC:
            BL_processCommand_CRC(data);
            break;
        case BLCMD_CONFIGURE:
            BL_processCommand_Configure(data);
            break;
        case BLCMD_DOWNLOAD:
            BL_processCommand_Download(data);
            break;
    }

    bl_cmd_inprogress = false;

    return true;
}

bool BL_flashStarted(void)
{
    return bl_cmd_inprogress || bl_flash_inprogress;
}
