/**
 * @file bootloader.c
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

#if defined(STM32L496xx) || defined(STM32L432xx)
#include "common/phal_L4/flash/flash.h"
#endif
#if defined(STM32F407xx) || defined(STM32F732xx)
#include "common/phal_F4_F7/flash/flash.h"
#include "common/phal_F4_F7/crc/crc.h"
#endif

#include "common/uds/uds.h"
#include "bootloader.h"

// #define EXECUTE_FROM_RAM __attribute__ ((section(".RamFunc")))
extern char _estack; // start of stack

// These functions should NOT be called by application code (running in A) hence they're in here
// To do so we would need to copy the current code to sram and etc
static void BL_JumptoApplication(uint32_t start_addr)
{
    __IO uint32_t *vtor = (__IO uint32_t*)start_addr;
    uint32_t msp = *vtor; // First firmware word
    uint32_t app_address = *(vtor + 1); // Second firmware word
    uint32_t estack = (uint32_t)((void *) &_estack); // Linker variable

    // Sanity checks to confirm app exists
    if (start_addr != BL_ADDRESS_BANK_A && start_addr != BL_ADDRESS_BANK_B && start_addr != BL_ADDRESS_BANK_C)
        return;
    // We only compile for bank A so check if first word of firmware (msp) == start of stack
    if (app_address < BL_ADDRESS_BANK_A || app_address == 0xffffffff || msp != estack)
        return;

    // Disable interrupts during critical regions
    // Getting an interrupt after we set VTOR would be bad
    __disable_irq();
    // Reset all of our used peripherals
    RCC->AHB1RSTR |= RCC_AHB1RSTR_CRCRST;
    RCC->AHB1RSTR &= ~(RCC_AHB1RSTR_CRCRST);
#if defined(STM32F407xx) || defined(STM32F732xx)
    RCC->APB1RSTR |= RCC_APB1RSTR_CAN1RST;
    RCC->APB1RSTR &= ~(RCC_APB1RSTR_CAN1RST);
#else
    RCC->APB1RSTR1 |= RCC_APB1RSTR1_CAN1RST;
    RCC->APB1RSTR1 &= ~(RCC_APB1RSTR1_CAN1RST);
#endif
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // Actually jump to application
    SCB->VTOR = (uint32_t)vtor; // VTOR
    __set_MSP(msp); // *VTOR
    __enable_irq();
   ((void(*)(void)) app_address)(); // Jump!, *(VTOR + 4)()
}

static bool BL_swapFirmwareBank(bl_metadata_t *meta)
{
    // F4 1MB doesn't support dual flash bank switch, and we only compile for bank A,
    // so if the firmware is not currently in A we have to copy it over manually
    // TODO actual hardware bank switch for F7
    if (meta->addr == BL_ADDRESS_BANK_A)
    {
        // No need to copy if it's already in A
        return true;
    }
    else if (meta->addr == BL_ADDRESS_BANK_B || meta->addr == BL_ADDRESS_BANK_C)
    {
        // If in B, copy from B to A and mark as A so we don't have to copy it again
        if (BL_memcpyFlashBuffer(BL_ADDRESS_BANK_A, meta->addr, meta->words, meta->crc) &&
            BL_setMetadata(BL_ADDRESS_BANK_A, meta->words, meta->crc)) // TODO CRC of meta itself
        {
            return true;
        }
    }

    return false;
}

static void _BL_checkAndBoot(bool initial, uint32_t meta_addr)
{
    // Check 16K metadata region to decide what to boot
    bl_metadata_t meta;
    PHAL_flashReadU32_Buffered(meta_addr, (uint32_t)&meta, BL_METADATA_WC);
    // If initial check (once on bootloader first boot), see if we want to bypass the 3 second backdoor mode
    // Check if the verified flag is set (bit flipped by app code once the app is successfully run)
    if (BL_metaSanityCheck(&meta) && (!initial || (initial && (meta.verified == BL_FIRMWARE_VERIFIED))))
    {
        if (PHAL_CRC32_Calculate((uint32_t *)meta.addr, meta.words) == meta.crc)
        {
            if (BL_swapFirmwareBank(&meta))
            {
                // Always Bank A: F4 has no double flash bank, and we only compile for bank A
                BL_JumptoApplication(BL_ADDRESS_BANK_A);
            }
        }
    }
}

void BL_checkAndBoot(bool initial)
{
    _BL_checkAndBoot(initial, BL_ADDRESS_META_1);
    _BL_checkAndBoot(initial, BL_ADDRESS_META_C); // Try to boot backup firmware
}

void bitstream_data_CALLBACK(CanParsedData_t* msg_data_a)
{
    ;
}

#if (APP_ID == APP_A_BOX)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_A_BOX(data);
}
#elif (APP_ID == APP_DASHBOARD)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_DASHBOARD(data);
}
#elif (APP_ID == APP_DAQ)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_DAQ(data);
}
#elif (APP_ID == APP_MAIN_MODULE)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_MAIN_MODULE(data);
}
#elif (APP_ID == APP_PDU)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_PDU(data);
}
#elif (APP_ID == APP_TORQUEVECTOR)
void uds_frame_send(uint64_t data)
{
    SEND_UDS_RESPONSE_TORQUE_VECTOR(data);
}
#else
#error "Unknown bootloader node!"
#endif

// Quickly setup the CAN callbacks based on Node ID
#define NODE_BL_CMD_CALLBACK(callback_name, node_id) \
void callback_name(uint64_t payload) {\
    uint8_t cmd = payload & 0xff;\
    uint64_t data = (payload >> 8);\
    if (APP_ID != node_id) return;\
    uds_handle_command(cmd, data);\
} \

NODE_BL_CMD_CALLBACK(uds_command_a_box_CALLBACK, APP_A_BOX)
NODE_BL_CMD_CALLBACK(uds_command_dashboard_CALLBACK, APP_DASHBOARD)
NODE_BL_CMD_CALLBACK(uds_command_daq_CALLBACK, APP_DAQ)
NODE_BL_CMD_CALLBACK(uds_command_main_module_CALLBACK, APP_MAIN_MODULE)
NODE_BL_CMD_CALLBACK(uds_command_pdu_CALLBACK, APP_PDU)
NODE_BL_CMD_CALLBACK(uds_command_torque_vector_CALLBACK, APP_TORQUEVECTOR)
