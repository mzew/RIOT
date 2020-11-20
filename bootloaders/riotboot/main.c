/*
 * Copyright (C) 2017 Kaspar Schleiser <kaspar@schleiser.de>
 *                    Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    bootloaders    RIOT compatible bootloaders
 * @ingroup     bootloaders
 * @{
 *
 * @file
 * @brief      Minimal riot-based bootloader
 *
 * @author     Kaspar Schleiser <kaspar@schleiser.de>
 * @author     Francisco Acosta <francisco.acosta@inria.fr>
 *
 * @}
 */

#include "cpu.h"
#include "panic.h"
#include "riotboot/slot.h"

#include "periph/gpio.h"
#include "periph/spi.h"
#include "mtd.h"
#include "mtd_spi_nor.h"
#include "flash_layout.h"
#include "riotboot/flashwrite.h"

typedef struct {
    uint32_t bootCount;
    uint32_t updatePending;
} boot_app_data_t;

typedef struct {
    uint32_t size;
    uint32_t crc;
} fw_header;

static void load_bootapp_data(mtd_dev_t* mtd, boot_app_data_t* data) {
    mtd_read(mtd, data, FL_BOOT_APP_DATA, sizeof(boot_app_data_t));
}

static void store_bootapp_data(mtd_dev_t* mtd, boot_app_data_t* data) {
    mtd_write(mtd, data, FL_BOOT_APP_DATA, sizeof(boot_app_data_t) );
}

static void boot_app_data_init(boot_app_data_t* d) {
    d->bootCount = 0;
    d->updatePending = 0;
}

static bool boot_app_data_valid(boot_app_data_t* d) {
    if (d->bootCount == 0xffffffff)
        return false;

    if (d->updatePending == 0xffffffff)
        return false;

    if (d->bootCount > 3)
        return false;

    return true;
}

static riotboot_flashwrite_t state;

static bool _boot_program_MCU_flash(mtd_dev_t* mtd, uint32_t hdr_addr, uint32_t fw_addr) {
    fw_header hdr;
    mtd_read(mtd, &hdr, hdr_addr, sizeof(hdr));

    riotboot_flashwrite_init(&state, 0);
    for (unsigned pos = 0; pos < hdr.size; pos += mtd->page_size) {
        uint8_t buf[mtd->page_size];
        mtd_read(mtd, buf, fw_addr+pos, mtd->page_size);
        if (pos == 0)
            riotboot_flashwrite_putbytes(&state, &buf[RIOTBOOT_FLASHWRITE_SKIPLEN],
                                         mtd->page_size-RIOTBOOT_FLASHWRITE_SKIPLEN, true);
        else
            riotboot_flashwrite_putbytes(&state, buf, mtd->page_size, true);
    }
    riotboot_flashwrite_finish(&state);
    return true;
}



void kernel_init(void)
{
    uint32_t version = 0;
    int slot = -1;

    boot_app_data_t data;

    spi_init(SPI_DEV(0));
    extern mtd_dev_t* mtd0;
    mtd_init(mtd0);
    load_bootapp_data(mtd0, &data);
    if (!boot_app_data_valid(&data)) {
        boot_app_data_init(&data);
        store_bootapp_data(mtd0, &data);
    }

    if (data.updatePending) {
        _boot_program_MCU_flash(mtd0, FL_UPDATE_HEADER, FL_UPDATE);
    }

    for (unsigned i = 0; i < riotboot_slot_numof; i++) {
        const riotboot_hdr_t *riot_hdr = riotboot_slot_get_hdr(i);
        if (riotboot_slot_validate(i)) {
            /* skip slot if metadata broken */
            continue;
        }
        if (riot_hdr->start_addr != riotboot_slot_get_image_startaddr(i)) {
            continue;
        }
        if (slot == -1 || riot_hdr->version > version) {
            version = riot_hdr->version;
            slot = i;
        }
    }

    if (slot != -1) {
        riotboot_slot_jump(slot);
    }

    /* serious trouble! nothing to boot */
    while (1) {}
}

NORETURN void core_panic(core_panic_t crash_code, const char *message)
{
    (void)crash_code;
    (void)message;
    while (1) {}
}
