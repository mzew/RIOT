/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_mtd_sdcard_sdio mtd_sdcard_sdio
 * @ingroup     drivers_storage
 * @brief       Implementation of MTD device for SDIO-connected SD card
 *
 * @{
 *
 * @file
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 */

#ifndef MTD_SDCARD_SDIO_H
#define MTD_SDCARD_SDIO_H

#include "sdcard_sdio.h"
#include "mtd.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
    mtd_dev_t base;
    sdcard_sdio_t *card;
    bool init_done;
} mtd_sdcard_sdio_t;

/**
 * @brief   sdcards handle sector erase internally so it's possible to directly
 *          write to the card without erasing the sector first.
 *          Attention: an erase call will therefore NOT touch the content,
 *                     so disable this feature to ensure overriding the data.
 */
#ifndef MTD_SDCARD_SKIP_ERASE
#define MTD_SDCARD_SKIP_ERASE (1)
#endif

/**
 * @brief   sdcard device operations table for mtd
 */
extern const mtd_desc_t mtd_sdcard_sdio_driver;

#ifdef __cplusplus
}
#endif

#endif /* MTD_SDCARD_SDIO_H */
/** @} */
