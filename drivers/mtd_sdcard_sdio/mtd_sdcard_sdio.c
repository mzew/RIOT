/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mtd_sdcard_sdio
 * @{
 *
 * @file
 * @brief       Device driver implementation for the mtd_sdcard_sdio
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 *
 * @}
 */

#include "mtd_sdcard_sdio.h"

#include "periph/sdio.h"

#include <errno.h>

static int mtd_sdcard_init(mtd_dev_t *mtd);
static int mtd_sdcard_read(mtd_dev_t *mtd, void *dest, uint32_t addr,
                           uint32_t size);
static int mtd_sdcard_write(mtd_dev_t *mtd, const void *src, uint32_t addr,
                            uint32_t size);
static int mtd_sdcard_erase(mtd_dev_t *mtd, uint32_t addr, uint32_t size);
static int mtd_sdcard_power(mtd_dev_t *mtd, enum mtd_power_state power);

const mtd_desc_t mtd_sdcard_sdio_driver = {
    .init = mtd_sdcard_init,
    .read = mtd_sdcard_read,
    .write = mtd_sdcard_write,
    .erase = mtd_sdcard_erase,
    .power = mtd_sdcard_power,
};

static int mtd_sdcard_init(mtd_dev_t *dev)
{
    mtd_sdcard_sdio_t* mtd_sd = (mtd_sdcard_sdio_t*)dev;
    if((mtd_sd->init_done == true) ||
        (sdcard_sdio_init(mtd_sd->card, SDIO_DEV(0)) == 0)) {
        dev->pages_per_sector = 1;
        dev->sector_count = mtd_sd->card->BlockCount;
        dev->page_size = mtd_sd->card->BlockSize;
        return 0;
    }
    return -EIO;
}

static int mtd_sdcard_read(mtd_dev_t *dev, void *buff, uint32_t addr,
                           uint32_t size)
{
    mtd_sdcard_sdio_t* mtd_sd = (mtd_sdcard_sdio_t*)dev;
    int res = sdcard_sdio_read(mtd_sd->card, addr, buff, size);
    if (res == 0)
        return size;
    return -EIO;
}

static int mtd_sdcard_write(mtd_dev_t *dev, const void *buff, uint32_t addr,
                            uint32_t size)
{
    mtd_sdcard_sdio_t* mtd_sd = (mtd_sdcard_sdio_t*)dev;
    int res = sdcard_sdio_write(mtd_sd->card, addr, (uint32_t*)buff, size);
    if (res == 0)
        return size;
    return -EIO;
}

static int mtd_sdcard_erase(mtd_dev_t *dev,
                            uint32_t addr,
                            uint32_t size)
{
    (void)dev;
    (void)addr;
    (void)size;

#if MTD_SDCARD_SKIP_ERASE == 1
    return 0;
#else
    return -ENOTSUP; /* explicit erase currently not supported */
#endif
}

static int mtd_sdcard_power(mtd_dev_t *dev, enum mtd_power_state power)
{
    (void)dev;
    (void)power;

    /* TODO: implement power down of sdcard in sdcard_spi
    (make use of sdcard_spi_params_t.power pin) */
    return -ENOTSUP; /* currently not supported */
}
