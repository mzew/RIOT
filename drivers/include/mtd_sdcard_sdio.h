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

/* Add header includes here */

#ifdef __cplusplus
extern "C" {
#endif

/* Declare the API of the driver */

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    /* add initialization params here */
} mtd_sdcard_sdio_params_t;

/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
    /** Device initialization parameters */
    mtd_sdcard_sdio_params_t params;
} mtd_sdcard_sdio_t;

/**
 * @brief   Initialize the given device
 *
 * @param[inout] dev        Device descriptor of the driver
 * @param[in]    params     Initialization parameters
 *
 * @return                  0 on success
 */
int mtd_sdcard_sdio_init(mtd_sdcard_sdio_t *dev, const mtd_sdcard_sdio_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* MTD_SDCARD_SDIO_H */
/** @} */
