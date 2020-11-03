/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sdcard_sdio drivers_sdcard_sdio
 * @ingroup     drivers_storage
 * @brief       Driver for reading and writing SD cards via SDIO
 *
 * @{
 *
 * @file
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 */

#ifndef SDCARD_SDIO_H
#define SDCARD_SDIO_H

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
} sdcard_sdio_params_t;

/**
 * @brief   Device descriptor for the driver
 */
typedef struct {
    /** Device initialization parameters */
    sdcard_sdio_params_t params;
} sdcard_sdio_t;

/**
 * @brief   Initialize the given device
 *
 * @param[inout] dev        Device descriptor of the driver
 * @param[in]    params     Initialization parameters
 *
 * @return                  0 on success
 */
int sdcard_sdio_init(sdcard_sdio_t *dev, const sdcard_sdio_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_SDIO_H */
/** @} */
