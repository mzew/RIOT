/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_mtd_sdcard_sdio
 *
 * @{
 * @file
 * @brief       Default configuration
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 */

#ifndef MTD_SDCARD_SDIO_PARAMS_H
#define MTD_SDCARD_SDIO_PARAMS_H

#include "board.h"
#include "mtd_sdcard_sdio.h"
#include "mtd_sdcard_sdio_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef MTD_SDCARD_SDIO_PARAM_PARAM1
#define MTD_SDCARD_SDIO_PARAM_PARAM1
#endif

#ifndef MTD_SDCARD_SDIO_PARAMS
#define MTD_SDCARD_SDIO_PARAMS
#endif
/**@}*/

/**
 * @brief   Configuration struct
 */
static const mtd_sdcard_sdio_params_t mtd_sdcard_sdio_params[] =
{
    MTD_SDCARD_SDIO_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* MTD_SDCARD_SDIO_PARAMS_H */
/** @} */
