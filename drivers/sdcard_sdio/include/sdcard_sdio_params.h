/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sdcard_sdio
 *
 * @{
 * @file
 * @brief       Default configuration
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 */

#ifndef SDCARD_SDIO_PARAMS_H
#define SDCARD_SDIO_PARAMS_H

#include "board.h"
#include "sdcard_sdio.h"
#include "sdcard_sdio_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 * @{
 */
#ifndef SDCARD_SDIO_PARAM_PARAM1
#define SDCARD_SDIO_PARAM_PARAM1
#endif

#ifndef SDCARD_SDIO_PARAMS
#define SDCARD_SDIO_PARAMS
#endif
/**@}*/

/**
 * @brief   Configuration struct
 */
static const sdcard_sdio_params_t sdcard_sdio_params[] =
{
    SDCARD_SDIO_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_SDIO_PARAMS_H */
/** @} */
