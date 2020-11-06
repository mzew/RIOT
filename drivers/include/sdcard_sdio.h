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

#include "periph/sdio.h"

#ifdef __cplusplus
extern "C" {
#endif

// RCA for the MMC card
#define SDIO_MMC_RCA                  ((uint16_t)0x0001U)

/* Declare the API of the driver */

/**
 * @brief   Device initialization parameters
 */
typedef struct {
    /* add initialization params here */
} sdcard_sdio_params_t;

// SD card description
typedef struct {
    sdio_t      bus;
    uint8_t     Type;            // Card type (detected by SD_Init())
    uint32_t    Capacity;        // Card capacity (MBytes for SDHC/SDXC, bytes otherwise)
    uint32_t    BlockCount;      // SD card blocks count
    uint32_t    BlockSize;       // SD card block size (bytes), determined in SD_ReadCSD()
    uint32_t    MaxBusClkFreq;   // Maximum card bus frequency (MHz)
    uint8_t     CSDVer;          // SD card CSD register version
    uint16_t    RCA;             // SD card RCA address (only for SDIO)
    uint8_t     MID;             // SD card manufacturer ID
    uint16_t    OID;             // SD card OEM/Application ID
    uint8_t     PNM[5];          // SD card product name (5-character ASCII string)
    uint8_t     PRV;             // SD card product revision (two BCD digits: '6.2' will be 01100010b)
    uint32_t    PSN;             // SD card serial number
    uint16_t    MDT;             // SD card manufacturing date
    uint8_t     CSD[16];         // SD card CSD register (card structure data)
    uint8_t     CID[16];         // SD card CID register (card identification number)
    uint8_t     SCR[8];          // SD card SCR register (SD card configuration)
} SDCard_TypeDef;

/**
 * @brief   Device descriptor for the driver
 */
typedef SDCard_TypeDef sdcard_sdio_t;

/**
 * @brief   Initialize the given device
 *
 * @param[inout] dev        Device descriptor of the driver
 * @param[in]    params     Initialization parameters
 *
 * @return                  0 on success
 */
int sdcard_sdio_init(sdcard_sdio_t *dev, sdio_t bus);

int sdcard_sdio_set_bus_width(sdcard_sdio_t *dev, sdio_width_t w);

int sdcard_sdio_read(sdcard_sdio_t *dev, uint32_t addr, uint32_t* data, uint32_t len);

int sdcard_sdio_write(sdcard_sdio_t *dev, uint32_t addr, uint32_t* data, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* SDCARD_SDIO_H */
/** @} */
