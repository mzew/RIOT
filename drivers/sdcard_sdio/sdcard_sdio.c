/*
 * Copyright (C) 2020 YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sdcard_sdio
 * @{
 *
 * @file
 * @brief       Device driver implementation for the drivers_sdcard_sdio
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 *
 * @}
 */

#include "sdcard_sdio.h"
#include "sdcard_sdio_constants.h"
#include "sdcard_sdio_params.h"

static void _sdcard_sdio_parse(sdcard_sdio_t* card) {
    uint32_t dev_size;
    uint32_t dev_size_mul;

    // Parse the CSD register
    card->CSDVer = card->CSD[0] >> 6; // CSD version
    if (card->Type != SDCT_MMC) {
        // SD
        card->MaxBusClkFreq = card->CSD[3];
        if (card->CSDVer == 0) {
            // CSD v1.00 (SDSCv1, SDSCv2)
            dev_size  = (uint32_t)(card->CSD[6] & 0x03) << 10; // Device size
            dev_size |= (uint32_t)card->CSD[7] << 2;
            dev_size |= (card->CSD[8] & 0xc0) >> 6;
            dev_size_mul  = (card->CSD[ 9] & 0x03) << 1; // Device size multiplier
            dev_size_mul |= (card->CSD[10] & 0x80) >> 7;
            card->BlockCount  = dev_size + 1;
            card->BlockCount *= 1 << (dev_size_mul + 2);
            card->BlockSize   = 1 << (card->CSD[5] & 0x0f); // Maximum read data block length
        } else {
            // CSD v2.00 (SDHC, SDXC)
            dev_size  = (card->CSD[7] & 0x3f) << 16;
            dev_size |=  card->CSD[8] << 8;
            dev_size |=  card->CSD[9]; // C_SIZE
            card->BlockSize = 512;
            card->BlockCount = dev_size + 1;
            // BlockCount >= 65535 means that this is SDXC card
        }
        card->Capacity = card->BlockCount * card->BlockSize;
    } else {
        // MMC
        card->MaxBusClkFreq = card->CSD[3];
        dev_size  = (uint32_t)(card->CSD[6] & 0x03) << 8; // C_SIZE
        dev_size += (uint32_t)card->CSD[7];
        dev_size <<= 2;
        dev_size += card->CSD[8] >> 6;
        card->BlockSize = 1 << (card->CSD[5] & 0x0f); // MMC read block length
        dev_size_mul = ((card->CSD[9] & 0x03) << 1) + ((card->CSD[10] & 0x80) >> 7);
        card->BlockCount = (dev_size + 1) * (1 << (dev_size_mul + 2));
        card->Capacity = card->BlockCount * card->BlockSize;
    }

    // Parse the CID register
    if (card->Type != SDCT_MMC) {
        // SD card
        card->MID = card->CID[0];
        card->OID = (card->CID[1] << 8) | card->CID[2];
        card->PNM[0] = card->CID[3];
        card->PNM[1] = card->CID[4];
        card->PNM[2] = card->CID[5];
        card->PNM[3] = card->CID[6];
        card->PNM[4] = card->CID[7];
        card->PRV = card->CID[8];
        card->PSN = (card->CID[9] << 24) | (card->CID[10] << 16) | (card->CID[11] << 8) | card->CID[12];
        card->MDT = ((card->CID[13] << 8) | card->CID[14]) & 0x0fff;
    } else {
        // MMC
        card->MID = 0x00;
        card->OID = 0x0000;
        card->PNM[0] = '*';
        card->PNM[1] = 'M';
        card->PNM[2] = 'M';
        card->PNM[3] = 'C';
        card->PNM[4] = '*';
        card->PRV = 0;
        card->PSN = 0x00000000;
        card->MDT = 0x0000;
    }
}

int sdcard_sdio_init(sdcard_sdio_t *dev, sdio_t bus)
{
    dev->bus = bus;
    sdio_init(dev->bus);
    sdio_set_bus_clock(dev->bus, SDIO_CLK_400KHZ);

    SDResult cmd_res;

    // Populate SDCard structure with default values
    *dev = (SDCard_TypeDef){0};
    dev->Type = SDCT_UNKNOWN;
    cmd_res = sdio_detect_card(dev->bus, &dev->Type);
    if (cmd_res != SDR_Success)
        return cmd_res;

    // Now the CMD2 and CMD3 commands should be issued in cycle until timeout to enumerate all cards on the bus
    // Since this module suitable to work with single card, issue this commands one time only

    // Send ALL_SEND_CID command
    sdio_cmd(dev->bus, SD_CMD_ALL_SEND_CID, 0, SD_RESP_LONG); // CMD2
    cmd_res = sdio_wait_R2(dev->bus, (uint32_t *)dev->CID); // response is a value of the CID/CSD register
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    if (dev->Type != SDCT_MMC) {
        // Send SEND_REL_ADDR command to ask the SD card to publish a new RCA (Relative Card Address)
        // Once the RCA is received the card state changes to the stand-by state
        sdio_cmd(dev->bus, SD_CMD_SEND_REL_ADDR, 0, SD_RESP_SHORT); // CMD3
        cmd_res = sdio_wait_R6(dev->bus, SD_CMD_SEND_REL_ADDR, (uint16_t *)(&dev->RCA));
        if (cmd_res != SDR_Success) {
            return cmd_res;
        }
    } else {
        ////////////////////////////////////////////////////////////////
        // This part has not been tested due to lack of MMCmicro card //
        ////////////////////////////////////////////////////////////////

        // For MMC card host should set a RCA value to the card by SET_REL_ADDR command
        sdio_cmd(dev->bus, SD_CMD_SEND_REL_ADDR, SDIO_MMC_RCA << 16, SD_RESP_SHORT); // CMD3
        cmd_res = sdio_wait_R1(dev->bus, SD_CMD_SEND_REL_ADDR);
        if (cmd_res != SDR_Success) {
            return cmd_res;
        }
        dev->RCA = SDIO_MMC_RCA;
    }

    // Send SEND_CSD command to retrieve CSD register from the card
    sdio_cmd(dev->bus, SD_CMD_SEND_CSD, dev->RCA << 16, SD_RESP_LONG); // CMD9
    cmd_res = sdio_wait_R2(dev->bus, (uint32_t *)dev->CSD);
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    // Parse the values of CID and CSD registers
    _sdcard_sdio_parse(dev);

    // Now card must be in stand-by mode, from this point it is possible to increase bus speed
    sdio_set_bus_clock(dev->bus, SDIO_CLK_24MHZ);

    // Put the SD card to the transfer mode
    sdio_cmd(dev->bus, SD_CMD_SEL_DESEL_CARD, dev->RCA << 16, SD_RESP_SHORT); // CMD7
    cmd_res = sdio_wait_R1(dev->bus, SD_CMD_SEL_DESEL_CARD); // In fact R1b response here
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    // Disable the pull-up resistor on CD/DAT3 pin of card
    // Send leading command for ACMD<n> command
    sdio_cmd(dev->bus, SD_CMD_APP_CMD, dev->RCA << 16, SD_RESP_SHORT); // CMD55
    cmd_res = sdio_wait_R1(dev->bus, SD_CMD_APP_CMD);
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }
    // Send SET_CLR_CARD_DETECT command
    sdio_cmd(dev->bus, SD_CMD_SET_CLR_CARD_DETECT, 0, SD_RESP_SHORT); // ACMD42
    cmd_res = sdio_wait_R1(dev->bus, SD_CMD_SET_CLR_CARD_DETECT);
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    // For SDv1, SDv2 and MMC card must set block size
    // The SDHC/SDXC always have fixed block size (512 bytes)
    if ((dev->Type == SDCT_SDSC_V1) || (dev->Type == SDCT_SDSC_V2) || (dev->Type == SDCT_MMC)) {
        sdio_cmd(dev->bus, SD_CMD_SET_BLOCKLEN, 512, SD_RESP_SHORT); // CMD16
        cmd_res = sdio_wait_R1(dev->bus, SD_CMD_SET_BLOCKLEN);
        if (cmd_res != SDR_Success) {
            return SDR_SetBlockSizeFailed;
        }
    }

    return SDR_Success;
}

int sdcard_sdio_set_bus_width(sdcard_sdio_t *dev, sdio_width_t w) {
    SDResult cmd_res = SDR_Success;
    uint32_t clk;
    if (dev->Type != SDCT_MMC) {
        // Send leading command for ACMD<n> command
        sdio_cmd(dev->bus, SD_CMD_APP_CMD, dev->RCA << 16, SD_RESP_SHORT); // CMD55
        cmd_res = sdio_wait_R1(dev->bus, SD_CMD_APP_CMD);
        if (cmd_res != SDR_Success) {
            return cmd_res;
        }

        // Send SET_BUS_WIDTH command
        clk = (w == SDIO_1bit) ? 0x00000000 : 0x00000002;
        sdio_cmd(dev->bus, SD_CMD_SET_BUS_WIDTH, clk, SD_RESP_SHORT); // ACMD6
        cmd_res = sdio_wait_R1(dev->bus, SD_CMD_SET_BUS_WIDTH);
        if (cmd_res != SDR_Success) {
            return cmd_res;
        }
    } else {
        // MMC supports only 8-bit ?
    }

    return sdio_set_bus_width(dev->bus, w);
}

int sdcard_sdio_read(sdcard_sdio_t *dev, uint32_t addr, uint32_t* data, uint32_t len) {
    // SDSC card uses byte unit address and
    // SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
    // For SDHC card addr must be converted to block unit address
    if (dev->Type == SDCT_SDHC) {
        addr >>= 9;
    }
    return sdio_read_blocks(dev->bus, addr, data, len);
}

int sdcard_sdio_write(sdcard_sdio_t *dev, uint32_t addr, uint32_t* data, uint32_t len) {
    // SDSC card uses byte unit address and
    // SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
    // For SDHC card addr must be converted to block unit address
    if (dev->Type == SDCT_SDHC) {
        addr >>= 9;
    }
    return sdio_write_blocks(dev->bus, dev->RCA, addr, data, len);
}
