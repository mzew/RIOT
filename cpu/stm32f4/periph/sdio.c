// -------------------------------
// (c) YUMA Engineering <X>
// Author: Yury Mazeev
// -------------------------------


#include "periph/sdio.h"

enum {
    SD_CMD_TIMEOUT = 10000,
};

// Mask for errors in card status value
#define SD_OCR_ALL_ERRORS             ((uint32_t)0xFDFFE008U) // All possible error bits
#define SD_OCR_OUT_OF_RANGE           ((uint32_t)0x80000000U) // The command's argument was out of allowed range
#define SD_OCR_ADDRESS_ERROR          ((uint32_t)0x40000000U) // A misaligned address used in the command
#define SD_OCR_BLOCK_LEN_ERROR        ((uint32_t)0x20000000U) // The transfer block length is not allowed for this card
#define SD_OCR_ERASE_SEQ_ERROR        ((uint32_t)0x10000000U) // An error in the sequence of erase commands occurred
#define SD_OCR_ERASE_PARAM            ((uint32_t)0x08000000U) // An invalid selection of write-blocks for erase occurred
#define SD_OCR_WP_VIOLATION           ((uint32_t)0x04000000U) // Attempt to write to a protected block or to the write protected card
#define SD_OCR_LOCK_UNLOCK_FAILED     ((uint32_t)0x01000000U) // Sequence or password error in lock/unlock card command
#define SD_OCR_COM_CRC_ERROR          ((uint32_t)0x00800000U) // The CRC check of the previous command failed
#define SD_OCR_ILLEGAL_COMMAND        ((uint32_t)0x00400000U) // Command not legal for the card state
#define SD_OCR_CARD_ECC_FAILED        ((uint32_t)0x00200000U) // Card internal ECC was applied but failed to correct the data
#define SD_OCR_CC_ERROR               ((uint32_t)0x00100000U) // Internal card controller error
#define SD_OCR_ERROR                  ((uint32_t)0x00080000U) // A general or an unknown error occurred during the operation
#define SD_OCR_STREAM_R_UNDERRUN      ((uint32_t)0x00040000U) // The card could not sustain data transfer in stream read operation
#define SD_OCR_STREAM_W_OVERRUN       ((uint32_t)0x00020000U) // The card could not sustain data programming in stream mode
#define SD_OCR_CSD_OVERWRITE          ((uint32_t)0x00010000U) // CSD overwrite error
#define SD_OCR_WP_ERASE_SKIP          ((uint32_t)0x00008000U) // Only partial address space was erased
#define SD_OCR_CARD_ECC_DISABLED      ((uint32_t)0x00004000U) // The command has been executed without using the internal ECC
#define SD_OCR_ERASE_RESET            ((uint32_t)0x00002000U) // An erase sequence was cleared before executing
#define SD_OCR_AKE_SEQ_ERROR          ((uint32_t)0x00000008U) // Error in the sequence of the authentication process

// Bitmap to clear the SDIO static flags (command and data)
#define SDIO_ICR_STATIC               ((uint32_t)(SDIO_ICR_CCRCFAILC | SDIO_ICR_DCRCFAILC | SDIO_ICR_CTIMEOUTC | \
                                                SDIO_ICR_DTIMEOUTC | SDIO_ICR_TXUNDERRC | SDIO_ICR_RXOVERRC  | \
                                                SDIO_ICR_CMDRENDC  | SDIO_ICR_CMDSENTC  | SDIO_ICR_DATAENDC  | \
                                                SDIO_ICR_DBCKENDC))

// Pattern for R6 response
#define SD_CHECK_PATTERN              ((uint32_t)0x000001AAU)

// R6 response error bits
#define SD_R6_GENERAL_UNKNOWN_ERROR   ((uint32_t)0x00002000U)
#define SD_R6_ILLEGAL_CMD             ((uint32_t)0x00004000U)
#define SD_R6_COM_CRC_FAILED          ((uint32_t)0x00008000U)

static inline SDIO_TypeDef *dev(sdio_t bus)
{
    return sdio_config[bus].dev;
}

static void sdio_init_pins(sdio_t bus)  {
    gpio_init(sdio_config[bus].cmd_pin, GPIO_OUT);
    gpio_init(sdio_config[bus].ck_pin, GPIO_OUT);
    gpio_init(sdio_config[bus].d0_pin, GPIO_OUT);
    gpio_init(sdio_config[bus].d1_pin, GPIO_OUT);
    gpio_init(sdio_config[bus].d2_pin, GPIO_OUT);
    gpio_init(sdio_config[bus].d3_pin, GPIO_OUT);
    gpio_init_af(sdio_config[bus].cmd_pin, sdio_config[bus].pins_af);
    gpio_init_af(sdio_config[bus].ck_pin, sdio_config[bus].pins_af);
    gpio_init_af(sdio_config[bus].d0_pin, sdio_config[bus].pins_af);
    gpio_init_af(sdio_config[bus].d1_pin, sdio_config[bus].pins_af);
    gpio_init_af(sdio_config[bus].d2_pin, sdio_config[bus].pins_af);
    gpio_init_af(sdio_config[bus].d3_pin, sdio_config[bus].pins_af);
}

void sdio_init(sdio_t bus) {
    sdio_init_pins(bus);
    periph_clk_en(sdio_config[bus].apbbus, sdio_config[bus].rccmask);
}

void sdio_init_card(sdio_t bus, sdcard_spi_t* card) {
    sdio_init(bus);
    (void)card;
}

int sdio_wait_R1(sdio_t bus, uint8_t cmd) {
    volatile uint32_t wait = SD_CMD_TIMEOUT;
    uint32_t respR1;

    // Wait for response, error or timeout
    while (!(dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)) && --wait);

    // Timeout?
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) && (wait == 0)) {
        dev(bus)->ICR = SDIO_ICR_CTIMEOUTC;
        return SDR_Timeout;
    }

    // CRC fail?
    if (dev(bus)->STA & SDIO_STA_CCRCFAIL) {
        dev(bus)->ICR = SDIO_ICR_CCRCFAILC;
        return SDR_CRCError;
    }

    // Illegal command?
    if (dev(bus)->RESPCMD != cmd) {
        return SDR_IllegalCommand;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    // Get a R1 response and analyze it for errors
    respR1 = dev(bus)->RESP1;
    if (!(respR1 & SD_OCR_ALL_ERRORS))      { return SDR_Success;          }
    if (respR1 & SD_OCR_OUT_OF_RANGE)       { return SDR_AddrOutOfRange;   }
    if (respR1 & SD_OCR_ADDRESS_ERROR)      { return SDR_AddrMisaligned;   }
    if (respR1 & SD_OCR_BLOCK_LEN_ERROR)    { return SDR_BlockLenError;    }
    if (respR1 & SD_OCR_ERASE_SEQ_ERROR)    { return SDR_EraseSeqError;    }
    if (respR1 & SD_OCR_ERASE_PARAM)        { return SDR_EraseParam;       }
    if (respR1 & SD_OCR_WP_VIOLATION)       { return SDR_WPViolation;      }
    if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED) { return SDR_LockUnlockFailed; }
    if (respR1 & SD_OCR_COM_CRC_ERROR)      { return SDR_ComCRCError;      }
    if (respR1 & SD_OCR_ILLEGAL_COMMAND)    { return SDR_IllegalCommand;   }
    if (respR1 & SD_OCR_CARD_ECC_FAILED)    { return SDR_CardECCFailed;    }
    if (respR1 & SD_OCR_CC_ERROR)           { return SDR_CCError;          }
    if (respR1 & SD_OCR_ERROR)              { return SDR_GeneralError;     }
    if (respR1 & SD_OCR_STREAM_R_UNDERRUN)  { return SDR_StreamUnderrun;   }
    if (respR1 & SD_OCR_STREAM_W_OVERRUN)   { return SDR_StreamOverrun;    }
    if (respR1 & SD_OCR_CSD_OVERWRITE)      { return SDR_CSDOverwrite;     }
    if (respR1 & SD_OCR_WP_ERASE_SKIP)      { return SDR_WPEraseSkip;      }
    if (respR1 & SD_OCR_CARD_ECC_DISABLED)  { return SDR_ECCDisabled;      }
    if (respR1 & SD_OCR_ERASE_RESET)        { return SDR_EraseReset;       }
    if (respR1 & SD_OCR_AKE_SEQ_ERROR)      { return SDR_AKESeqError;      }

    return SDR_Success;
}

int sdio_wait_R2(sdio_t bus, uint32_t* pBuf) {
    volatile uint32_t wait = SD_CMD_TIMEOUT;

    // Wait for response, error or timeout
    while (!(dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)) && --wait);

    // Timeout?
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) && (wait == 0)) {
        dev(bus)->ICR = SDIO_ICR_CTIMEOUTC;
        return SDR_Timeout;
    }

    // CRC fail?
    if (dev(bus)->STA & SDIO_STA_CCRCFAIL) {
        dev(bus)->ICR = SDIO_ICR_CCRCFAILC;
        return SDR_CRCError;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    // SDIO_RESP[1..4] registers contains the R2 response
#ifdef __GNUC__
    // Use GCC built-in intrinsics (fastest, less code) (GCC v4.3 or later)
    *pBuf++ = __builtin_bswap32(dev(bus)->RESP1);
    *pBuf++ = __builtin_bswap32(dev(bus)->RESP2);
    *pBuf++ = __builtin_bswap32(dev(bus)->RESP3);
    *pBuf   = __builtin_bswap32(dev(bus)->RESP4);
#else
    // Use ARM 'REV' instruction (fast, a bit bigger code than GCC intrinsics)
    *pBuf++ = __REV(dev(bus)->RESP1);
    *pBuf++ = __REV(dev(bus)->RESP2);
    *pBuf++ = __REV(dev(bus)->RESP3);
    *pBuf   = __REV(dev(bus)->RESP4);
/*
    // Use SHIFT, AND and OR (slower, biggest code)
    *pBuf++ = SWAP_UINT32(dev(bus)->RESP1);
    *pBuf++ = SWAP_UINT32(dev(bus)->RESP2);
    *pBuf++ = SWAP_UINT32(dev(bus)->RESP3);
    *pBuf   = SWAP_UINT32(dev(bus)->RESP4);
*/
#endif

    return SDR_Success;
}

int sdio_wait_R3(sdio_t bus) {
    volatile uint32_t wait = SD_CMD_TIMEOUT;

    // Wait for response, error or timeout
    while (!(dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)) && --wait);

    // Timeout?
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) && (wait == 0)) {
        dev(bus)->ICR = SDIO_ICR_CTIMEOUTC;
        return SDR_Timeout;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    return SDR_Success;
}

int sdio_wait_R6(sdio_t bus, uint8_t cmd, uint16_t* rca) {
    volatile uint32_t wait = SD_CMD_TIMEOUT;
    uint32_t respR6;

    // Wait for response, error or timeout
    while (!(dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)) && --wait);

    // Timeout?
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) && (wait == 0)) {
        dev(bus)->ICR = SDIO_ICR_CTIMEOUTC;
        return SDR_Timeout;
    }

    // CRC fail?
    if (dev(bus)->STA & SDIO_STA_CCRCFAIL) {
        dev(bus)->ICR = SDIO_ICR_CCRCFAILC;
        return SDR_CRCError;
    }

    // Illegal command?
    if (dev(bus)->RESPCMD != cmd) {
        return SDR_IllegalCommand;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    // Get a R6 response and analyze it for errors
    respR6 = dev(bus)->RESP1;
    if (!(respR6 & (SD_R6_ILLEGAL_CMD | SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_COM_CRC_FAILED))) {
        *rca = (uint16_t)(respR6 >> 16);
        return SDR_Success;
    }
    if (respR6 & SD_R6_GENERAL_UNKNOWN_ERROR) { return SDR_UnknownError;   }
    if (respR6 & SD_R6_ILLEGAL_CMD)           { return SDR_IllegalCommand; }
    if (respR6 & SD_R6_COM_CRC_FAILED)        { return SDR_ComCRCError;    }

    return SDR_Success;
}

int sdio_wait_R7(sdio_t bus) {
    volatile uint32_t wait = SD_CMD_TIMEOUT;

    // Wait for response, error or timeout
    while (!(dev(bus)->STA & (SDIO_STA_CCRCFAIL | SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT)) && --wait);

    // Timeout?
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) || (wait == 0)) {
        dev(bus)->ICR = SDIO_ICR_CTIMEOUTC;
        return SDR_Timeout;
    }

    // Clear command response received flag
    if (dev(bus)->STA & SDIO_STA_CMDREND) {
        dev(bus)->ICR = SDIO_ICR_CMDRENDC;
        return SDR_Success;
    }

    return SDR_NoResponse;
}

int sdio_read_blocks(sdio_t bus, int blockaddr, uint8_t* data, int blocksize, // card?
                     int nblocks, sd_rw_response_t *state) {
    (void)bus;
    (void)blockaddr;
    (void)data;
    (void)blocksize;
    (void)nblocks;
    (void)state;
    return 0;
}

int sdio_write_blocks(sdio_t bus, int blockaddr, const uint8_t *data, int blocksize,
                      int nblocks, sd_rw_response_t *state) {
    (void)bus;
    (void)blockaddr;
    (void)data;
    (void)blocksize;
    (void)nblocks;
    (void)state;
    return 0;
}
