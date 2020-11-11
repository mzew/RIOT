// -------------------------------
// (c) YUMA Engineering <X>
// Author: Yury Mazeev
// -------------------------------


#include "periph/sdio.h"
#include "xtimer.h"

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

// Bitmap to clear the SDIO command flags
#define SDIO_ICR_CMD                  ((uint32_t)(SDIO_ICR_CCRCFAILC | SDIO_ICR_CTIMEOUTC | \
                                                SDIO_ICR_CMDRENDC | SDIO_ICR_CMDSENTC))

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
    dev(bus)->CLKCR = SDIO_CLKCR_CLKEN | SDIO_CLKCR_HWFC_EN;
}

int sdio_detect_card(sdio_t bus, sdio_card_type_t* typ) {

    int cmd_res = 0;
    volatile uint32_t wait;
    uint32_t sd_type = SD_STD_CAPACITY; // SD card capacity
    // Enable the SDIO clock
    dev(bus)->POWER = SDIO_POWER_PWRCTRL;
    xtimer_usleep(1000);

    // CMD0
    wait = SD_CMD_TIMEOUT;
    sdio_cmd(bus, SD_CMD_GO_IDLE_STATE, 0x00, SD_RESP_NONE);
    while (!(dev(bus)->STA & (SDIO_STA_CTIMEOUT | SDIO_STA_CMDSENT)) && --wait);
    if ((dev(bus)->STA & SDIO_STA_CTIMEOUT) || !wait) {
        return SDR_Timeout;
    }

    // CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //           - [11:08]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //           - [07:00]: Check Pattern (recommended 0xAA)
    sdio_cmd(bus, SD_CMD_HS_SEND_EXT_CSD, SD_CHECK_PATTERN, SD_RESP_SHORT); // CMD8
    cmd_res = sdio_wait_R7(bus);
    if (cmd_res == SDR_Success) {
        // SD v2.0 or later

        // Check echo-back of check pattern
        if ((dev(bus)->RESP1 & 0x01FF) != (SD_CHECK_PATTERN & 0x01FF)) {
            return SDR_Unsupported;
        }
        sd_type = SD_HIGH_CAPACITY; // SD v2.0 or later

        // Issue ACMD41 command
        wait = SD_ACMD41_TRIALS;
        while (--wait) {
            // Send leading command for ACMD<n> command
            sdio_cmd(bus, SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
            cmd_res = sdio_wait_R1(bus, SD_CMD_APP_CMD);
            if (cmd_res != SDR_Success) {
                return cmd_res;
            }

            // ACMD41 - initiate initialization process.
            // Set 3.0-3.3V voltage window (bit 20)
            // Set HCS bit (30) (Host Capacity Support) to inform card what host support high capacity
            // Set XPC bit (28) (SDXC Power Control) to use maximum performance (SDXC only)
            sdio_cmd(bus, SD_CMD_SD_SEND_OP_COND, SD_OCR_VOLTAGE | sd_type, SD_RESP_SHORT);
            cmd_res = sdio_wait_R3(bus);
            if (cmd_res != SDR_Success) {
                return cmd_res;
            }
            if (dev(bus)->RESP1 & (1 << 31)) {
                // The SD card has finished the power-up sequence
                break;
            }
        }
        if (wait == 0) {
            // Unsupported voltage range
            return SDR_InvalidVoltage;
        }

        // This is SDHC/SDXC card?
        *typ = (dev(bus)->RESP1 & SD_HIGH_CAPACITY) ? SDCT_SDHC : SDCT_SDSC_V2;
    } else if (cmd_res == SDR_Timeout) {
        // SD v1.x or MMC

        // Issue CMD55 to reset 'Illegal command' bit of the SD card
        sdio_cmd(bus, SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
        sdio_wait_R1(bus, SD_CMD_APP_CMD);

        // Issue ACMD41 command with zero argument
        wait = SD_ACMD41_TRIALS;
        while (--wait) {
            // Send leading command for ACMD<n> command
            sdio_cmd(bus, SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
            cmd_res = sdio_wait_R1(bus, SD_CMD_APP_CMD);
            if (cmd_res != SDR_Success) {
                return cmd_res;
            }

            // Send ACMD41 - initiate initialization process (bit HCS = 0)
            sdio_cmd(bus, SD_CMD_SD_SEND_OP_COND, SD_OCR_VOLTAGE, SD_RESP_SHORT); // ACMD41
            cmd_res = sdio_wait_R3(bus);
            if (cmd_res == SDR_Timeout) {
                // MMC will not respond to this command
                break;
            }
            if (cmd_res != SDR_Success) {
                return cmd_res;
            }
            if (dev(bus)->RESP1 & (1 << 31)) {
                // The SD card has finished the power-up sequence
                break;
            }
        }
        if (wait == 0) {
            // Unknown/Unsupported card type
            return SDR_UnknownCard;
        }
        if (cmd_res != SDR_Timeout) {
            // SD v1.x
            *typ = SDCT_SDSC_V1; // SDv1
        } else {
            // MMC or not SD memory card

            ////////////////////////////////////////////////////////////////
            // This part has not been tested due to lack of MMCmicro card //
            ////////////////////////////////////////////////////////////////

            wait = SD_ACMD41_TRIALS;
            while (--wait) {
                // Issue CMD1: initiate initialization process.
                sdio_cmd(bus, SD_CMD_SEND_OP_COND, SD_OCR_VOLTAGE, SD_RESP_SHORT); // CMD1
                cmd_res = sdio_wait_R3(bus);
                if (cmd_res != SDR_Success) {
                    return cmd_res;
                }
                if (dev(bus)->RESP1 & (1 << 31)) {
                    // The SD card has finished the power-up sequence
                    break;
                }
            }
            if (wait == 0) {
                return SDR_UnknownCard;
            }
            *typ = SDCT_MMC; // MMC
        }
    }

    return cmd_res;
}

int sdio_set_bus_width(sdio_t bus, sdio_width_t w) {
    SDResult cmd_res = SDR_Success;
    uint32_t clk;

    // Configure new bus width
    clk  = dev(bus)->CLKCR;
    clk &= ~SDIO_CLKCR_WIDBUS;
    clk |= (w & SDIO_CLKCR_WIDBUS);
    dev(bus)->CLKCR = clk;

    return cmd_res;
}

static const uint8_t stm32_sdio_clk_div[] = {118, 10, 4, 1, 0, 0};

void sdio_set_bus_clock(sdio_t bus, sdio_clk_t c) {
    uint32_t clk;

    clk  = dev(bus)->CLKCR;
    clk &= ~SDIO_CLKCR_CLKDIV;
    clk |= (stm32_sdio_clk_div[c] & SDIO_CLKCR_CLKDIV);
    dev(bus)->CLKCR = clk;
    // TODO: SDIO_CLK_48MHZ - activate bypass
}

void sdio_cmd(sdio_t bus, uint8_t cmd, uint32_t arg, uint32_t resp_type) {
    // Clear command flags
    dev(bus)->ICR = SDIO_ICR_CMD;

    // Program an argument for command
    dev(bus)->ARG = arg;

    // Program command value and response type, enable CPSM
    dev(bus)->CMD = cmd | resp_type | SDIO_CMD_CPSMEN;
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

static int _sdio_stop_transfer(sdio_t bus) {
    sdio_cmd(bus, SD_CMD_STOP_TRANSMISSION, 0, SD_RESP_SHORT); // CMD12
    return sdio_wait_R1(bus, SD_CMD_STOP_TRANSMISSION);
}

int sdio_read_blocks(sdio_t bus, uint32_t addr, uint32_t *pBuf, uint32_t length) {
    SDResult cmd_res = SDR_Success;
    uint32_t blk_count = length >> 9; // Sectors in block
    register uint32_t STA; // to speed up SDIO flags checking
    register uint32_t STA_mask; // mask for SDIO flags checking

    // Initialize the data control register
    dev(bus)->DCTRL = 0;

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    dma_acquire(sdio_config[bus].dma);
    int dma_ret = dma_configure(sdio_config[bus].dma, sdio_config[bus].dma_chan, &dev(bus)->FIFO, pBuf, length >> 2,
                               DMA_PERIPH_TO_MEM, DMA_DATA_WIDTH_WORD | DMA_INC_DST_ADDR | DMA_PFCTRL | DMA_BURST);
    if (dma_ret)  {
        __BKPT();
    }
    dma_start(sdio_config[bus].dma);

    if (blk_count > 1) {
        // Prepare bit checking variable for multiple block transfer
        STA_mask = SDIO_RX_MB_FLAGS;
        // Send READ_MULT_BLOCK command
        sdio_cmd(bus, SD_CMD_READ_MULT_BLOCK, addr, SD_RESP_SHORT); // CMD18
        cmd_res = sdio_wait_R1(bus, SD_CMD_READ_MULT_BLOCK);
    } else {
        // Prepare bit checking variable for single block transfer
        STA_mask = SDIO_RX_SB_FLAGS;
        // Send READ_SINGLE_BLOCK command
        sdio_cmd(bus, SD_CMD_READ_SINGLE_BLOCK, addr, SD_RESP_SHORT); // CMD17
        cmd_res = sdio_wait_R1(bus, SD_CMD_READ_SINGLE_BLOCK);
    }
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    // Data read timeout
    dev(bus)->DTIMER = SD_DATA_R_TIMEOUT;
    // Data length
    dev(bus)->DLEN   = length;
    // Data transfer:
    //   transfer mode: block
    //   direction: to card
    //   DMA: disabled
    //   block size: 2^9 = 512 bytes
    //   DPSM: enabled
#if 0//ndef MODULE_PERIPH_DMA
    dev(bus)->DCTRL  = SDIO_DCTRL_DTDIR | (9 << 4) | SDIO_DCTRL_DTEN;

    // Receive a data block from the SDIO
    // ----> TIME CRITICAL SECTION BEGIN <----
    do {
        STA = dev(bus)->STA;
        if (STA & SDIO_STA_RXFIFOHF) {
            // The receive FIFO is half full, there are at least 8 words in it
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
            *pBuf++ = dev(bus)->FIFO;
        }
    } while (!(STA & STA_mask));
    // <---- TIME CRITICAL SECTION END ---->
#else

    dev(bus)->DCTRL  = SDIO_DCTRL_DTDIR | (9 << 4) | SDIO_DCTRL_DTEN | SDIO_DCTRL_DMAEN;
    dma_wait(sdio_config[bus].dma);
    xtimer_usleep(50);
    dma_stop(sdio_config[bus].dma);
    STA = dev(bus)->STA & STA_mask;
    dma_release(sdio_config[bus].dma);
#endif

    // Send stop transmission command in case of multiple block transfer
    if (blk_count > 1) { // && (SDCard.Type != SDCT_MMC)
        cmd_res = _sdio_stop_transfer(bus);
    }

    // Check for errors
    if (STA & SDIO_XFER_ERROR_FLAGS) {
        if (STA & SDIO_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
        if (STA & SDIO_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
        if (STA & SDIO_STA_RXOVERR)  cmd_res = SDR_RXOverrun;
    }

    // Read the data remnant from RX FIFO (if there is still any data)
    while (dev(bus)->STA & SDIO_STA_RXDAVL) {
        *pBuf++ = dev(bus)->FIFO;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    return cmd_res;
}

int _sdio_get_card_state(sdio_t bus, uint16_t rca, uint8_t *pState) {
    uint8_t cmd_res = SDR_Success;

    // Send SEND_STATUS command
    sdio_cmd(bus, SD_CMD_SEND_STATUS, rca << 16, SD_RESP_SHORT); // CMD13
    cmd_res = sdio_wait_R1(bus, SD_CMD_SEND_STATUS);
    if (cmd_res != SDR_Success) {
        *pState = SD_STATE_ERROR;
        return cmd_res;
    }

    // Find out a card status
    *pState = (dev(bus)->RESP1 & 0x1e00) >> 9;

    // Check for errors
    return cmd_res;
}

int sdio_write_blocks(sdio_t bus, uint16_t rca, uint32_t addr, uint32_t *pBuf, uint32_t length) {
    SDResult cmd_res = SDR_Success;
    uint32_t blk_count = length >> 9; // Sectors in block
    uint32_t STA; // To speed up SDIO flags checking
    register uint32_t STA_mask; // Mask for SDIO flags checking
    uint32_t data_sent = 0; // Counter of transferred bytes
    uint8_t card_state; // Card state

    // Initialize the data control register
    dev(bus)->DCTRL = 0;

    if (blk_count > 1) {
        // Prepare bit checking variable for multiple block transfer
        STA_mask = SDIO_TX_MB_FLAGS;
        // Send WRITE_MULTIPLE_BLOCK command
        sdio_cmd(bus, SD_CMD_WRITE_MULTIPLE_BLOCK, addr, SD_RESP_SHORT); // CMD25
        cmd_res = sdio_wait_R1(bus, SD_CMD_WRITE_MULTIPLE_BLOCK);
    } else {
        // Prepare bit checking variable for single block transfer
        STA_mask = SDIO_TX_SB_FLAGS;
        // Send WRITE_BLOCK command
        sdio_cmd(bus, SD_CMD_WRITE_BLOCK, addr, SD_RESP_SHORT); // CMD24
        cmd_res = sdio_wait_R1(bus, SD_CMD_WRITE_BLOCK);
    }
    if (cmd_res != SDR_Success) {
        return cmd_res;
    }

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    // Data write timeout
    dev(bus)->DTIMER = SD_DATA_W_TIMEOUT;
    // Data length
    dev(bus)->DLEN = length;
    // Data transfer:
    //   transfer mode: block
    //   direction: to card
    //   DMA: disabled
    //   block size: 2^9 = 512 bytes
    //   DPSM: enabled
    dev(bus)->DCTRL = (9 << 4) | SDIO_DCTRL_DTEN;

    // Transfer data block to the SDIO
    // ----> TIME CRITICAL SECTION BEGIN <----
    if (!(length & 0x1F)) {
        // The block length is multiple of 32, simplified transfer procedure can be used
        do {
            if ((dev(bus)->STA & SDIO_STA_TXFIFOHE) && (data_sent < length)) {
                // The TX FIFO is half empty, at least 8 words can be written
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                dev(bus)->FIFO = *pBuf++;
                data_sent += 32;
            }
        } while (!(dev(bus)->STA & STA_mask));
    } else {
        // Since the block length is not a multiple of 32, it is necessary to apply additional calculations
        do {
            if ((dev(bus)->STA & SDIO_STA_TXFIFOHE) && (data_sent < length)) {
                // TX FIFO half empty, at least 8 words can be written
                uint32_t data_left = length - data_sent;
                if (data_left < 32) {
                    // Write last portion of data to the TX FIFO
                    data_left = ((data_left & 0x03) == 0) ? (data_left >> 2) : ((data_left >> 2) + 1);
                    data_sent += data_left << 2;
                    while (data_left--) {
                        dev(bus)->FIFO = *pBuf++;
                    }
                } else {
                    // Write 8 words to the TX FIFO
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    dev(bus)->FIFO = *pBuf++;
                    data_sent += 32;
                }
            }
        } while (!(dev(bus)->STA & STA_mask));
    }
    // <---- TIME CRITICAL SECTION END ---->

    // Save STA register value for further analysis
    STA = dev(bus)->STA;

    // Send stop transmission command in case of multiple block transfer
    if (blk_count > 1) { // && (SDCard.Type != SDCT_MMC)
        cmd_res = _sdio_stop_transfer(bus);
    }

    // Check for errors
    if (STA & SDIO_XFER_ERROR_FLAGS) {
        if (STA & SDIO_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
        if (STA & SDIO_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
        if (STA & SDIO_STA_TXUNDERR) cmd_res = SDR_TXUnderrun;
    }

    // Wait while the card is in programming state
    do {
        if (_sdio_get_card_state(bus, rca, &card_state) != SDR_Success) {
            break;
        }
    } while ((card_state == SD_STATE_PRG) || (card_state == SD_STATE_RCV));

    // Clear the static SDIO flags
    dev(bus)->ICR = SDIO_ICR_STATIC;

    return cmd_res;
}
