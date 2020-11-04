// -------------------------------
// (c) YUMA Engineering <X>
// Author: Yury Mazeev
// -------------------------------

#ifndef PERIPH_SDIO_H
#define PERIPH_SDIO_H

#include "sdcard_spi.h"

#include <stdint.h>

#ifndef HAVE_SDIO_T
typedef unsigned int sdio_t;
#endif

#ifndef HAVE_SDIO_CLK_T
typedef enum {
    SDIO_CLK_400KHZ = 0,
    SDIO_CLK_4MHZ,
    SDIO_CLK_8MHZ,
    SDIO_CLK_16MHZ,
    SDIO_CLK_24MHZ,
    SDIO_CLK_48MHZ,
} sdio_clk_t;
#endif

typedef enum {
    SDIO_1bit = 0,
    SDIO_4bit,
    SDIO_8Bit,
} sdio_width_t;

typedef enum {
    SDR_Success             = 0x00,
    SDR_Timeout             = 0x01,  // Timeout
    SDR_CRCError            = 0x02,  // Response for command received but CRC check failed
    SDR_ReadError           = 0x03,  // Read block error (response for CMD17)
    SDR_WriteError          = 0x04,  // Write block error (response for CMD24)
    SDR_WriteErrorInternal  = 0x05,  // Write block error due to internal card error
    SDR_Unsupported         = 0x06,  // Unsupported card found
    SDR_BadResponse         = 0x07,
    SDR_SetBlockSizeFailed  = 0x08,  // Set block size command failed (response for CMD16)
    SDR_UnknownCard         = 0x09,
    SDR_NoResponse          = 0x0A,
    SDR_AddrOutOfRange      = 0x0B,  // Address out of range
    SDR_WriteCRCError       = 0x0C,  // Data write rejected due to a CRC error
    SDR_InvalidVoltage      = 0x0D,  // Unsupported voltage range
    SDR_DataTimeout         = 0x0E,  // Data block transfer timeout
    SDR_DataCRCFail         = 0x0F,  // Data block transfer CRC failed
    SDR_RXOverrun           = 0x10,  // Receive FIFO overrun
    SDR_TXUnderrun          = 0x11,  // Transmit FIFO underrun
    SDR_StartBitError       = 0x12,  // Start bit not detected on all data signals
    SDR_AddrMisaligned      = 0x13,  // A misaligned address which did not match the block length was used in the command
    SDR_BlockLenError       = 0x14,  // The transfer block length is not allowed for this card
    SDR_EraseSeqError       = 0x15,  // An error in the sequence of erase commands occurred
    SDR_EraseParam          = 0x16,  // An invalid selection of write-blocks for erase occurred
    SDR_WPViolation         = 0x17,  // Attempt to write to a protected block or to the write protected card
    SDR_LockUnlockFailed    = 0x18,  // Error in lock/unlock command
    SDR_ComCRCError         = 0x19,  // The CRC check of the previous command failed
    SDR_IllegalCommand      = 0x1A,  // Command is not legal for the the current card state
    SDR_CardECCFailed       = 0x1B,  // Card internal ECC was applied but failed to correct the data
    SDR_CCError             = 0x1C,  // Internal card controller error
    SDR_GeneralError        = 0x1D,  // A general or an unknown error occurred during the operation
    SDR_StreamUnderrun      = 0x1E,  // The card could not sustain data transfer in stream read operation
    SDR_StreamOverrun       = 0x1F,  // The card could not sustain data programming in stream mode
    SDR_CSDOverwrite        = 0x20,  // CSD overwrite error
    SDR_WPEraseSkip         = 0x21,  // Only partial address space was erased
    SDR_ECCDisabled         = 0x22,  // The command has been executed without using the internal ECC
    SDR_EraseReset          = 0x23,  // An erase sequence was cleared before executing
    SDR_AKESeqError         = 0x24,  // Error in the sequence of the authentication process
    SDR_UnknownError        = 0xFF   // Unknown error
} SDResult;

// stm32's SDIO peripheral related functions

void sdio_init(sdio_t bus);

void sdio_init_card(sdio_t bus, sdcard_spi_t* card);

void sdio_set_bus_width(sdio_t bus, sdio_width_t w);

void sdio_set_bus_clock(sdio_t bus, sdio_clk_t clk);

// SDIO command related functions

void sdio_cmd(sdio_t bus, uint8_t cmd, uint32_t arg, uint32_t resp_type);

int sdio_wait_R1(sdio_t bus, uint8_t cmd);

// return status
int sdio_wait_R2(sdio_t bus, uint32_t* pBuf);

int sdio_wait_R3(sdio_t bus);

int sdio_wait_R6(sdio_t bus, uint8_t cmd, uint16_t* rca);

int sdio_wait_R7(sdio_t bus);

int sdio_read_blocks(sdio_t bus, int blockaddr, uint8_t* data, int blocksize,
                     int nblocks, sd_rw_response_t *state);

int sdio_write_blocks(sdio_t bus, int blockaddr, const uint8_t *data, int blocksize,
                     int nblocks, sd_rw_response_t *state);

#endif // PERIPH_SDIO_H
