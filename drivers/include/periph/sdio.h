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

#ifndef SDIO_DEV
#define SDIO_DEV(x)      (x)
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

// Card type
typedef enum {
    SDCT_UNKNOWN = 0x00,
    SDCT_SDSC_V1 = 0x01,  // Standard capacity SD card v1.0
    SDCT_SDSC_V2 = 0x02,  // Standard capacity SD card v2.0
    SDCT_MMC     = 0x03,  // MMC
    SDCT_SDHC    = 0x04   // High capacity SD card (SDHC or SDXC)
} sdio_card_type_t;

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

// SD commands  index
#define SD_CMD_GO_IDLE_STATE          ((uint8_t)0U)
#define SD_CMD_SEND_OP_COND           ((uint8_t)1U)  // MMC only
#define SD_CMD_ALL_SEND_CID           ((uint8_t)2U)  // Not supported in SPI mode
#define SD_CMD_SEND_REL_ADDR          ((uint8_t)3U)  // Not supported in SPI mode
#define SD_CMD_SWITCH_FUNC            ((uint8_t)6U)
#define SD_CMD_SEL_DESEL_CARD         ((uint8_t)7U)  // Not supported in SPI mode
#define SD_CMD_HS_SEND_EXT_CSD        ((uint8_t)8U)
#define SD_CMD_SEND_CSD               ((uint8_t)9U)
#define SD_CMD_SEND_CID               ((uint8_t)10U)
#define SD_CMD_READ_DAT_UNTIL_STOP    ((uint8_t)11U) // Not supported in SPI mode
#define SD_CMD_STOP_TRANSMISSION      ((uint8_t)12U)
#define SD_CMD_SEND_STATUS            ((uint8_t)13U)
#define SD_CMD_GO_INACTIVE_STATE      ((uint8_t)15U) // Not supported in SPI mode
#define SD_CMD_SET_BLOCKLEN           ((uint8_t)16U)
#define SD_CMD_READ_SINGLE_BLOCK      ((uint8_t)17U)
#define SD_CMD_READ_MULT_BLOCK        ((uint8_t)18U)
#define SD_CMD_WRITE_DAT_UNTIL_STOP   ((uint8_t)20U) // Not supported in SPI mode
#define SD_CMD_WRITE_BLOCK            ((uint8_t)24U)
#define SD_CMD_WRITE_MULTIPLE_BLOCK   ((uint8_t)25U)
#define SD_CMD_PROG_CSD               ((uint8_t)27U)
#define SD_CMD_SET_WRITE_PROT         ((uint8_t)28U) // Not supported in SPI mode
#define SD_CMD_CLR_WRITE_PROT         ((uint8_t)29U) // Not supported in SPI mode
#define SD_CMD_SEND_WRITE_PROT        ((uint8_t)30U) // Not supported in SPI mode
#define SD_CMD_ERASE                  ((uint8_t)38U)
#define SD_CMD_LOCK_UNLOCK            ((uint8_t)42U)
#define SD_CMD_APP_CMD                ((uint8_t)55U)
#define SD_CMD_READ_OCR               ((uint8_t)58U) // Read OCR register
#define SD_CMD_CRC_ON_OFF             ((uint8_t)59U) // On/Off CRC check by SD Card (in SPI mode)

// Trials count for ACMD41
#define SD_ACMD41_TRIALS              ((uint32_t)0x0000FFFFU)

// SDIO clock divider
#define SD_CLK_DIV_400K               ((uint32_t)0x00000076U) // SDIO clock 400kHz  (48MHz / (0x76 + 2) = 400kHz)
#define SD_CLK_DIV_1M                 ((uint32_t)0x0000002EU) // SDIO clock 1MHz    (48MHz / (0x2e + 2) = 1MHz)
#define SD_CLK_DIV_2M                 ((uint32_t)0x00000016U) // SDIO clock 2MHz    (48MHz / (0x16 + 2) = 2MHz)
#define SD_CLK_DIV_4M                 ((uint32_t)0x0000000AU) // SDIO clock 4MHz    (48MHz / (0x0a + 2) = 4MHz)
#define SD_CLK_DIV_6M85               ((uint32_t)0x00000005U) // SDIO clock 6.85MHz (48MHz / (0x05 + 2) = 6.85MHz)
#define SD_CLK_DIV_8M                 ((uint32_t)0x00000004U) // SDIO clock 8MHz    (48MHz / (0x04 + 2) = 8MHz)
#define SD_CLK_DIV_9M6                ((uint32_t)0x00000003U) // SDIO clock 9.6MHz  (48MHz / (0x03 + 2) = 9.6MHz)
#define SD_CLK_DIV_12M                ((uint32_t)0x00000002U) // SDIO clock 12MHz   (48MHz / (0x02 + 2) = 12MHz)
#define SD_CLK_DIV_16M                ((uint32_t)0x00000001U) // SDIO clock 16MHz   (48MHz / (0x01 + 2) = 16MHz)
#define SD_CLK_DIV_24M                ((uint32_t)0x00000000U) // SDIO clock 24MHz   (48MHz / (0x00 + 2) = 24MHz)

// SDIO clocks for initialization and data transfer
#define SD_CLK_DIV_INIT               (SD_CLK_DIV_400K) // SDIO initialization frequency (400kHz)
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_400K) // SDIO data transfer 400kHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_1M)   // SDIO data transfer 1MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_8M)   // SDIO data transfer 8MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_9M6)  // SDIO data transfer 9.6MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_12M)  // SDIO data transfer 12MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_16M)  // SDIO data transfer 16MHz
#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_24M) // SDIO data transfer frequency

// SDIO CMD response type
#define SD_RESP_NONE                  ((uint32_t)0x00000000U) // No response
#define SD_RESP_SHORT                 (SDIO_CMD_WAITRESP_0)  // Short response
#define SD_RESP_LONG                  (SDIO_CMD_WAITRESP_1)    // Long response

// Following commands are SD Card Specific commands.
// SD_CMD_APP_CMD should be sent before sending these commands.
#define SD_CMD_SET_BUS_WIDTH          ((uint8_t)6U)  // ACMD6
#define SD_CMD_SD_SEND_OP_COND        ((uint8_t)41U) // ACMD41
#define SD_CMD_SET_CLR_CARD_DETECT    ((uint8_t)42U) // ACMD42
#define SD_CMD_SEND_SCR               ((uint8_t)51U) // ACMD51

// Pattern for R6 response
#define SD_CHECK_PATTERN              ((uint32_t)0x000001AAU)

// R6 response error bits
#define SD_R6_GENERAL_UNKNOWN_ERROR   ((uint32_t)0x00002000U)
#define SD_R6_ILLEGAL_CMD             ((uint32_t)0x00004000U)
#define SD_R6_COM_CRC_FAILED          ((uint32_t)0x00008000U)

// Argument for ACMD41 to select voltage window
#define SD_OCR_VOLTAGE                ((uint32_t)0x80100000U)

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

// Card state (OCR[12:9] bits CURRENT_STATE)
#define SD_STATE_IDLE                 ((uint8_t)0x00U) // Idle
#define SD_STATE_READY                ((uint8_t)0x01U) // Ready
#define SD_STATE_IDENT                ((uint8_t)0x02U) // Identification
#define SD_STATE_STBY                 ((uint8_t)0x03U) // Stand-by
#define SD_STATE_TRAN                 ((uint8_t)0x04U) // Transfer
#define SD_STATE_DATA                 ((uint8_t)0x05U) // Sending data
#define SD_STATE_RCV                  ((uint8_t)0x06U) // Receive data
#define SD_STATE_PRG                  ((uint8_t)0x07U) // Programming
#define SD_STATE_DIS                  ((uint8_t)0x08U) // Disconnect
#define SD_STATE_ERROR                ((uint8_t)0xFFU) // Error or unknown state

// Mask for ACMD41
#define SD_STD_CAPACITY               ((uint32_t)0x00000000U)
#define SD_HIGH_CAPACITY              ((uint32_t)0x40000000U)

// Timeout for CMD0 or CMD8
#define SD_CMD_TIMEOUT                ((uint32_t)0x00010000U)

// SDIO timeout for data transfer ((48MHz / CLKDIV / 1000) * timeout_ms)
#define SD_DATA_R_TIMEOUT             ((uint32_t)((48000000U / (SD_CLK_DIV_TRAN + 2U) / 1000U) * 100U)) // Data read timeout is 100ms
#define SD_DATA_W_TIMEOUT             ((uint32_t)((48000000U / (SD_CLK_DIV_TRAN + 2U) / 1000U) * 250U)) // Date write timeout is 250ms

// SDIO transfer flags
#define SDIO_XFER_COMMON_FLAGS        (SDIO_STA_DTIMEOUT | SDIO_STA_DCRCFAIL)

// SDIO flags for single block receive
#define SDIO_RX_SB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDIO_STA_DBCKEND | SDIO_STA_RXOVERR)

// SDIO flags for multiple block receive
#define SDIO_RX_MB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDIO_STA_DATAEND | SDIO_STA_RXOVERR)

// SDIO flags for single block transmit
#define SDIO_TX_SB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDIO_STA_DBCKEND | SDIO_STA_TXUNDERR)

// SDIO flags for multiple block transmit
#define SDIO_TX_MB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDIO_STA_DATAEND | SDIO_STA_TXUNDERR)

// SDIO transfer error flags
#define SDIO_XFER_ERROR_FLAGS         (SDIO_XFER_COMMON_FLAGS | SDIO_STA_TXUNDERR | SDIO_STA_RXOVERR)

// stm32's SDIO peripheral related functions

void sdio_init(sdio_t bus);

int sdio_detect_card(sdio_t bus, sdio_card_type_t* typ);

int sdio_set_bus_width(sdio_t bus, sdio_width_t w);

void sdio_set_bus_clock(sdio_t bus, sdio_clk_t clk);

int sdio_getSCR(sdio_t bus, uint16_t rca, uint32_t* scr);

// SDIO command related functions

void sdio_cmd(sdio_t bus, uint8_t cmd, uint32_t arg, uint32_t resp_type);

int sdio_wait_R1(sdio_t bus, uint8_t cmd);

// return status
int sdio_wait_R2(sdio_t bus, uint32_t* pBuf);

int sdio_wait_R3(sdio_t bus);

int sdio_wait_R6(sdio_t bus, uint8_t cmd, uint16_t* rca);

int sdio_wait_R7(sdio_t bus);

int sdio_read_blocks(sdio_t bus, uint32_t addr, uint32_t *pBuf, uint32_t length);

int sdio_write_blocks(sdio_t bus, uint16_t rca, uint32_t addr, uint32_t *pBuf, uint32_t length);

#endif // PERIPH_SDIO_H
