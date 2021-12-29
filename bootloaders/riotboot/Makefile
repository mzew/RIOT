# Default RIOT bootloader
APPLICATION = riotboot

# Include riotboot flash partition functionality
USEMODULE += riotboot_slot

# SPI-nor golden image and update
USEMODULE += periph_spi
USEMODULE += mtd_spi_nor
USEMODULE += riotboot_flashwrite
USEMODULE += riotboot_shared_data

# stm32f4 128k alligned slot
SLOT0_OFFSET ?= 0x20000
SLOT0_LEN ?= 0x80000
RIOTBOOT_LEN ?= $(SLOT0_OFFSET)

include ../riotboot_common.mk
