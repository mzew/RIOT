# Default RIOT bootloader
APPLICATION = riotboot

# Default testing board
BOARD ?= samr21-xpro

# Select the boards with riotboot feature
FEATURES_REQUIRED += riotboot

# Set RIOTBOOT_BUILD to indicate a riotboot application build
RIOTBOOT_BUILD = 1
# Provide a define to detect if building the bootloader
CFLAGS += -DRIOTBOOT

# Disable unused modules
CFLAGS += -DNDEBUG -DLOG_LEVEL=LOG_NONE
DISABLE_MODULE += core_init core_msg core_panic
DISABLE_MODULE += auto_init auto_init_%

# avoid using stdio
USEMODULE += stdio_null

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

# RIOT codebase
RIOTBASE ?= $(CURDIR)/../../

include $(RIOTBASE)/Makefile.include

# limit riotboot bootloader size
# TODO: Manage to set this variable for boards which already embed a
# bootloader, currently it will be overwritten
ROM_LEN := $(RIOTBOOT_LEN)
