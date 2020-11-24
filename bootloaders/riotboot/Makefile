# Default RIOT bootloader
APPLICATION = riotboot

# Include riotboot flash partition functionality
USEMODULE += riotboot_slot

# SPI-nor golden image and update
USEMODULE += periph_spi
USEMODULE += mtd_spi_nor
USEMODULE += riotboot_flashwrite
USEMODULE += riotboot_shared_data

include ../riotboot_common.mk
