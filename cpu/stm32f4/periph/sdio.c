// -------------------------------
// (c) YUMA Engineering <X>
// Author: Yury Mazeev
// -------------------------------


#include "sdio.h"


void sdio_init(sdio_t bus) {

}

void sdio_init_card(sdio_t bus) {

}

int sdio_read_blocks(sdcard_spi_t *card, int blockaddr, uint8_t* data, int blocksize, // card?
                     int nblocks, sd_rw_response_t *state) {
    return 0;
}

int sdio_write_blocks(sdcard_spi_t *card, int blockaddr, const uint8_t *data, int blocksize,
                      int nblocks, sd_rw_response_t *state) {
    return 0;
}
