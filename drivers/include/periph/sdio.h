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


void sdio_init(sdio_t bus);

void sdio_init_card(sdio_t bus); // if needed?

int sdio_read_blocks(sdcard_spi_t *card, int blockaddr, uint8_t* data, int blocksize, // card?
                     int nblocks, sd_rw_response_t *state);

int sdio_write_blocks(sdcard_spi_t *card, int blockaddr, const uint8_t *data, int blocksize,
                     int nblocks, sd_rw_response_t *state);

#endif // PERIPH_SDIO_H
