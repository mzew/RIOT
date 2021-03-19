// @@copyright_tag@@

#include <stdio.h>
#include "xtimer.h"

#include "shell.h"

#include "periph/gpio.h"
#include "periph/spi.h"
#include "mtd.h"
#include "mtd_spi_nor.h"

#include <stdbool.h>
#include <stdlib.h>
#include <time.h>

enum {
    pack_size = 256,
};


    extern mtd_dev_t* mtd0;

static void gen_packet(uint8_t* buf, size_t s) {

    srand(time(NULL));
    for (size_t i = 0; i < s; ++i) {
        buf[i] = (uint8_t)(rand()%256);
    }
}

static bool cmp_packet(uint8_t* a, uint8_t* b, size_t s) {

    bool ret = true;
    for (size_t i = 0; i < s; ++i) {
        if (a[i] != b[i]) {
            ret = false;
            break;
        }
    }
    return ret;
}

static bool _spi_flash_rw_test(uint32_t addr) {

    bool r = true;

    int __attribute__((unused)) ret = mtd_erase( mtd0, addr, 4096 );
    for (int i = 0; i < 16; ++i) {
        uint8_t tx[pack_size] = {0};
        uint8_t rx[pack_size] = {0};

        gen_packet(tx, sizeof(tx));

        ret = mtd_write( mtd0, tx, addr+i*pack_size, sizeof(tx) );
        ret = mtd_read( mtd0, rx, addr+i*pack_size, sizeof(rx) );
        r = cmp_packet(tx, rx, sizeof(tx));
        if (!r)
            break;
    }

    return r;
}

static int _flash_rw_test(int argc, char **argv) {
    if (argc != 2) {
        printf("Usage: %s <addr>\n", argv[0]);
        return 1;
    }
    if (_spi_flash_rw_test(atoi(argv[1]))) {
        puts("test OK\n");
    } else {
        puts("test FAIL\n");
    }
    return 0;
}

static const shell_command_t shell_commands[] = {
    { "rw_test", "SPI flash read-write test", _flash_rw_test },
    { NULL, NULL, NULL }
};

static int spi_flash_init(void) {
    spi_init(SPI_DEV(0));
    int ret = mtd_init(mtd0);
    if (!ret) {
        mtd_spi_nor_t *dev = (mtd_spi_nor_t *)mtd0;
        printf("SPI NOR flash successfully intialized: MfgID: %x, DevId: %x\n",
               dev->jedec_id.manuf, *(uint16_t*)(&dev->jedec_id.device[0]));
    } else {
        printf("MTD init: %d\n", ret);
    }
    return ret;
}

int main(void)
{
    puts("Hello World!");

    printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
    printf("This board features a(n) %s MCU.\n", RIOT_MCU);

//    gpio_set( EN_FLASH );
    xtimer_usleep(1000);
    spi_flash_init();
//    gpio_clear( EN_FLASH_PIN );

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
