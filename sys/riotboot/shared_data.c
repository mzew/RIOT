
#include "riotboot/shared_data.h"
#include "flash_layout.h"

shared_data_t boot_app_data;

void load_shared_data(mtd_dev_t* mtd, shared_data_t* data) {
    mtd_read(mtd, data, FL_BOOT_APP_DATA, sizeof(shared_data_t));
}

void store_shared_data(mtd_dev_t* mtd, shared_data_t* data) {
    mtd_write(mtd, data, FL_BOOT_APP_DATA, sizeof(shared_data_t) );
}

void shared_data_init(shared_data_t* d) {
    d->bootCount = 0;
    d->updatePending = 0;
}

bool shared_data_valid(shared_data_t* d) {
    if (d->bootCount == 0xffffffff)
        return false;

    if (d->updatePending == 0xffffffff)
        return false;

    if (d->bootCount > 3)
        return false;

    return true;
}
