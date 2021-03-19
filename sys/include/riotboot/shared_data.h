
#include "mtd.h"
#include <stdbool.h>

typedef struct {
    uint32_t bootCount;
    uint32_t updatePending;
} shared_data_t;

void load_shared_data(mtd_dev_t* mtd, shared_data_t* data);

void store_shared_data(mtd_dev_t* mtd, shared_data_t* data);

void shared_data_init(shared_data_t* d);

bool shared_data_valid(shared_data_t* d);
