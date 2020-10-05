/*
 * Copyright (C) 2020 Yury Mazeev, <X> YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_periph_plscnt Pulse Counter
 * @ingroup     drivers_periph
 * @brief       Low-level Pulse Counter peripheral driver
 *
 * This file was inspired by qdec.h written by :
 *  Gilles DOFFE <gdoffe@gmail.com>
 *
 * @{
 * @file
 * @brief       Low-level Pulse Counter peripheral driver interface definitions
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 */

#ifndef PERIPH_PLSCNT_H
#define PERIPH_PLSCNT_H

#include <stdint.h>
#include <limits.h>

#include "periph_cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Default PLSCNT access macro
 */
#ifndef PLSCNT_DEV
#define PLSCNT_DEV(x)          (x)
#endif

/**
 * @brief  Default PLSCNT undefined value
 */
#ifndef PLSCNT_UNDEF
#define PLSCNT_UNDEF           (UINT_MAX)
#endif

/**
 * @brief   Default PLSCNT type definition
 */
#ifndef HAVE_PLSCNT_T
typedef unsigned int plscnt_t;
#endif

/**
 * @brief   Signature of event callback functions triggered from interrupts
 *
 * @param[in] arg       optional context for the callback
 */
typedef void (*plscnt_cb_t)(void *arg, unsigned channel);

typedef struct {
    unsigned period;
    unsigned channels[TIMER_CHAN];
} plscnt_ctx_t;

/**
 * @brief   Default interrupt context entry holding callback and argument
 */
#ifndef HAVE_TIMER_ISR_CTX_T
typedef struct {
    unsigned current;
    plscnt_ctx_t ctx[2];
} plscnt_isr_ctx_t;
#endif

/**
 * @brief   Initialize a PLSCNT device
 * @param[in] dev           PLSCNT device to initialize
 *
 * @return                  error code on error
 * @return                  0 on success
 */
int32_t plscnt_init(plscnt_t dev);

/**
 * @brief Read the current value of the given PLSCNT device
 *
 * @param[in] dev           the PLSCNT to read the current value from
 *
 * @return                  the PLSCNTs current value
 */
plscnt_ctx_t* plscnt_read(plscnt_t dev);

/**
 * @brief Start the given PLSCNT timer
 *
 * This function is only needed if the PLSCNT timer was stopped manually before.
 *
 * @param[in] PLSCNT          the PLSCNT device to start
 */
void plscnt_start(plscnt_t dev);

/**
 * @brief Stop the given PLSCNT timer
 *
 * This will effect all of the timer's channels.
 *
 * @param[in] PLSCNT          the PLSCNT device to stop
 */
void plscnt_stop(plscnt_t dev);

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_PLSCNT_H */
/** @} */
