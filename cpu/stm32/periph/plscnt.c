/*
 * Copyright (C) 2020 Yury Mazeev, <X> YUMA Engineering
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_cortexm_common
 * @ingroup     drivers_periph_plscnt
 * @{
 *
 * @file
 * @brief       Low-level Pulse Counter driver implementation
 *
 * @author      Yury Mazeev <yuri.mazeyev@gmail.com>
 *
 * @}
 */

#include <errno.h>

#include "cpu.h"
#include "assert.h"
#include "periph/plscnt.h"
#include "periph/gpio.h"

#include <string.h>

#ifdef PLSCNT_NUMOF

/**
 * @brief   Interrupt context for each configured qdec
 */
static plscnt_isr_ctx_t isr_ctx[PLSCNT_NUMOF];

static inline TIM_TypeDef *dev(plscnt_t qdec)
{
    return plscnt_config[qdec].dev;
}

int32_t plscnt_init(plscnt_t t)
{
    /* Control variables */
    uint8_t i = 0;

    /* Verify parameters */
    assert((t < PLSCNT_NUMOF));

    /* Power on the used timer */
    periph_clk_en(plscnt_config[t].bus, plscnt_config[t].rcc_mask);

    /* Reset configuration and CC channels */
    dev(t)->CR1 = 0;
    dev(t)->CR2 = 0;
    dev(t)->SMCR = 0;
    dev(t)->CCER = 0;
    dev(t)->CCMR1 = 0;
    dev(t)->CCMR2 = 0;

    /* Reset configuration and CC channels */
    dev(t)->CCR1 = 0;
    dev(t)->CCR2 = 0;
    dev(t)->CCR3 = 0;
    dev(t)->CCR4 = 0;

    uint32_t ccer = 0;
    uint32_t dier = 0;


    /* Configure the used pins */
    i = 0;
    while ((i < TIMER_CHANNEL_NUMOF) && (plscnt_config[t].chan[i].pin != GPIO_UNDEF)) {
        gpio_init(plscnt_config[t].chan[i].pin, GPIO_IN_PU);
        gpio_init_af(plscnt_config[t].chan[i].pin, plscnt_config[t].af);
        // configure cc-channels to capture rising edge
        switch (plscnt_config[t].chan[i].cc_chan) {
        case 0:
            dev(t)->CCMR1 |= TIM_CCMR1_CC1S_0;
            break;
        case 1:
            dev(t)->CCMR1 |= TIM_CCMR1_CC2S_0;
            break;
        case 2:
            dev(t)->CCMR2 |= TIM_CCMR2_CC3S_0;
            break;
        case 3:
            dev(t)->CCMR2 |= TIM_CCMR2_CC4S_0;
            break;
        default:
            break;
        }
        // enable interrupt
        ccer |= TIM_CCER_CC1E << 4*plscnt_config[t].chan[i].cc_chan;
        dier |= TIM_DIER_CC1IE << plscnt_config[t].chan[i].cc_chan;

        i++;
    }
    dev(t)->CCER |= ccer;
    dev(t)->DIER |= dier;

    dev(t)->ARR = plscnt_config[t].max;

    dev(t)->CNT = 0;

    /* Initialize the interrupt context */
    memset(&isr_ctx[t], 0, sizeof(plscnt_isr_ctx_t));

    /* Enable the qdec's interrupt */
    NVIC_EnableIRQ(plscnt_config[t].irqn);

//    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;
    /* Reset counter and start qdec */
    plscnt_start(t);

    return 0;
}

plscnt_ctx_t* plscnt_read(plscnt_t t)
{
    unsigned next_ctx = isr_ctx[t].current ^ 0x1;
    // clear next part of double buffer
    memset(&isr_ctx[t].ctx[next_ctx], 0, sizeof(plscnt_ctx_t));

    plscnt_ctx_t* ret = &isr_ctx[t].ctx[isr_ctx[t].current];

    uint32_t irq_save = irq_disable();
    isr_ctx[t].current = next_ctx;
    irq_restore(irq_save);

    return ret;
}

void plscnt_start(plscnt_t t)
{
    // Reset counter
    dev(t)->EGR |= TIM_EGR_UG;
    dev(t)->CR1 |= TIM_CR1_CEN;
}

void plscnt_stop(plscnt_t t)
{
    dev(t)->CR1 &= ~TIM_CR1_CEN;
}

static inline void irq_handler(plscnt_t t)
{
    uint32_t status = (dev(t)->SR & dev(t)->DIER);

    plscnt_ctx_t* ctx = &isr_ctx[t].ctx[isr_ctx[t].current];

    for (unsigned bit = 0; bit < 4; ++bit)
    {
        uint32_t mask = 0x1UL << (bit+1);
        if (status & mask)
        {
            uint32_t now = *((uint32_t*)(&dev(t)->CCR1) + bit);
            ctx->avg_period[bit] = now - isr_ctx[t].last_reading[bit];
            isr_ctx[t].last_reading[bit] = now;
            dev(t)->SR &= ~mask;
        }
    }
    cortexm_isr_end();
}


#ifdef PLSCNT_0_ISR
void PLSCNT_0_ISR(void)
{
    irq_handler(0);
}
#endif

#ifdef PLSCNT_1_ISR
void PLSCNT_1_ISR(void)
{
    irq_handler(1);
}
#endif

#ifdef PLSCNT_2_ISR
void PLSCNT_2_ISR(void)
{
    irq_handler(2);
}
#endif

#ifdef PLSCNT_3_ISR
void PLSCNT_3_ISR(void)
{
    irq_handler(3);
}
#endif

#ifdef PLSCNT_4_ISR
void PLSCNT_4_ISR(void)
{
    irq_handler(4);
}
#endif

#endif /* PLSCNT_NUMOF */
