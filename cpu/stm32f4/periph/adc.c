/*
 * Copyright (C) 2014-2016 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_stm32f4
 * @ingroup     drivers_periph_adc
 * @{
 *
 * @file
 * @brief       Low-level ADC driver implementation
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 *
 * @}
 */

#include "cpu.h"
#include "mutex.h"
#include "periph/adc.h"
#include "periph_conf.h"

/**
 * @brief   Maximum allowed ADC clock speed
 */
#define MAX_ADC_SPEED           (12000000U)

/**
 * @brief   Load the ADC configuration
 */
static const adc_conf_t adc_config[] = ADC_CONFIG;

/**
 * @brief   Allocate locks for all three available ADC devices
 */
static mutex_t locks[] = {
#if ADC_DEVS > 1
    MUTEX_INIT,
#endif
#if ADC_DEVS > 2
    MUTEX_INIT,
#endif
    MUTEX_INIT
};

static inline ADC_TypeDef *dev(adc_t line)
{
    return (ADC_TypeDef *)(ADC1_BASE + (adc_config[line].dev << 8));
}

static inline void prep(adc_t line)
{
    mutex_lock(&locks[adc_config[line].dev]);
    periph_clk_en(APB2, (RCC_APB2ENR_ADC1EN << adc_config[line].dev));
}

static inline void done(adc_t line)
{
    periph_clk_dis(APB2, (RCC_APB2ENR_ADC1EN << adc_config[line].dev));
    mutex_unlock(&locks[adc_config[line].dev]);
}

int adc_init(adc_t line)
{
    uint32_t clk_div = 2;

    /* check if the line is valid */
    if (line >= ADC_NUMOF) {
        return -1;
    }

    /* lock and power-on the device */
    prep(line);

    /* configure the pin */
    gpio_init_analog(adc_config[line].pin);
    /* set sequence length to 1 conversion and enable the ADC device */
    dev(line)->SQR1 = 0;
    dev(line)->CR2 = ADC_CR2_ADON;
    /* set clock prescaler to get the maximal possible ADC clock value */
    for (clk_div = 2; clk_div < 8; clk_div += 2) {
        if ((CLOCK_CORECLOCK / clk_div) <= MAX_ADC_SPEED) {
            break;
        }
    }
    ADC->CCR = ((clk_div / 2) - 1) << 16;

    /* free the device again */
    done(line);
    return 0;
}

int32_t adc_sample(adc_t line, adc_res_t res)
{
    int sample;

    /* check if resolution is applicable */
    if (res & 0xff) {
        return -1;
    }

    /* lock and power on the ADC device  */
    prep(line);

    /* set resolution and conversion channel */
    dev(line)->CR1 = res;
    dev(line)->SQR3 = adc_config[line].chan;
    /* start conversion and wait for results */
    dev(line)->CR2 |= ADC_CR2_SWSTART;
    while (!(dev(line)->SR & ADC_SR_EOC)) {}
    /* finally read sample and reset the STRT bit in the status register */
    sample = (int)dev(line)->DR;

    /* power off and unlock device again */
    done(line);

    return sample;
}

#ifdef MODULE_PERIPH_DMA
void adc_sample_all(adc_res_t res, uint16_t* data, size_t len) {
    assert(len >= ADC_NUMOF);

    // assume all lines belong to the same device
    // configure ADC sequence based on order of lines' definition
    adc_t line = ADC_LINE(0);
    prep(line);
    dev(line)->CR1 = res;
    dev(line)->SQR1 = 0; // is manual reset needed?
    dev(line)->SQR2 = 0;
    dev(line)->SQR3 = 0;
    for (unsigned i = 0; i < ADC_NUMOF; ++i) {
        if (i < 6 ) {
            // SQR3
            dev(line)->SQR3 |= adc_config[i].chan << 5*i;
        } else
        if (i >= 6 && i < 12) {
            // SQR2
            dev(line)->SQR2 |= adc_config[i].chan << 5*(i-6);
        } else
        if (i >= 12 && i < 16) {
            //SQR1
            dev(line)->SQR1 |= adc_config[i].chan << 5*(i-12);
        }
    }
    // write ADC_NUMOF to SQR1
    dev(line)->SQR1 |= ADC_NUMOF << ADC_SQR1_L_Pos;
    // initiate DMA transfer
    dma_acquire(adc_dma_config[0].dma);
    dev(line)->CR2 |= ADC_CR2_DMA;
    dev(line)->CR1 |= ADC_CR1_SCAN;
    int ret = dma_configure(adc_dma_config[0].dma, adc_dma_config[0].dma_chan,
            &dev(line)->DR, data, len,
            DMA_PERIPH_TO_MEM, DMA_INC_DST_ADDR | DMA_DATA_WIDTH_HALF_WORD);
    (void)ret;
    dma_start(adc_dma_config[0].dma);
    dev(line)->CR2 |= ADC_CR2_SWSTART;
    dma_wait(adc_dma_config[0].dma);
    dma_stop(adc_dma_config[0].dma);

    dev(line)->CR1 &= ~ADC_CR1_SCAN;
    dev(line)->CR2 &= ~ADC_CR2_DMA;
    dma_release(adc_dma_config[0].dma);

    // de-init
    done(line);
}
#endif
