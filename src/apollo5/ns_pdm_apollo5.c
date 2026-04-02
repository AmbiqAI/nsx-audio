/**
 * @file ns_pdm_apollo5.c
 * @brief PDM driver for Apollo5 family (Apollo5B, Apollo510, Apollo510B).
 *
 * Migrated from legacy ns-audio/src/apollo5/ns_pdm.c with the following fixes:
 *   - pdm_deinit passes the HAL handle, not the config struct
 *   - frequency config uses proper if/else-if chain (no fall-through)
 *   - 32-byte DMA alignment enforced
 *   - no printf in driver code — errors returned as status codes
 */

#include "nsx_audio.h"
#include "am_bsp.h"
#include "am_mcu_apollo.h"
#include "am_util.h"
#include "ns_core.h"

/* ------------------------------------------------------------------ */
/* Internal state                                                      */
/* ------------------------------------------------------------------ */

/* Per-instance config pointers — indexed by PDM module (mic index).
 * Supports simultaneous operation of multiple PDM instances for
 * multi-mic configurations. */
#ifdef NS_PDM1TO3_PRESENT
#define NSX_PDM_MAX_INSTANCES 4
#else
#define NSX_PDM_MAX_INSTANCES 1
#endif

static nsx_audio_config_t *g_pdm_cfgs[NSX_PDM_MAX_INSTANCES] = {0};

/* PDM IRQ table — PDM0 always present, PDM1-3 gated by NS_PDM1TO3_PRESENT. */
#ifdef NS_PDM1TO3_PRESENT
static const IRQn_Type g_pdm_irqs[] = {PDM0_IRQn, PDM1_IRQn, PDM2_IRQn, PDM3_IRQn};
#else
static const IRQn_Type g_pdm_irqs[] = {PDM0_IRQn};
#endif

/* ------------------------------------------------------------------ */
/* ISR                                                                 */
/* ------------------------------------------------------------------ */

static void pdm_isr_handler(uint32_t mic_idx) {
    uint32_t status;
    if (mic_idx >= NSX_PDM_MAX_INSTANCES) return;
    nsx_audio_config_t *cfg = g_pdm_cfgs[mic_idx];
    if (cfg == NULL || cfg->_pdm_handle == NULL) {
        return;
    }

    am_hal_pdm_interrupt_status_get(cfg->_pdm_handle, &status, true);
    am_hal_pdm_interrupt_clear(cfg->_pdm_handle, status);

    if (status & AM_HAL_PDM_INT_DCMP) {
        /* DMA transfer complete — extract PCM samples. */
        uint32_t *raw = (uint32_t *)((uintptr_t)cfg->dma_buffer & ~0xFU);
        uint32_t n = cfg->num_samples * cfg->num_channels;
        int16_t *pcm = cfg->pcm_buffer;

        if (cfg->pdm.sample_width == NSX_AUDIO_PDM_16BIT) {
            for (uint32_t i = 0; i < n; i++) {
                pcm[i] = (int16_t)(raw[i] & 0xFFFF);
            }
        } else {
            /* 24-bit: take upper 16 bits */
            for (uint32_t i = 0; i < n; i++) {
                pcm[i] = (int16_t)((raw[i] >> 8) & 0xFFFF);
            }
        }

        if (cfg->callback) {
            cfg->callback(cfg, pcm, n);
        }

        /* Re-trigger DMA for continuous capture. */
        am_hal_pdm_dma_start(cfg->_pdm_handle, &(am_hal_pdm_transfer_t){
            .ui32TotalCount = n * cfg->pdm.sample_width * 2,
            .ui32TargetAddr = (uint32_t)(((uintptr_t)cfg->dma_buffer + 3) & ~0xFU),
            .ui32TargetAddrReverse = (uint32_t)(((uintptr_t)cfg->dma_buffer + 3) & ~0xFU)
                                     + n * cfg->pdm.sample_width * 2,
        });
    }
}

void am_pdm0_isr(void) { pdm_isr_handler(0); }
#ifdef NS_PDM1TO3_PRESENT
void am_pdm1_isr(void) { pdm_isr_handler(1); }
void am_pdm2_isr(void) { pdm_isr_handler(2); }
void am_pdm3_isr(void) { pdm_isr_handler(3); }
#endif

/* ------------------------------------------------------------------ */
/* Default PDM config                                                  */
/* ------------------------------------------------------------------ */

const nsx_audio_pdm_config_t nsx_audio_pdm_default = {
    /* HFRC2_ADJ: calibrates HFRC2 to 196.608 MHz, so HFRC2/8 = 24.576 MHz.
     * With CLKO_DIV15 + DecimationRate=48 this gives exactly 16 kHz.
     * (PLL path calls am_hal_clkmgr_clock_config(SYSPLL) which blocks
     * indefinitely if crystal is not available.) */
    .clock        = NSX_AUDIO_CLK_HFRC2_ADJ,
    .clock_freq   = NSX_AUDIO_PDM_CLK_750KHZ,
    .mic          = NSX_AUDIO_PDM_MIC0,
    .sample_width = NSX_AUDIO_PDM_16BIT,
    .left_gain    = AM_HAL_PDM_GAIN_P180DB,  /* +18 dB — better sensitivity */
    .right_gain   = AM_HAL_PDM_GAIN_P180DB,
};

/* ------------------------------------------------------------------ */
/* Init / Start / Stop                                                 */
/* ------------------------------------------------------------------ */

uint32_t nsx_audio_init(nsx_audio_config_t *cfg) {
    if (cfg == NULL) {
        return NS_STATUS_INVALID_HANDLE;
    }
    if (cfg->source != NSX_AUDIO_SOURCE_PDM) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->callback == NULL) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->dma_buffer == NULL || cfg->pcm_buffer == NULL) {
        return NS_STATUS_INVALID_CONFIG;
    }
    /* Enforce 32-byte DMA alignment. */
    if (((uintptr_t)cfg->dma_buffer & 0x1F) != 0) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->num_samples == 0) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->num_channels != 1 && cfg->num_channels != 2) {
        return NS_STATUS_INVALID_CONFIG;
    }

    /* Validate buffer sizes.
     * DMA buffer must hold 2× (double-buffered) the sample frame.
     * PCM buffer must hold the deinterleaved samples. */
    uint32_t samples_total = cfg->num_samples * cfg->num_channels;
    uint32_t dma_frame_bytes = samples_total * cfg->pdm.sample_width * 2;
    uint32_t pcm_frame_bytes = samples_total * sizeof(int16_t);
    if (cfg->dma_buffer_size < dma_frame_bytes * 2) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->pcm_buffer_size < pcm_frame_bytes) {
        return NS_STATUS_INVALID_CONFIG;
    }

    uint32_t mic_idx = (uint32_t)cfg->pdm.mic;
#ifndef NS_PDM1TO3_PRESENT
    if (mic_idx > 0) {
        return NS_STATUS_INVALID_CONFIG;
    }
#endif

    /* ---- Build HAL config ---- */
    am_hal_pdm_config_t pdm_hal = {
        .eStepSize          = AM_HAL_PDM_GAIN_STEP_0_13DB,
        .bHighPassEnable    = AM_HAL_PDM_HIGH_PASS_ENABLE,
        .ui32HighPassCutoff = 0x3,
        .eLeftGain          = (am_hal_pdm_gain_e)cfg->pdm.left_gain,
        .eRightGain         = (am_hal_pdm_gain_e)cfg->pdm.right_gain,
        .bDataPacking       = 1,
        .bPDMSampleDelay    = AM_HAL_PDM_CLKOUT_PHSDLY_NONE,
        .ui32GainChangeDelay = AM_HAL_PDM_CLKOUT_DELAY_NONE,
        .bSoftMute          = 0,
        .bLRSwap            = 0,
    };

    /* Clock source */
    switch (cfg->pdm.clock) {
    case NSX_AUDIO_CLK_PLL:
        pdm_hal.ePDMClkSpeed  = AM_HAL_PDM_CLK_PLL;
        pdm_hal.eClkDivider   = AM_HAL_PDM_MCLKDIV_1;
        break;
    case NSX_AUDIO_CLK_HFRC:
        pdm_hal.ePDMClkSpeed  = AM_HAL_PDM_CLK_HFRC_24MHZ;
        pdm_hal.eClkDivider   = AM_HAL_PDM_MCLKDIV_1;
        break;
    case NSX_AUDIO_CLK_HFRC2_ADJ:
#if defined(AM_PART_APOLLO510L) || defined(AM_PART_APOLLO330P)
        return NS_STATUS_INVALID_CONFIG;  /* not supported on these parts */
#else
        pdm_hal.ePDMClkSpeed  = AM_HAL_PDM_CLK_HFRC2_31MHZ;
        pdm_hal.eClkDivider   = AM_HAL_PDM_MCLKDIV_1;
#endif
        break;
    case NSX_AUDIO_CLK_HFXTAL:
        pdm_hal.ePDMClkSpeed  = AM_HAL_PDM_CLK_HFXTAL;
        pdm_hal.eClkDivider   = AM_HAL_PDM_MCLKDIV_3;
        break;
    default:
        return NS_STATUS_INVALID_CONFIG;
    }

    /* Frequency config — FIXED: proper if/else-if chain (legacy had missing else).
     * HFRC2_ADJ on Apollo510: after calibrating HFRC2 to 196.608 MHz,
     * HFRC2/8 = 24.576 MHz. With CLKO_DIV15 (÷32) + DecimationRate=48:
     *   24.576 MHz / 32 / 48 = 16 000 Hz exactly.
     * HFRC clocks run at 24 MHz; CLKO_DIV15 + Dec=24 gives the same
     * 24 MHz / 32 / 24 = 31 250 Hz — but user typically selects HFRC2_ADJ
     * for 16 kHz target.  The 1.5 MHz path doubles the PDM clock. */
    if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC2_ADJ) {
        if (cfg->pdm.clock_freq == NSX_AUDIO_PDM_CLK_750KHZ) {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV15;
            pdm_hal.ui32DecimationRate = 48;   /* 24.576 MHz / 32 / 48 = 16 000 Hz */
        } else {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV7;
            pdm_hal.ui32DecimationRate = 96;
        }
    } else if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC) {
        if (cfg->pdm.clock_freq == NSX_AUDIO_PDM_CLK_750KHZ) {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV15;
            pdm_hal.ui32DecimationRate = 24;
        } else {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV7;
            pdm_hal.ui32DecimationRate = 48;
        }
    } else if (cfg->pdm.clock == NSX_AUDIO_CLK_PLL) {
        pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV3;
        pdm_hal.ui32DecimationRate = 48;
    } else {
        pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV14;
        pdm_hal.ui32DecimationRate = 24;
    }

    /* Channel config */
    pdm_hal.ePCMChannels = (cfg->num_channels == 2)
                               ? AM_HAL_PDM_CHANNEL_STEREO
                               : AM_HAL_PDM_CHANNEL_LEFT;

    /* ---- Clock source setup ---- */
    if (cfg->pdm.clock == NSX_AUDIO_CLK_PLL) {
#ifdef AM_PART_APOLLO510
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_SYSPLL, 12288000, NULL);
#else
        am_hal_clkmgr_syspll_fref_priority_t prio = {
            .high = AM_HAL_SYSPLL_FREFSEL_XTAL48MHz,
            .mid  = AM_HAL_SYSPLL_FREFSEL_EXTREFCLK,
            .low  = AM_HAL_SYSPLL_FREFSEL_HFRC192DIV4,
        };
        am_hal_clkmgr_control(AM_HAL_CLKMGR_SYPLL_FREF_PRIORITY_SET, (void *)&prio);
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLVCO, 245760000, NULL);
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_PLLPOSTDIV, 24576000, NULL);
#endif
    } else if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC) {
#if defined(AM_PART_APOLLO5A)
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);
        am_util_delay_us(1500);
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE, false);
        am_util_delay_us(500);
#elif defined(AM_PART_APOLLO5B)
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2,
                                   AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, NULL);
#endif
    } else if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC2_ADJ) {
        /* Calibrate HFRC2 to exactly 196.608 MHz.  HFRC2/8 then yields
         * 24.576 MHz — an exact audio-standard frequency.  With the
         * 750 kHz divider chain (CLKO_DIV15 + DecimationRate=48) this
         * produces exactly 16 000 Hz sample rate:
         *   24.576 MHz / 32 / 48 = 16 000 Hz.
         *
         * NOTE: am_hal_clkmgr_clock_config() blocks until the FLL locks.
         * This was previously omitted because it was confused with the
         * SYSPLL hang (which is a separate issue).
         */
#if defined(AM_PART_APOLLO510) || defined(AM_PART_APOLLO5B)
        am_hal_clkmgr_clock_config(AM_HAL_CLKMGR_CLK_ID_HFRC2,
                                   AM_HAL_CLKMGR_HFRC2_FREQ_ADJ_196P608MHZ, NULL);
#elif defined(AM_PART_APOLLO5A)
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);
        am_util_delay_us(1500);
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE, false);
        am_util_delay_us(500);
#endif
    }

    /* ---- Initialize PDM hardware ---- */
    am_bsp_pdm_pins_enable(mic_idx);
    uint32_t rc = am_hal_pdm_initialize(mic_idx, &cfg->_pdm_handle);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }
    rc = am_hal_pdm_power_control(cfg->_pdm_handle, AM_HAL_PDM_POWER_ON, false);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }
    rc = am_hal_pdm_configure(cfg->_pdm_handle, &pdm_hal);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }

    am_hal_pdm_fifo_flush(cfg->_pdm_handle);
    am_hal_pdm_fifo_threshold_setup(cfg->_pdm_handle, 16);

    /* Enable DMA-complete interrupt. */
    am_hal_pdm_interrupt_enable(cfg->_pdm_handle,
                                AM_HAL_PDM_INT_DERR | AM_HAL_PDM_INT_DCMP |
                                AM_HAL_PDM_INT_UNDFL | AM_HAL_PDM_INT_OVF);
    NVIC_SetPriority(g_pdm_irqs[mic_idx], AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(g_pdm_irqs[mic_idx]);

    g_pdm_cfgs[mic_idx] = cfg;
    cfg->_started = 0;

    return NS_STATUS_SUCCESS;
}

uint32_t nsx_audio_start(nsx_audio_config_t *cfg) {
    if (cfg == NULL || cfg->_pdm_handle == NULL) {
        return NS_STATUS_INVALID_HANDLE;
    }

    am_hal_pdm_enable(cfg->_pdm_handle);

    /* Trigger the first DMA transfer. */
    uint32_t n = cfg->num_samples * cfg->num_channels;
    am_hal_pdm_transfer_t xfer = {
        .ui32TotalCount      = n * cfg->pdm.sample_width * 2,
        .ui32TargetAddr      = (uint32_t)(((uintptr_t)cfg->dma_buffer + 3) & ~0xFU),
        .ui32TargetAddrReverse = (uint32_t)(((uintptr_t)cfg->dma_buffer + 3) & ~0xFU)
                                 + n * cfg->pdm.sample_width * 2,
    };
    am_hal_pdm_dma_start(cfg->_pdm_handle, &xfer);

    cfg->_started = 1;
    return NS_STATUS_SUCCESS;
}

uint32_t nsx_audio_stop(nsx_audio_config_t *cfg) {
    if (cfg == NULL || cfg->_pdm_handle == NULL) {
        return NS_STATUS_INVALID_HANDLE;
    }

    uint32_t mic_idx = (uint32_t)cfg->pdm.mic;
    NVIC_DisableIRQ(g_pdm_irqs[mic_idx]);

    /* FIXED: pass the HAL handle, not the config struct (legacy bug). */
    am_hal_pdm_interrupt_clear(cfg->_pdm_handle, 0xFFFFFFFF);
    am_hal_pdm_interrupt_disable(cfg->_pdm_handle, 0xFFFFFFFF);
    am_hal_pdm_disable(cfg->_pdm_handle);
    am_hal_pdm_power_control(cfg->_pdm_handle, AM_HAL_PDM_POWER_OFF, false);
    am_hal_pdm_deinitialize(cfg->_pdm_handle);
    am_bsp_pdm_pins_disable(mic_idx);

    cfg->_pdm_handle = NULL;
    cfg->_started = 0;
    if (mic_idx < NSX_PDM_MAX_INSTANCES) {
        g_pdm_cfgs[mic_idx] = NULL;
    }

    return NS_STATUS_SUCCESS;
}
