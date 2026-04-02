/**
 * @file ns_pdm_apollo4.c
 * @brief PDM driver for Apollo4 family (Apollo4B, Apollo4P, Apollo4L).
 *
 * Ported from legacy ns-audio/src/apollo4/ns_pdm.c with the following fixes:
 *   - pdm_deinit passes the HAL handle, not the config struct (legacy bug)
 *   - No printf in driver code — errors returned as status codes
 *   - HFRC2_ADJ kick-start sequence with proper delays
 *   - Uses am_bsp_pdm_pins_enable() instead of manual GPIO config
 *   - PCM extraction: bits [23:8] from each 32-bit DMA word → int16_t
 *
 * Key differences from Apollo5:
 *   - No PLL clock source (Apollo4 has no SYSPLL for PDM)
 *   - HFRC2 calibration uses am_hal_clkgen_control, not am_hal_clkmgr
 *   - ISR uses am_hal_pdm_interrupt_service() + am_hal_pdm_dma_get_buffer()
 *     for hardware-managed DMA ping-pong (Apollo5 re-triggers manually)
 *   - Clock enum: AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ (not HFRC2_31MHZ)
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
 * Supports simultaneous operation of multiple PDM instances. */
#ifndef AM_PART_APOLLO4L
#define NSX_PDM_MAX_INSTANCES 4
#else
#define NSX_PDM_MAX_INSTANCES 1
#endif

static nsx_audio_config_t *g_pdm_cfgs[NSX_PDM_MAX_INSTANCES] = {0};

/* Per-instance persistent DMA transfer configs — needed by
 * am_hal_pdm_interrupt_service which manages ping-pong internally. */
static am_hal_pdm_transfer_t g_xfers[NSX_PDM_MAX_INSTANCES];

/* PDM IRQ table. */
#ifndef AM_PART_APOLLO4L
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
        /* Let the HAL manage the DMA ping-pong buffer swap. */
        am_hal_pdm_interrupt_service(cfg->_pdm_handle, status, &g_xfers[mic_idx]);

        /* Get pointer to the completed DMA buffer. */
        uint32_t *raw = (uint32_t *)am_hal_pdm_dma_get_buffer(cfg->_pdm_handle);
        uint32_t n = cfg->num_samples * cfg->num_channels;
        int16_t *pcm = cfg->pcm_buffer;

        /* Apollo4 PDM data: 24-bit sign-extended in 32-bit words.
         * Extract bits [23:8] as a 16-bit sample (discards LSB noise). */
        for (uint32_t i = 0; i < n; i++) {
            pcm[i] = (int16_t)((raw[i] >> 8) & 0xFFFF);
        }

        if (cfg->callback) {
            cfg->callback(cfg, pcm, n);
        }
    }
}

void am_pdm0_isr(void) { pdm_isr_handler(0); }
#ifndef AM_PART_APOLLO4L
void am_pdm1_isr(void) { pdm_isr_handler(1); }
void am_pdm2_isr(void) { pdm_isr_handler(2); }
void am_pdm3_isr(void) { pdm_isr_handler(3); }
#endif

/* ------------------------------------------------------------------ */
/* Default PDM config                                                  */
/* ------------------------------------------------------------------ */

const nsx_audio_pdm_config_t nsx_audio_pdm_default = {
    .clock        = NSX_AUDIO_CLK_HFRC2_ADJ,
    .clock_freq   = NSX_AUDIO_PDM_CLK_750KHZ,
    .mic          = NSX_AUDIO_PDM_MIC0,
    .sample_width = NSX_AUDIO_PDM_16BIT,
    .left_gain    = AM_HAL_PDM_GAIN_0DB,
    .right_gain   = AM_HAL_PDM_GAIN_0DB,
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
    if (((uintptr_t)cfg->dma_buffer & 0x1F) != 0) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->num_samples == 0) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->num_channels != 1 && cfg->num_channels != 2) {
        return NS_STATUS_INVALID_CONFIG;
    }

    /* Validate buffer sizes. */
    uint32_t samples_total = cfg->num_samples * cfg->num_channels;
    uint32_t dma_frame_bytes = samples_total * cfg->pdm.sample_width * 2;
    uint32_t pcm_frame_bytes = samples_total * sizeof(int16_t);
    if (cfg->dma_buffer_size < dma_frame_bytes * 2) {
        return NS_STATUS_INVALID_CONFIG;
    }
    if (cfg->pcm_buffer_size < pcm_frame_bytes) {
        return NS_STATUS_INVALID_CONFIG;
    }

    /* Apollo4 has no PLL for PDM. */
    if (cfg->pdm.clock == NSX_AUDIO_CLK_PLL) {
        return NS_STATUS_INVALID_CONFIG;
    }

    uint32_t mic_idx = (uint32_t)cfg->pdm.mic;
#ifdef AM_PART_APOLLO4L
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

    /* Clock source. */
    switch (cfg->pdm.clock) {
    case NSX_AUDIO_CLK_HFRC:
        pdm_hal.ePDMClkSpeed = AM_HAL_PDM_CLK_HFRC_24MHZ;
        pdm_hal.eClkDivider  = AM_HAL_PDM_MCLKDIV_1;
        break;
    case NSX_AUDIO_CLK_HFRC2_ADJ:
        pdm_hal.ePDMClkSpeed = AM_HAL_PDM_CLK_HFRC2ADJ_24_576MHZ;
        pdm_hal.eClkDivider  = AM_HAL_PDM_MCLKDIV_1;
        break;
    case NSX_AUDIO_CLK_HFXTAL:
        pdm_hal.ePDMClkSpeed = AM_HAL_PDM_CLK_HFXTAL;
        pdm_hal.eClkDivider  = AM_HAL_PDM_MCLKDIV_3;
        break;
    default:
        return NS_STATUS_INVALID_CONFIG;
    }

    /* Frequency / decimation config.
     * HFRC2_ADJ = 24.576 MHz, HFRC = 24 MHz.
     *   750 kHz path: CLKO_DIV15 (÷32) → 768 kHz / Dec24 = 32 kHz (HFRC2_ADJ)
     *                                     750 kHz / Dec24 ≈ 31.25 kHz (HFRC)
     *   1.5 MHz path: CLKO_DIV7 (÷16)  → 1536 kHz / Dec48 = 32 kHz (HFRC2_ADJ) */
    if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC2_ADJ ||
        cfg->pdm.clock == NSX_AUDIO_CLK_HFRC) {
        if (cfg->pdm.clock_freq == NSX_AUDIO_PDM_CLK_750KHZ) {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV15;
            pdm_hal.ui32DecimationRate = 24;
        } else {
            pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV7;
            pdm_hal.ui32DecimationRate = 48;
        }
    } else {
        /* HFXTAL */
        pdm_hal.ePDMAClkOutDivder  = AM_HAL_PDM_PDMA_CLKO_DIV14;
        pdm_hal.ui32DecimationRate = 24;
    }

    /* Channel config. */
    pdm_hal.ePCMChannels = (cfg->num_channels == 2)
                               ? AM_HAL_PDM_CHANNEL_STEREO
                               : AM_HAL_PDM_CHANNEL_LEFT;

    /* ---- Initialize PDM hardware ---- */
    uint32_t rc = am_hal_pdm_initialize(mic_idx, &cfg->_pdm_handle);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }
    rc = am_hal_pdm_power_control(cfg->_pdm_handle, AM_HAL_PDM_POWER_ON, false);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }

    /* ---- Clock source calibration ---- */
#if defined(AM_PART_APOLLO4B) || defined(AM_PART_APOLLO4P)
    if (cfg->pdm.clock == NSX_AUDIO_CLK_HFXTAL) {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);
    } else if (cfg->pdm.clock == NSX_AUDIO_CLK_HFRC2_ADJ) {
        am_hal_mcuctrl_control(AM_HAL_MCUCTRL_CONTROL_EXTCLK32M_KICK_START, false);
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HFRC2_START, false);
        am_util_delay_us(200);   /* wait for FLL to lock */
        am_hal_clkgen_control(AM_HAL_CLKGEN_CONTROL_HF2ADJ_ENABLE, false);
        am_util_delay_us(500);   /* wait for adjustment to apply */
    }
#endif

    rc = am_hal_pdm_configure(cfg->_pdm_handle, &pdm_hal);
    if (rc != AM_HAL_STATUS_SUCCESS) {
        return NS_STATUS_INIT_FAILED;
    }

    /* Configure microphone pins via BSP. */
    am_bsp_pdm_pins_enable(mic_idx);

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

    /* Set up the persistent DMA transfer config for interrupt_service. */
    uint32_t mic_idx = (uint32_t)cfg->pdm.mic;
    uint32_t n = cfg->num_samples * cfg->num_channels;
    g_xfers[mic_idx].ui32TotalCount      = n * cfg->pdm.sample_width * 2;
    g_xfers[mic_idx].ui32TargetAddr      = (uint32_t)(((uintptr_t)cfg->dma_buffer + 3) & ~0xFU);
    g_xfers[mic_idx].ui32TargetAddrReverse = g_xfers[mic_idx].ui32TargetAddr
                                              + g_xfers[mic_idx].ui32TotalCount;

    am_hal_pdm_dma_start(cfg->_pdm_handle, &g_xfers[mic_idx]);

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
