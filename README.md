# nsx-audio

PDM audio capture for Ambiq Apollo5-family SoCs.

This module provides a clean, callback-driven PDM microphone interface with
DMA-backed double-buffered sampling. It is the NSX successor to the legacy
`ns-audio` module.

## Key changes from legacy `ns-audio`

- **PDM-only** — AUDADC is not present on Apollo5 parts and was removed.
- **No feature extraction** — MFCC / mel-spectrogram are a separate concern.
- **No prints in driver code** — errors are returned as status codes.
- **Fixed `pdm_deinit` handle bug** — legacy code passed the config struct
  instead of the PDM HAL handle; this is corrected.
- **Fixed frequency config logic** — the missing `else` that caused
  HFRC/HFRC2_ADJ settings to be silently overwritten is fixed.
- **32-byte DMA alignment enforced** at init time.

## Supported SoCs

| SoC | PDM |
|-----|-----|
| Apollo5B | Yes |
| Apollo510 | Yes |
| Apollo510B | Yes |

## Public API

```c
#include "nsx_audio.h"

uint32_t nsx_audio_init(nsx_audio_config_t *cfg);
uint32_t nsx_audio_start(nsx_audio_config_t *cfg);
uint32_t nsx_audio_stop(nsx_audio_config_t *cfg);
```
