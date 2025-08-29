# TDK T5838 & T5848 drivers for ESP32

An ESP-IDF/PlatformIO I2S driver for the T5838 & T5848 MEMS microphones by TDK.

This is a port of the AAD driver (see below) originally developed for the
[Zephyr platform](https://github.com/IRNAS/irnas-t5838-driver).

The T5838 is a PDM microphone, whereas the T5848 is a I2S microphone.
Both are supported by the I2S (Inter-IC Sound) bus of the ESP32 chips.

Apart from this detail, the two devices are similar and feature Acoustic Activity Detect
(AAD) functionality, allowing them to remain powered on in low-power mode while being able
to wake up in response to an acoustic signal.

The activation and configuration for all AAD Modes is carried out via a one wire
write on the THSEL pin that reuses the CLK (T5838) / SCK (T5848) pin.
This library is essentially about this one wire custom protocol.

## ⚠️ Disclaimer ⚠️

At this time, the library has not yet been thoroughly tested and may contain errors.
Please report them by opening an issue or a PR. <3

## Configuration

Tested with ESP-IDF framework version of PlatformIO (@ 3.50401.0 (5.4.1)).

```bash
$ idf.py menuconfig
# or
$ pio run -t menuconfig
```

AAD setting is reachable here (enabled by default):
`Component config / T58XX Digital Microphone / Enable AAD functionality`


## Quick examples

### T5838 with AAD

```c
#include "t5838.h"
#include "driver/i2s_pdm.h"

#define GPIO_MIC_DATA    3
#define GPIO_MIC_WAKE    4
#define GPIO_MIC_THSEL   5
#define GPIO_MIC_PDMCLK  6 // Also used as the clock pin for the AAD config

/* Sample rates (with a downsampling setting I2S_PDM_DSR_8S):
 * - Low power mode: 6250-12500 Hz (CLK 400-800 kHz)
 * - High quality mode: 31250-57812 Hz (CLK 2-3.7 MHz) */
#define SAMPLE_RATE      12000 // in Hz

struct device t5838_aad_dev;
i2s_chan_handle_t rx_handle;

/**
 * @brief Configure the I2S port + AAD profile of the microphone
 * After this function the port is in READY state.
 */
esp_err_t configure_mic() {
    esp_err_t err;

    /* Init the channel into PDM RX mode */
    i2s_pdm_rx_config_t pdm_rx_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        // Note: for IDF 5.5, use I2S_PDM_RX_SLOT_PCM_FMT_DEFAULT_CONFIG macro for I2S0 port
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, // Data bit width only supports 16 bits
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .clk = GPIO_MIC_PDMCLK,
            .din = GPIO_MIC_DATA,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };

    struct t58xx_aad_drv_cfg t58xx_aad_cfg = {
        // Pin used to turn on/off the mic circuit power (load switch, etc.)
        .micen_available = false,
        // Manage the WAKE signal externally with a custom interrupt/ISR
        .wake            = NULL,
        .wake_available  = false,
        .thsel           = GPIO_MIC_THSEL,
        .clk             = GPIO_MIC_PDMCLK
    };

    err = t5838_init(&pdm_rx_cfg, NULL, &t58xx_aad_cfg, &t5838_aad_dev, &rx_handle);
    if (err != ESP_OK)
        return err;

    // Config aad mode
    err = t58xx_aad_init(&t5838_aad_dev);
    if (err != ESP_OK)
        return err;

    struct t58xx_aad_a_conf aad_a_conf = {
        .aad_a_lpf = T58XX_AAD_A_LPF_1_8kHz,
        .aad_a_thr = T58XX_AAD_A_THR_60dB
    };
    err = t58xx_aad_a_mode_set(&t5838_aad_dev, &aad_a_conf);
    if (err != ESP_OK)
        return err;

    return ESP_OK;
}
```

### T5848 without AAD

```c
#include "t5848.h"
#include "driver/i2s_std.h"

#define GPIO_MIC_SD      3
#define GPIO_MIC_WAKE    4
#define GPIO_MIC_THSEL   5
#define GPIO_MIC_SCK     6
#define GPIO_MIC_WS      7
#define SAMPLE_RATE      12000 // in Hz

i2s_chan_handle_t rx_handle;

/**
 * @brief Configure the I2S port without AAD profile of the microphone
 * After this function the port is in READY state.
 */
esp_err_t mic_configure() {
    esp_err_t err;

    /* Init the channel into STD RX mode */
    i2s_std_config_t std_rx_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, // 8/16/24/32-bit width sample data
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = GPIO_MIC_SCK,
            .ws = GPIO_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din = GPIO_MIC_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = t5848_init(&std_rx_cfg, NULL, &rx_handle);
    if (err != ESP_OK)
        return err;

    return ESP_OK;
}
```

# License

This program is licensed under the Apache 2.0 License.

Copyright (c) 2023 Irnas<br>
Copyright (c) 2025 Ysard
