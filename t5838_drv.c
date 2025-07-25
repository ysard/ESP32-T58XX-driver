/** @file t5838_drv.c
 *
 * @brief Functions related to the configuration of the I2S driver
 *  for the T5838 MEMS microphone by TDK.
 *
 * @copyright (c) 2025 Ysard. All rights reserved.
 */

#include "t5838.h"

static const char *TAG = "T5838_I2S";

/**
 * @brief Init the T5838 PDM device & prepare AAD settings.
 *
 * @param[in] pdm_rx_cfg I2S PDM configuration.
 * @param[in] chan_cfg Pointer to channel configuration, can be left to NULL automatically select the I2S port.
 * @param[in] t5838_aad_cfg Pointer to AAD pinout configuration.
 * @param[out] dev Pointer to device structure which will be populated, ready to be used for AAD configurations.
 * @param[out] rx_handle Pointer to the channel handle, which will be populated.
 */
esp_err_t t5838_init(const i2s_pdm_rx_config_t *pdm_rx_cfg, i2s_chan_config_t *chan_cfg, struct t5838_aad_drv_cfg *t5838_aad_cfg,
                     struct device *dev, i2s_chan_handle_t *rx_handle) {
    esp_err_t err;

    /* Allocate an I2S RX channel */
    if (chan_cfg == NULL) {
        /* With I2S_NUM_AUTO rx channel will be registered on another I2S,
         * if no other available I2S unit found it will return ESP_ERR_NOT_FOUND */
        *chan_cfg = (i2s_chan_config_t) I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    }
    err = i2s_new_channel(chan_cfg, NULL, rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %d", err);
        return err;
    }

    /* Init the channel into PDM RX mode */
    err = i2s_channel_init_pdm_rx_mode(*rx_handle, pdm_rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_pdm_rx_mode failed: %d", err);
        return err;
    }

    /* Build the device structure for AAD configuration */
    struct t5838_drv_data t5838_data = {
        .rx_handle = *rx_handle
    };
    struct device t5838_pdm_dev = {
        .config = pdm_rx_cfg,
        .data = &t5838_data
    };

    /* Inject PDM device in the AAD config */
    // (Used later to test if the PDM clock is released,
    // and ready to be used for AAD config through THSEL pin)
    t5838_aad_cfg->pdm_dev = &t5838_pdm_dev;

    struct t5838_aad_drv_data t5838_aad_data;

    *dev = (struct device) {
        .config = t5838_aad_cfg,
        .data = &t5838_aad_data
    };
    return ESP_OK;
}
