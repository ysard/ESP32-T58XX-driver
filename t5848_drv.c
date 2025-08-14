/** @file t5848_drv.c
 *
 * @brief Functions related to the configuration of the I2S driver
 *  for the T5848 MEMS microphone by TDK.
 *
 * @copyright (c) 2025 Ysard. All rights reserved.
 */

#include "esp_log.h"
#include "t5848.h"

static const char *TAG = "T5848_I2S";


esp_err_t i2s_init(const i2s_std_config_t *std_rx_cfg, i2s_chan_config_t *chan_cfg, i2s_chan_handle_t *rx_handle) {
    esp_err_t err;
    i2s_chan_config_t *l_chan_cfg;

    /* Allocate an I2S RX channel */
    if (chan_cfg == NULL) {
        /* With I2S_NUM_AUTO rx channel will be registered on another I2S,
         * if no other available I2S unit found it will return ESP_ERR_NOT_FOUND */
        l_chan_cfg = &(i2s_chan_config_t) I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    } else {
        l_chan_cfg = chan_cfg;
    }
    err = i2s_new_channel(l_chan_cfg, NULL, rx_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed: %d", err);
        return err;
    }

    /* Init the channel into I2S RX mode */
    err = i2s_channel_init_std_mode(*rx_handle, std_rx_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %d", err);
        return err;
    }
}

#ifdef CONFIG_T58XX_AAD_TRIGGER

esp_err_t t5848_init(const i2s_std_config_t *std_rx_cfg, i2s_chan_config_t *chan_cfg, struct t58xx_aad_drv_cfg *t58xx_aad_cfg,
                     struct device *aad_dev, i2s_chan_handle_t *rx_handle) {
    esp_err_t err = i2s_init(std_rx_cfg, chan_cfg, rx_handle);
    if (err != ESP_OK)
        return err;

    /* Build the device structure for AAD configuration */
    struct t58xx_drv_data t58xx_data = {
        .rx_handle = *rx_handle
    };
    struct device t58xx_std_dev = {
        .config = std_rx_cfg,
        .data = &t58xx_data
    };

    /* Inject I2S device in the AAD config */
    // (Used later to test if the I2S clock is released,
    // and ready to be used for AAD config through THSEL pin)
    t58xx_aad_cfg->i2s_dev = &t58xx_std_dev;

    struct t58xx_aad_drv_data t58xx_aad_data;

    *aad_dev = (struct device) {
        .config = t58xx_aad_cfg,
        .data = &t58xx_aad_data
    };
    return ESP_OK;
}

#endif /* CONFIG_T58XX_AAD_TRIGGER */
