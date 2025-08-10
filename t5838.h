/** @file t5838.h
 *
 * @brief Functions related to the configuration of the I2S driver
 *  for the T5838 MEMS microphone by TDK.
 *
 * @copyright (c) 2025 Ysard.  All rights reserved.
 */

#ifndef T5838_H
#define T5838_H

#include "driver/i2s_types.h"
#include "driver/i2s_pdm.h"

#include "t58xx_common.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t pdm_init(const i2s_pdm_rx_config_t *pdm_rx_cfg, i2s_chan_config_t *chan_cfg, i2s_chan_handle_t *rx_handle);

#ifdef CONFIG_T58XX_AAD_TRIGGER

/**
 * @brief Init the T5838 PDM device & prepare AAD settings.
 *
 * @param[in] pdm_rx_cfg I2S PDM configuration.
 * @param[in] chan_cfg Pointer to channel configuration, can be left to NULL to
 *  automatically select the I2S port.
 * @param[in] t58xx_aad_cfg Pointer to AAD pinout configuration.
 * @param[out] aad_dev Pointer to device structure which will be populated,
 *  ready to be used for AAD configurations.
 * @param[out] rx_handle Pointer to the channel handle, which will be populated.
 */
esp_err_t t5838_init(const i2s_pdm_rx_config_t *pdm_rx_cfg, i2s_chan_config_t *chan_cfg, struct t58xx_aad_drv_cfg *t58xx_aad_cfg, struct device *aad_dev, i2s_chan_handle_t *rx_handle);

#else

#define t5838_init pdm_init

#endif /* CONFIG_T58XX_AAD_TRIGGER */

#ifdef __cplusplus
}
#endif

#endif /* T5838_H */
