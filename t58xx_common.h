/** @file t58xx_common.h
 *
 * @brief Header for added AAD functionality for T58XX.
 *
 * @note Originally developed for the Zephyr platform.
 *
 * @copyright (c) 2023 Irnas.  All rights reserved.
 * @copyright (c) 2025 Ysard.  All rights reserved.
 */

#ifndef T58XX_COMMON_H
#define T58XX_COMMON_H

#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


#define T58XX_REG_AAD_MODE			 0x29
#define T58XX_REG_AAD_D_FLOOR_HI		 0x2A
#define T58XX_REG_AAD_D_FLOOR_LO		 0x2B
#define T58XX_REG_AAD_D_ALGO_SEL		 0x2D
#define T58XX_REG_AAD_D_REL_PULSE_MIN_LO	 0x2E
#define T58XX_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED 0x2F
#define T58XX_REG_AAD_D_ABS_PULSE_MIN_LO	 0x30
#define T58XX_REG_AAD_D_ABS_THR_LO		 0x31
#define T58XX_REG_AAD_D_ABS_THR_HI		 0x32
#define T58XX_REG_AAD_D_REL_THR			 0x33
#define T58XX_REG_AAD_A_LPF			 0x35
#define T58XX_REG_AAD_A_THR			 0x36

/**
 * @brief Mimic minimal Zephyr device structure
 */
struct device {
	// Address of device instance config information.
    const void *config;
	// Address of the device instance private data.
    void *data;
};

/**
 * @brief AAD modes
 *
 * This setting selects desired AAD mode.
 */
enum t58xx_aad_select {
	T58XX_AAD_SELECT_NONE = 0x00,
	T58XX_AAD_SELECT_D1 = 0x01,
	T58XX_AAD_SELECT_D2 = 0x02, /* We received info that this is supposed to be 0x0F? */
	T58XX_AAD_SELECT_A = 0x08
};

/**
 * @brief AAD A mode low pass filter
 *
 * This setting selects low pass filter for AAD A mode.
 */
enum t58xx_aad_a_lpf {
	T58XX_AAD_A_LPF_4_4kHz = 0x01,
	T58XX_AAD_A_LPF_2_0kHz = 0x02,
	T58XX_AAD_A_LPF_1_9kHz = 0x03,
	T58XX_AAD_A_LPF_1_8kHz = 0x04,
	T58XX_AAD_A_LPF_1_6kHz = 0x05,
	T58XX_AAD_A_LPF_1_3kHz = 0x06,
	T58XX_AAD_A_LPF_1_1kHz = 0x07
};

/**
 * @brief AAD A mode threshold
 *
 * This setting selects threshold for AAD A mode.
 * @note According to datasheet there are 8 possible values, but table in datasheet describes 9
 * values. it seems to be 2,5dB step for LSB but only every other value exist in table.
 */
enum t58xx_aad_a_thr {
	T58XX_AAD_A_THR_60dB = 0x00,
	T5848_AAD_A_THR_62_5dB = 0x01,
	T58XX_AAD_A_THR_65dB = 0x02,
	T5848_AAD_A_THR_67_5dB = 0x03,
	T58XX_AAD_A_THR_70dB = 0x04,
	T5848_AAD_A_THR_72_5dB = 0x05,
	T58XX_AAD_A_THR_75dB = 0x06,
	T5848_AAD_A_THR_77_5dB = 0x07,
	T58XX_AAD_A_THR_80dB = 0x08,
	T5848_AAD_A_THR_82_5dB = 0x09,
	T58XX_AAD_A_THR_85dB = 0x0A,
	T5848_AAD_A_THR_87_5dB = 0x0B,
	T58XX_AAD_A_THR_90dB = 0x0C,
	T5848_AAD_A_THR_92_5dB = 0x0D,
	T58XX_AAD_A_THR_95dB = 0x0E,
	T58XX_AAD_A_THR_97_5dB = 0x0F
};

/**
 * @brief AAD D threshold algorithms
 *
 * This setting selects desired threshold algorithms
 * (relative/absolute) for AAD D mode.
 * @note Datasheet claim that T5838 does not support this register
 *  but the values may work... Both algorithms seems to be enabled
 *  by default on T5838 & T5848.
 */
enum t58xx_aad_d_algo_select {
	T58XX_AAD_D_ALGO_SELECT_NONE = 0x00,
	T58XX_AAD_D_ALGO_SELECT_REL = 0x40,
	T58XX_AAD_D_ALGO_SELECT_ABS = 0x80,
	T58XX_AAD_D_ALGO_SELECT_BOTH = 0xC0
};

/**
 * @brief AAD D mode absolute threshold
 *
 * This setting selects absolute threshold for AAD D mode.
 * @note Datasheet claim values between 0x00F and 0x7BC are allowed, but only provides us with table
 * of discrete values defined in this enumerator.
 */
enum t58xx_aad_d_abs_thr {
	T58XX_AAD_D_ABS_THR_40dB = 0x000F,
	T58XX_AAD_D_ABS_THR_45dB = 0x0016,
	T58XX_AAD_D_ABS_THR_50dB = 0x0032,
	T58XX_AAD_D_ABS_THR_55dB = 0x0037,
	T58XX_AAD_D_ABS_THR_60dB = 0x005F,
	T58XX_AAD_D_ABS_THR_65dB = 0x00A0,
	T58XX_AAD_D_ABS_THR_70dB = 0x0113,
	T58XX_AAD_D_ABS_THR_75dB = 0x01E0,
	T58XX_AAD_D_ABS_THR_80dB = 0x0370,
	T58XX_AAD_D_ABS_THR_85dB = 0x062C,
	T58XX_AAD_D_ABS_THR_87dB = 0x07BC
};

/**
 * @brief AAD D mode relative threshold
 *
 * This setting selects relative threshold for AAD D mode.
 * @note Datasheet claim values between 0x24 and 0xFF are allowed, but only provides us with table
 * of discrete values defined in this enumerator.
 */
enum t58xx_aad_d_rel_thr {
	T58XX_AAD_D_REL_THR_3dB = 0x24,
	T58XX_AAD_D_REL_THR_6dB = 0x36,
	T58XX_AAD_D_REL_THR_9dB = 0x48,
	T58XX_AAD_D_REL_THR_12dB = 0x64,
	T58XX_AAD_D_REL_THR_15dB = 0x8F,
	T58XX_AAD_D_REL_THR_18dB = 0xCA,
	T58XX_AAD_D_REL_THR_20dB = 0xFF
};

/**
 * @brief AAD D mode floor
 *
 * This setting selects relative threshold floor for AAD D mode.
 * @note Datasheet claim values between 0x00F and 0x7BC are allowed, but only provides us with table
 * of discrete values defined in this enumerator.
 */
enum t58xx_aad_d_floor {
	T58XX_AAD_D_FLOOR_40dB = 0x000F,
	T58XX_AAD_D_FLOOR_45dB = 0x0016,
	T58XX_AAD_D_FLOOR_50dB = 0x0032,
	T58XX_AAD_D_FLOOR_55dB = 0x0037,
	T58XX_AAD_D_FLOOR_60dB = 0x005F,
	T58XX_AAD_D_FLOOR_65dB = 0x00A0,
	T58XX_AAD_D_FLOOR_70dB = 0x0113,
	T58XX_AAD_D_FLOOR_75dB = 0x01E0,
	T58XX_AAD_D_FLOOR_80dB = 0x0370,
	T58XX_AAD_D_FLOOR_85dB = 0x062C,
	T58XX_AAD_D_FLOOR_87dB = 0x07BC
};

/**
 * @brief AAD D mode relative pulse minimum
 *
 * This setting selects pulse minimum for AAD D mode relative threshold detection.
 * @note Datasheet claim values between 0x000 and 0x12C are allowed, but only provides us with table
 * of discrete values defined in this enumerator.
 */
enum t58xx_aad_d_rel_pulse_min {
	T58XX_AAD_D_REL_PULSE_MIN_0_7ms = 0x0000,
	T58XX_AAD_D_REL_PULSE_MIN_10ms = 0x0064,
	T58XX_AAD_D_REL_PULSE_MIN_19ms = 0x00C8,
	T58XX_AAD_D_REL_PULSE_MIN_29ms = 0x012C
};

/**
 * @brief AAD D mode absolute pulse minimum
 *
 * This setting selects pulse minimum for AAD D mode absolute threshold detection.
 * @note Datasheet claim values between 0x000 and 0x0DAC are allowed, but only provides us with
 * table of discrete values defined in this enumerator.
 */
enum t58xx_aad_d_abs_pulse_min {
	T58XX_AAD_D_ABS_PULSE_MIN_1_1ms = 0x0000,
	T58XX_AAD_D_ABS_PULSE_MIN_10ms = 0x0064,
	T58XX_AAD_D_ABS_PULSE_MIN_19ms = 0x00C8,
	T58XX_AAD_D_ABS_PULSE_MIN_29ms = 0x012C,
	T58XX_AAD_D_ABS_PULSE_MIN_48ms = 0x01F4,
	T58XX_AAD_D_ABS_PULSE_MIN_95ms = 0x03E8,
	T58XX_AAD_D_ABS_PULSE_MIN_188ms = 0x07D0,
	T58XX_AAD_D_ABS_PULSE_MIN_282ms = 0x0BB8,
	T58XX_AAD_D_ABS_PULSE_MIN_328ms = 0x0DAC
};

typedef void (*t58xx_wake_handler_t)(const struct device *dev);

/**
 * @brief AAD A configuration structure
 *
 * This structure holds full configuration of AAD A mode to be written to device.
 */
struct t58xx_aad_a_conf {
	enum t58xx_aad_a_lpf aad_a_lpf;
	enum t58xx_aad_a_thr aad_a_thr;
};

/**
 * @brief AAD D configuration structure
 *
 * This structure holds full configuration of AAD D1 and D2 modes to be written to device.
 */
struct t58xx_aad_d_conf {
	enum t58xx_aad_d_algo_select aad_d_algo_sel;
	enum t58xx_aad_d_floor aad_d_floor;
	enum t58xx_aad_d_rel_pulse_min aad_d_rel_pulse_min;
	enum t58xx_aad_d_abs_pulse_min aad_d_abs_pulse_min;
	enum t58xx_aad_d_abs_thr aad_d_abs_thr;
	enum t58xx_aad_d_rel_thr aad_d_rel_thr;
};

// struct t58xx_drv_data {
// 	struct onoff_manager *clk_mgr;
// 	struct onoff_client clk_cli;
// 	struct k_mem_slab *mem_slab;
// 	uint32_t block_size;
// 	struct k_msgq rx_queue;
// 	bool request_clock : 1;
// 	bool configured : 1;
// 	volatile bool active;
// 	volatile bool stopping;
//
// #ifdef CONFIG_T58XX_AAD_TRIGGER
// 	/* Pointer to child device for putting device back into low power after sampling */
// 	const struct device *aad_child_dev;
// #endif /* CONFIG_T58XX_AAD_TRIGGER */
// };

struct t58xx_drv_data {
	i2s_chan_handle_t rx_handle;
#ifdef CONFIG_T58XX_AAD_TRIGGER
	/* Pointer to child device for putting device back into low power after sampling */
	const struct device *aad_child_dev;
#endif /* CONFIG_T58XX_AAD_TRIGGER */
};

#ifdef CONFIG_T58XX_AAD_TRIGGER
struct t58xx_aad_drv_cfg {
	const struct device *i2s_dev;

    const gpio_num_t micen;
    bool micen_available; // true: support for load switch

    const gpio_num_t wake;
	bool wake_available; // false: manually handled outside the lib

    const gpio_num_t thsel;
    const gpio_num_t clk;
};

struct t58xx_aad_drv_data {
    bool aad_unlocked;
	enum t58xx_aad_select aad_enabled_mode;

    bool cb_configured;
    bool int_handled;
    t58xx_wake_handler_t wake_handler;

    const struct t58xx_aad_drv_cfg *aad_cfg;
};

/**
 * @brief get default AAD internal data structure
 * @note aad_cfg is populated in t58xx_aad_init()
 */
#define T58XX_AAD_DEFAULT_DATA() { \
    .aad_unlocked = false, \
    .aad_enabled_mode = T58XX_AAD_SELECT_NONE, \
    .cb_configured = false, \
    .int_handled = false, \
    .wake_handler = NULL, \
    .aad_cfg = NULL, \
}

#endif /* CONFIG_T58XX_AAD_TRIGGER */

#ifdef CONFIG_T58XX_AAD_TRIGGER
/**
 * @brief Set AAD wake pin interrupt handler function
 *
 * Will set handler function to be called when interrupt is triggered. We can set handler
 * at any time, but t58xx_aad_<*>_mode_set() must be called for interrupt triggering to be enabled.
 *
 * @note when interrupt gets triggered it will disable further interrupts until
 * t58xx_wake_clear() is called.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] handler Pointer to the handler function to be called when interrupt is triggered.
 */
void t58xx_aad_wake_handler_set(const struct device *dev, t58xx_wake_handler_t handler);

/**
 * @brief Clear AAD wake pin interrupt and re-enable AAD interrupts.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_aad_wake_clear(const struct device *dev);

/**
 * @brief Configure T58XX device into AAD A mode (not voice band)
 *
 * Will configure and run T58XX AAD and configure interrupts on wake pin.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] aadconf Pointer to the structure containing AAD A configuration
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_aad_a_mode_set(const struct device *dev, struct t58xx_aad_a_conf *aadconf);

/**
 * @brief Configure T58XX device into AAD D1 mode (voice band)
 *
 * Will configure and run T58XX AAD and configure interrupts on wake pin.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] aadconf Pointer to the structure containing AAD D configuration
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_aad_d1_mode_set(const struct device *dev, struct t58xx_aad_d_conf *aadconf);

/**
 * @brief Configure T58XX device into AAD D2 mode (voice band)
 *
 * Will configure and run T58XX AAD and configure interrupts on wake pin.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] aadconf Pointer to the structure containing AAD D configuration
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_aad_d2_mode_set(const struct device *dev, struct t58xx_aad_d_conf *aadconf);

/**
 * @brief Disable AAD functionality
 *
 * Will disable AAD functionality and disable interrupts on wake pin.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_aad_mode_disable(const struct device *dev);

/**
 * @brief Reset T58XX device using mic enable pin GPIO
 *
 * @note Do not use if mic enable pin is not configured.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code if othewise.
 */
esp_err_t t58xx_reset(const struct device *dev);

/**
 * @brief Put T58XX into sleep mode when AAD is enabled. Is called after
 * writing to AAD registers.
 *
 * @note Make sure that the interrupt has been rearmed via `t58xx_aad_wake_clear`
 *  if it is handled by the lib.
 *
 * @todo Should currently work for AAD A, AAD D2 modes which require a frequency
 *  between 50 kHz and 200 kHz for 2 ms followed by setting CLK = OFF.
 *  BUT, AAD D1 seems to need a constant CLK = 768 kHz that is not handled for now!
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 */
esp_err_t t58xx_aad_sleep(const struct device *dev);

/**
 * @brief Initialize T58XX AAD trigger device.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code if otherwise.
 */
esp_err_t t58xx_aad_init(const struct device *dev);

#endif /* CONFIG_T58XX_AAD_TRIGGER */

#ifdef __cplusplus
}
#endif

#endif /* T58XX_COMMON_H */
