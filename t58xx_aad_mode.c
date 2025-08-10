/** @file t58xx_aad_mode.c
 *
 * @brief Functions related to the configuration of the AAD modes
 *  of the T58XX MEMS microphone by TDK.
 *
 * @copyright (c) 2023 Irnas. All rights reserved.
 * @copyright (c) 2025 Ysard. All rights reserved.
 */

#include "esp_log.h"
#include <rom/ets_sys.h> // ets_delay_us()
// Workaround to access to private enum i2s_state_t and struct i2s_channel_obj_t definitions.
// We use i2s_channel_obj_t only to test the `state` member of the i2s handle (value I2S_CHAN_STATE_RUNNING).
#include "../i2s_private.h"

#include "t58xx_common.h"

/* Although datasheet mentions that t58xx uses One Wire Protocol for configuration of AAD
 * functionality, it is really using two wires (THSEL and PDMCLK) for this, similar to the I2C
 * protocol. That's why FAKE2C names was used to describe this protocol. */
#define T58XX_FAKE2C_START_PILOT_CLKS 10
#define T58XX_FAKE2C_ZERO	      1 * T58XX_FAKE2C_START_PILOT_CLKS
#define T58XX_FAKE2C_ONE	      3 * T58XX_FAKE2C_START_PILOT_CLKS
#define T58XX_FAKE2C_STOP	      130 /* according to datasheet >128clk cycles */
#define T58XX_FAKE2C_SPACE	      1 * T58XX_FAKE2C_START_PILOT_CLKS

#define T58XX_FAKE2C_POST_WRITE_CYCLES 60 /* According to datasheet >50clk cycles */
#define T58XX_FAKE2C_PRE_WRITE_CYCLES  60 /* According to datasheet >50clk cycles */
#define T58XX_FAKE2C_DEVICE_ADDRESS    0x53

#define T58XX_FAKE2C_CLK_PERIOD_US 10 /* approx 100kHz */

/* >2ms of clock is required before entering sleep with AAD  */
#define T58XX_ENTER_SLEEP_MODE_CLOCKING_TIME_US 2500
#define T58XX_ENTER_SLEEP_MODE_CLK_PERIOD_US	10 /* approx 100kHz */

#define T58XX_RESET_TIME_MS 10 /* Time required for device to reset when power is disabled*/

static const char *TAG = "T58XX";

struct t58xx_address_data_pair {
	uint8_t address;
	uint8_t data;
};

/**
 * @brief drive clock pin in bitbang fashion.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] cycles Number of cycles to clock.
 * @param[in] period Period of clock in microseconds.
 */
void IRAM_ATTR prv_clock_bitbang(const struct device *dev, uint16_t cycles, uint16_t period)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	for (int i = 0; i < cycles; i++) {
		gpio_set_level(drv_cfg->pdmclk, 1);
		ets_delay_us(period / 2);
		gpio_set_level(drv_cfg->pdmclk, 0);
		ets_delay_us(period / 2);
	}
}

/**
 * @brief Function for writing to T58XX registers using their proprietary "one wire" that isn't
 * really one wire. We refer to this protocol as fake2c to avoid confusion with actual one wire
 * protocol.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] reg Register address to write to.
 * @param[in] data Data to write to register.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
// TODO: disable interrupts ?
esp_err_t IRAM_ATTR prv_reg_write(const struct device *dev, uint8_t reg, uint8_t data)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	struct t58xx_drv_data *pdm_data = drv_cfg->i2s_dev->data;
	if (pdm_data->rx_handle->state == I2S_CHAN_STATE_RUNNING) {
		ESP_LOGE(TAG, "Cannot write to device while pdm is active");
		return ESP_ERR_INVALID_STATE;
	}

	ESP_LOGD(TAG, "prv_reg_write, reg: 0x%x, data: 0x%x", reg, data);

	/** prepare data */
	uint8_t wr_buf[] = {T58XX_FAKE2C_DEVICE_ADDRESS << 1, reg, data};
	/* put into wr_buf since it gets written first */
	uint8_t cyc_buf;

	/** start with thsel low */
	gpio_set_level(drv_cfg->thsel, 0);
	/** Clock device before writing to prepare device for communication */
	prv_clock_bitbang(dev, T58XX_FAKE2C_PRE_WRITE_CYCLES, T58XX_FAKE2C_CLK_PERIOD_US);
	/** write start condition*/
	gpio_set_level(drv_cfg->thsel, 1);
	prv_clock_bitbang(dev, T58XX_FAKE2C_START_PILOT_CLKS, T58XX_FAKE2C_CLK_PERIOD_US);
	/** write first space before writing data */
	gpio_set_level(drv_cfg->thsel, 0);
	prv_clock_bitbang(dev, T58XX_FAKE2C_SPACE, T58XX_FAKE2C_CLK_PERIOD_US);
	/** write data */
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 8; j++) {

			if (wr_buf[i] & (1 << (7 - j))) {
				/* sending one */
				cyc_buf = T58XX_FAKE2C_ONE;
			} else {
				/* sending 0 */
				cyc_buf = T58XX_FAKE2C_ZERO;
			}
			/** Send data bit */
			gpio_set_level(drv_cfg->thsel, 1);
			prv_clock_bitbang(dev, cyc_buf, T58XX_FAKE2C_CLK_PERIOD_US);
			/** Send space */
			gpio_set_level(drv_cfg->thsel, 0);
			prv_clock_bitbang(dev, T58XX_FAKE2C_SPACE, T58XX_FAKE2C_CLK_PERIOD_US);
		}
	}
	/**write stop condition */
	gpio_set_level(drv_cfg->thsel, 1);
	prv_clock_bitbang(dev, T58XX_FAKE2C_STOP, T58XX_FAKE2C_CLK_PERIOD_US);
	gpio_set_level(drv_cfg->thsel, 0);

	/**keep clock to apply */
	prv_clock_bitbang(dev, T58XX_FAKE2C_POST_WRITE_CYCLES, T58XX_FAKE2C_CLK_PERIOD_US);
	return ESP_OK;
}

/**
 * @brief Function for writing to multiple T58XX registers from provided array of address data
 * pairs.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] data Array of address data pairs with sequence to write to device.
 * @param[in] num Number of address data pairs in data array.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
esp_err_t prv_multi_reg_write(const struct device *dev, struct t58xx_address_data_pair *data, uint8_t num)
{
	esp_err_t err;
	for (int i = 0; i < num; i++) {
		err = prv_reg_write(dev, data[i].address, data[i].data);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "t58xx reg write err: %d, addr: 0x%x, data: 0x%x", err,
				data[i].address, data[i].data);
			return err;
		}
	}
	return ESP_OK;
}

/**
 * @brief Function helper for sending predefined sequence that unlocks aad mode.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 if successful.
 * @retval negative errno code otherwise.
 */
esp_err_t prv_aad_unlock_sequence(const struct device *dev)
{
	struct t58xx_aad_drv_data *drv_data = dev->data;
	esp_err_t err;

	/** This is unlock sequence for AAD modes provided in datasheet. */
	struct t58xx_address_data_pair write_data[] = {
		{0x5C, 0x00}, {0x3E, 0x00}, {0x6F, 0x00}, {0x3B, 0x00}, {0x4C, 0x00},
	};

	err = prv_multi_reg_write(dev, write_data, sizeof(write_data) / sizeof(write_data[0]));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_t58xx_multi_reg_write, err: %d", err);
		return err;
	}
	drv_data->aad_unlocked = true;

	return ESP_OK;
}

esp_err_t t58xx_aad_sleep(const struct device *dev)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	/** Make sure there is no PDM transfer in progress, and we can take clock signal */
	struct t58xx_drv_data *pdm_data = drv_cfg->i2s_dev->data;
	if (pdm_data->rx_handle->state == I2S_CHAN_STATE_RUNNING) {
		ESP_LOGE(TAG, "Cannot write to device while pdm is active");
		return ESP_ERR_INVALID_STATE;
	}

	prv_clock_bitbang(
		dev, T58XX_ENTER_SLEEP_MODE_CLOCKING_TIME_US / T58XX_ENTER_SLEEP_MODE_CLK_PERIOD_US,
		T58XX_ENTER_SLEEP_MODE_CLK_PERIOD_US);
	return ESP_OK;
}

/**
 * @brief ISR called when the WAKE interrupt is triggered
 */
static void IRAM_ATTR prv_wake_cb_handler(void *arg)
{
	struct device *dev = (struct device *)arg;
	struct t58xx_aad_drv_data *drv_data = dev->data;
	// const struct t58xx_aad_drv_cfg *drv_cfg = drv_data->aad_cfg;
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config; // No ?

	if (drv_data->int_handled) {
		return;
	}
	drv_data->int_handled = true;
	esp_err_t err = gpio_intr_disable(drv_cfg->wake);
	if (err != ESP_OK) {
		ESP_DRAM_LOGE(DRAM_STR("T58XX_ISR"), "gpio_intr_disable failed: %d", err);
		return;
	}

	if (drv_data->wake_handler) {
		drv_data->wake_handler(dev);
	}
}

esp_err_t t58xx_reset(const struct device *dev)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	struct t58xx_aad_drv_data *drv_data = dev->data;
	if (!drv_cfg->micen_available) {
		return ESP_ERR_NOT_SUPPORTED;
	}
	gpio_set_level(drv_cfg->micen, 0);

	drv_data->aad_unlocked = false;
	drv_data->aad_enabled_mode = T58XX_AAD_SELECT_NONE;

	/* Wait for device to power down and reapply power to reset it */
	vTaskDelay(pdMS_TO_TICKS(T58XX_RESET_TIME_MS));
	gpio_set_level(drv_cfg->micen, 1);

	return ESP_OK;
}

esp_err_t t58xx_aad_wake_clear(const struct device *dev)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	struct t58xx_aad_drv_data *drv_data = dev->data;
	esp_err_t err;

	if (!drv_cfg->wake_available) {
		ESP_LOGE(TAG, "interrupt is not available (managed externally)");
		return ESP_ERR_NOT_ALLOWED;
	}

	if (drv_data->cb_configured) {
		// Rearm the interrupt previously disabled in the ISR
		err = gpio_intr_enable(drv_cfg->wake);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "gpio_intr_enable failed: %d", err);
			return err;
		}
	}
	drv_data->int_handled = false;
	return ESP_OK;
}

void t58xx_aad_wake_handler_set(const struct device *dev, t58xx_wake_handler_t handler)
{
	struct t58xx_aad_drv_data *drv_data = dev->data;
	drv_data->wake_handler = handler;
}

/**
 * @brief Function for setting AAD mode from provided array of address data pairs.
 *
 * @param[in] dev Pointer to the device structure for the driver instance.
 * @param[in] write_array Array of address data pairs with setup sequence to write to device.
 * @param[in] write_array_size Size of write_array.
 *
 * @retval 0 if successful.
 * @retval negative errno code if otherwise.
 */
esp_err_t prv_aad_mode_set(const struct device *dev, struct t58xx_address_data_pair *write_array,
			 uint8_t write_array_size)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	struct t58xx_aad_drv_data *drv_data = dev->data;
	struct t58xx_drv_data *pdm_data = drv_cfg->i2s_dev->data;
	esp_err_t err;

	/** Note: The test to make sure there is no PDM transfer in progress, and we can take clock signal
	 * is made in prv_aad_unlock_sequence & prv_reg_write */

	/** Between AAD mode config & AAD activation with the sleep sequence,
	 * there is an acknowledgement on the WAKE pin of 12us.
	 * We disable the interrupt to avoid false triggers. */
	if (drv_cfg->wake_available && drv_data->cb_configured) {
		err = gpio_intr_disable(drv_cfg->wake);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "gpio_intr_disable failed: %d", err);
			return err;
		}
	}

	if (drv_data->aad_unlocked == false) {
		err = prv_aad_unlock_sequence(dev);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "error writing aad unlock sequence, err: %d", err);
			return err;
		}
	}

	err = prv_multi_reg_write(dev, write_array, write_array_size);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_t58xx_multi_reg_write, err: %d", err);
		return err;
	}

	t58xx_aad_sleep(dev);

	pdm_data->aad_child_dev = dev;

	/** Configure interrupts */
	if (drv_cfg->wake_available) {
		if (drv_data->cb_configured == false) {
			drv_data->int_handled = false; // Clear interrupt

			err = gpio_set_intr_type(drv_cfg->wake, GPIO_INTR_POSEDGE);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "gpio_set_intr_type failed: %d", err);
				return err;
			}

			// Register the handler
			err = gpio_isr_handler_add(drv_cfg->wake, prv_wake_cb_handler, (void *)dev);
			if (err != ESP_OK) {
				ESP_LOGE(TAG, "gpio_isr_handler_add failed: %d", err);
				return err;
			}
			drv_data->cb_configured = true;
		}
		err = gpio_intr_enable(drv_cfg->wake);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "gpio_intr_enable failed: %d", err);
			return err;
		}
	}
	return ESP_OK;
}

esp_err_t t58xx_aad_a_mode_set(const struct device *dev, struct t58xx_aad_a_conf *aadconf)
{
	struct t58xx_aad_drv_data *drv_data = dev->data;
	esp_err_t err;

	struct t58xx_address_data_pair write_data[] = {
		{T58XX_REG_AAD_MODE, 0x00},
		{T58XX_REG_AAD_A_LPF, aadconf->aad_a_lpf},
		{T58XX_REG_AAD_A_THR, aadconf->aad_a_thr},
		{T58XX_REG_AAD_MODE, T58XX_AAD_SELECT_A},
	};
	err = prv_aad_mode_set(dev, write_data, sizeof(write_data) / sizeof(write_data[0]));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T58XX_AAD_SELECT_A;
	return ESP_OK;
}

/** Helper macro to avoid code duplication in D1 and D2 configuration functions */
#define AAD_D_CONFIG_BUILDER(write_data, mode)                                                     \
	struct t58xx_address_data_pair write_data[] = {                                            \
		{T58XX_REG_AAD_MODE, 0x00},                                                        \
		{T58XX_REG_AAD_D_FLOOR_HI, (aadconf->aad_d_floor >> 8) & 0x1F},                    \
		{T58XX_REG_AAD_D_FLOOR_LO, aadconf->aad_d_floor & 0xFF},                           \
		{0x2C, 0x32},                                                                      \
		{0x2D, 0xC0},                                                                      \
		{T58XX_REG_AAD_D_REL_PULSE_MIN_LO, aadconf->aad_d_rel_pulse_min & 0xFF},           \
		{T58XX_REG_AAD_D_ABS_REL_PULSE_MIN_SHARED,                                         \
		 ((aadconf->aad_d_abs_pulse_min >> 4) / 0xF0) |                                    \
			 ((aadconf->aad_d_rel_pulse_min >> 8) & 0x0F)},                            \
		{T58XX_REG_AAD_D_ABS_PULSE_MIN_LO, aadconf->aad_d_abs_pulse_min & 0xFF},           \
		{T58XX_REG_AAD_D_ABS_THR_LO, aadconf->aad_d_abs_thr & 0xFF},                       \
		{T58XX_REG_AAD_D_ABS_THR_HI, ((aadconf->aad_d_abs_thr >> 8) & 0x1F) | 0x40},       \
		{T58XX_REG_AAD_D_REL_THR, aadconf->aad_d_rel_thr},                                 \
		{T58XX_REG_AAD_MODE, mode},                                                        \
	}

esp_err_t t58xx_aad_d1_mode_set(const struct device *dev, struct t58xx_aad_d_conf *aadconf)
{
	struct t58xx_aad_drv_data *drv_data = dev->data;
	esp_err_t err;
	AAD_D_CONFIG_BUILDER(write_data, T58XX_AAD_SELECT_D1);
	err = prv_aad_mode_set(dev, write_data, sizeof(write_data) / sizeof(write_data[0]));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T58XX_AAD_SELECT_D1;
	return ESP_OK;
}

esp_err_t t58xx_aad_d2_mode_set(const struct device *dev, struct t58xx_aad_d_conf *aadconf)
{
	struct t58xx_aad_drv_data *drv_data = dev->data;
	esp_err_t err;
	AAD_D_CONFIG_BUILDER(write_data, T58XX_AAD_SELECT_D2);
	err = prv_aad_mode_set(dev, write_data, sizeof(write_data) / sizeof(write_data[0]));
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_aad_mode_set, err: %d", err);
		return err;
	}
	drv_data->aad_enabled_mode = T58XX_AAD_SELECT_D2;
	return ESP_OK;
}

esp_err_t t58xx_aad_mode_disable(const struct device *dev)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	struct t58xx_aad_drv_data *drv_data = dev->data;
	struct t58xx_drv_data *pdm_data = drv_cfg->i2s_dev->data;

	esp_err_t err;
	/** Set AAD mode to zero while we configure device to avoid problems */
	err = prv_reg_write(dev, T58XX_REG_AAD_MODE, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "prv_t58xx_reg_write, err: %d", err);
		return err;
	}
	/** Disable interrupts */
	if (drv_cfg->wake_available) {
		err = gpio_intr_disable(drv_cfg->wake);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "gpio_intr_disable, err: %d", err);
			return err;
		}
	}
	drv_data->aad_enabled_mode = T58XX_AAD_SELECT_NONE;
	drv_data->aad_unlocked = false;
	pdm_data->aad_child_dev = NULL;
	return ESP_OK;
}

esp_err_t t58xx_aad_init(const struct device *dev)
{
	const struct t58xx_aad_drv_cfg *drv_cfg = dev->config;
	struct t58xx_aad_drv_data *drv_data = dev->data;

	/* T58XX specific configuration */
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = 1ULL << drv_cfg->thsel,
		.pull_down_en = 0,
		.pull_up_en = 0
	};
	esp_err_t err = gpio_config(&io_conf);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure THSEL pin");
		return err;
	}

	io_conf.pin_bit_mask = 1ULL << drv_cfg->pdmclk;
	err = gpio_config(&io_conf);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "Failed to configure PDMCLK pin");
		return err;
	}

	/* set micen low to turn off microphone and make sure we start in reset state */
	if (drv_cfg->micen_available) {
		io_conf.pin_bit_mask = 1ULL << drv_cfg->micen;
		err = gpio_config(&io_conf);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to configure MICEN pin");
			return err;
		}
		t58xx_reset(dev);
	}

	if (drv_cfg->wake_available) {
		io_conf.mode = GPIO_MODE_INPUT;
		io_conf.pin_bit_mask = 1ULL << drv_cfg->wake;
		err = gpio_config(&io_conf);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to configure WAKE pin");
			return err;
		}
	}

	drv_data->aad_cfg = drv_cfg;
	return ESP_OK;
}
