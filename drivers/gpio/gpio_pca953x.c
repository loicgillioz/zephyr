/*
 * Copyright (c) 2018 Peter Bigot Consulting, LLC
 * Copyright (c) 2018 Aapo Vienamo
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2019 Vestas Wind Systems A/S
 * Copyright (c) 2020 ZedBlox Ltd.
 * Copyright (c) 2021 Laird Connectivity
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* xCA9534 : 8-bit, 1 port I2C I/O expander with interrupt */
/* xCA9535 : 16-bit, 2 ports I2C I/O expander with interrupt */
/* xCA9536 : 4-bit, 1 port I2C I/O expander */
/* xCA9537 : 4-bit, 1 port I2C I/O expander with interrupt and reset */
/* xCA9538 : 8-bit, 1 port I2C I/O expander with interrupt and reset */
/* xCA9539 : 16-bit, 2 ports I2C I/O expander with interrupt and reset */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pca953x, CONFIG_GPIO_LOG_LEVEL);

#include <zephyr/drivers/gpio/gpio_utils.h>

/* xCA953x Register addresses */
#define XCA953X_INPUT_PORT    0x00
#define XCA953X_OUTPUT_PORT   0x01
#define XCA953X_CONFIGURATION 0x03
#define REG_INPUT_LATCH_PORT0 0x42
#define REG_INT_MASK_PORT0    0x45

#define XCA953X_ALIGN_REG(ngpios, reg) ((ngpios > 8 ? reg << 1 : reg))

/* Theoretically the pins are split into ports of max. 8 gpios, but the driver
 * treats them as a single port. The functionality is not improved by splitting
 * into separate ports, so to keep it simple it's assumed as single port.
 */
/* Maximum amount of GPIOs in the family is 16 (then using uint16_t) */
#define ALL_PINS(num_pins) ((uint16_t)BIT_MASK(num_pins))

/** Cache of the output configuration and data of the pins. */
struct xca953x_pin_state {
	uint16_t dir;
	uint16_t input;
	uint16_t output;
};

struct xca953x_irq_state {
	uint16_t rising;
	uint16_t falling;
};

/** Runtime driver data */
struct xca953x_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	struct xca953x_pin_state pin_state;
	struct k_sem lock;
	struct gpio_callback gpio_cb;
	struct k_work work;
	struct xca953x_irq_state irq_state;
	const struct device *dev;
	/* user ISR cb */
	sys_slist_t cb;
};

/** Configuration data */
struct xca953x_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	struct i2c_dt_spec i2c;
	const struct gpio_dt_spec gpio_int;
	bool interrupt_enabled;
	int interrupt_mask;
	int input_latch;
	int ngpios;
};

/**
 * @brief Gets the state of input pins of the XCA953X I/O Port and
 * stores in driver data struct.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int update_input(const struct device *dev)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	uint16_t input_states;
	int rc = 0;

	if (cfg->ngpios == -1) {
		return -EINVAL;
	}

	uint8_t input_states_port0;
	rc = i2c_reg_read_byte_dt(&cfg->i2c, XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_INPUT_PORT),
				  &input_states_port0);
	if (rc != 0) {
		return rc;
	}

	uint8_t input_states_port1 = 0;
	if (cfg->ngpios > 8) {
		rc = i2c_reg_read_byte_dt(&cfg->i2c,
					  XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_INPUT_PORT) + 1,
					  &input_states_port1);
		if (rc != 0) {
			return rc;
		}
	}

	input_states = (input_states_port1 << 8) | input_states_port0;

	if (rc == 0) {
		drv_data->pin_state.input = input_states;
	}
	return rc;
}

/**
 * @brief Gets the state of input pins of the XCA953X I/O Port and
 * stores in driver data struct.
 *
 * @param dev Pointer to the device structure for the driver instance.
 *
 * @retval 0 If successful.
 * @retval Negative value for error code.
 */
static int update_output(const struct device *dev)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	uint16_t output_states = drv_data->pin_state.output;
	int rc = 0;

	if (cfg->ngpios == -1) {
		return -EINVAL;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_OUTPUT_PORT),
				   output_states & BIT_MASK(8));
	if (rc != 0) {
		return rc;
	}

	if (cfg->ngpios > 8) {
		rc = i2c_reg_write_byte_dt(&cfg->i2c,
					   XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_OUTPUT_PORT) + 1,
					   (output_states >> 8) & BIT_MASK(8));
		if (rc != 0) {
			return rc;
		}
	}

	return rc;
}

static int update_config(const struct device *dev)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	uint16_t config_states = drv_data->pin_state.dir;
	int rc = 0;

	if (cfg->ngpios == -1) {
		return -EINVAL;
	}

	rc = i2c_reg_write_byte_dt(&cfg->i2c, XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_CONFIGURATION),
				   config_states & BIT_MASK(8));
	if (rc != 0) {
		return rc;
	}

	if (cfg->ngpios > 8) {
		rc = i2c_reg_write_byte_dt(&cfg->i2c,
					   XCA953X_ALIGN_REG(cfg->ngpios, XCA953X_CONFIGURATION) + 1,
					   (config_states >> 8) & BIT_MASK(8));
		if (rc != 0) {
			return rc;
		}
	}

	return rc;

}

/**
 * @brief Handles interrupt triggered by the interrupt pin of XCA953X I/O Port.
 *
 * If nint_gpios is configured in device tree then this will be triggered each
 * time a gpio configured as an input changes state. The gpio input states are
 * read in this function which clears the interrupt.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
static void gpio_xca953x_handle_interrupt(const struct device *dev)
{
	struct xca953x_drv_data *drv_data = dev->data;
	struct xca953x_irq_state *irq_state = &drv_data->irq_state;
	int rc;
	uint16_t previous_state;
	uint16_t current_state;
	uint16_t transitioned_pins;
	uint16_t interrupt_status = 0;

	k_sem_take(&drv_data->lock, K_FOREVER);

	/* Any interrupts enabled? */
	if (!irq_state->rising && !irq_state->falling) {
		rc = -EINVAL;
		goto out;
	}

	/* Store previous input state then read new value */
	previous_state = drv_data->pin_state.input;
	rc = update_input(dev);
	if (rc != 0) {
		goto out;
	}

	/* Find out which input pins have changed state */
	current_state = drv_data->pin_state.input;
	transitioned_pins = previous_state ^ current_state;

	/* Mask gpio transactions with rising/falling edge interrupt config */
	interrupt_status = (irq_state->rising & transitioned_pins & current_state);
	interrupt_status |= (irq_state->falling & transitioned_pins & previous_state);

out:
	k_sem_give(&drv_data->lock);

	if ((rc == 0) && (interrupt_status)) {
		gpio_fire_callbacks(&drv_data->cb, dev, interrupt_status);
	}
}

/**
 * @brief Work handler for XCA953X interrupt
 *
 * @param work Work struct that contains pointer to interrupt handler function
 */
static void gpio_xca953x_work_handler(struct k_work *work)
{
	struct xca953x_drv_data *drv_data = CONTAINER_OF(work, struct xca953x_drv_data, work);

	gpio_xca953x_handle_interrupt(drv_data->dev);
}

/**
 * @brief ISR for interrupt pin of XCA953X
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param gpio_cb Pointer to callback function struct
 * @param pins Bitmask of pins that triggered interrupt
 */
static void gpio_xca953x_init_cb(const struct device *dev, struct gpio_callback *gpio_cb,
				 uint32_t pins)
{
	struct xca953x_drv_data *drv_data = CONTAINER_OF(gpio_cb, struct xca953x_drv_data, gpio_cb);

	ARG_UNUSED(pins);

	k_work_submit(&drv_data->work);
}

static int gpio_xca953x_config(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	struct xca953x_pin_state *pins = &drv_data->pin_state;
	int rc = 0;
	bool data_first = false;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	/* Single Ended lines (Open drain and open source) not supported */
	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	/* The XCA953X has no internal pull up support */
	if (((flags & GPIO_PULL_UP) != 0) || ((flags & GPIO_PULL_DOWN) != 0)) {
		return -ENOTSUP;
	}

	/* Simultaneous input & output mode not supported */
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	/* Ensure either Output or Input is specified */
	if ((flags & GPIO_OUTPUT) != 0) {
		pins->dir &= ~BIT(pin);
		if ((flags & GPIO_OUTPUT_INIT_LOW) != 0) {
			pins->output &= ~BIT(pin);
			data_first = true;
		} else if ((flags & GPIO_OUTPUT_INIT_HIGH) != 0) {
			pins->output |= BIT(pin);
			data_first = true;
		}
	} else if ((flags & GPIO_INPUT) != 0) {
		pins->dir |= BIT(pin);
	} else {
		rc = -ENOTSUP;
		goto out;
	}

	/* Set output values */
	if (data_first) {
		update_output(dev);
	}

	if (rc == 0) {
		/* Set pin directions */
		rc = update_config(dev);
	}

	if (rc == 0) {
		/* Refresh input status */
		rc = update_input(dev);
	}

out:
	k_sem_give(&drv_data->lock);
	return rc;
}

static int gpio_xca953x_port_read(const struct device *dev, gpio_port_value_t *value)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	uint16_t input_pin_data;
	int rc = 0;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	/* Read Input Register */
	update_input(dev);

	LOG_DBG("read %x got %d", input_pin_data, rc);

	if (rc == 0) {
		drv_data->pin_state.input = input_pin_data;
		*value = (gpio_port_value_t)(drv_data->pin_state.input);
	}

	k_sem_give(&drv_data->lock);
	return rc;
}

static int gpio_xca953x_port_write(const struct device *dev, gpio_port_pins_t mask,
				   gpio_port_value_t value, gpio_port_value_t toggle)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	uint16_t *outp = &drv_data->pin_state.output;
	int rc;
	uint16_t orig_out;
	uint16_t out;

	/* Can't do I2C bus operations from an ISR */
	if (k_is_in_isr()) {
		return -EWOULDBLOCK;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	orig_out = *outp;
	out = ((orig_out & ~mask) | (value & mask)) ^ toggle;
	drv_data->pin_state.output = out;

	rc = update_output(dev);

	if (rc != 0) {
		/* Restore the original output value */
		drv_data->pin_state.output = orig_out;
	}

	k_sem_give(&drv_data->lock);

	LOG_DBG("write %016x msk %016x val %016x => %016x: %d", orig_out, mask, value, out, rc);

	return rc;
}

static int gpio_xca953x_port_set_masked(const struct device *dev, gpio_port_pins_t mask,
					gpio_port_value_t value)
{
	return gpio_xca953x_port_write(dev, mask, value, 0);
}

static int gpio_xca953x_port_set_bits(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_xca953x_port_write(dev, pins, pins, 0);
}

static int gpio_xca953x_port_clear_bits(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_xca953x_port_write(dev, pins, 0, 0);
}

static int gpio_xca953x_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
	return gpio_xca953x_port_write(dev, 0, 0, pins);
}

static int gpio_xca953x_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
						enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	struct xca953x_irq_state *irq = &drv_data->irq_state;

	if (!cfg->interrupt_enabled) {
		return -ENOTSUP;
	}
	/* Device does not support level-triggered interrupts. */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	k_sem_take(&drv_data->lock, K_FOREVER);

	if (mode == GPIO_INT_MODE_DISABLED) {
		irq->falling &= ~BIT(pin);
		irq->rising &= ~BIT(pin);
	} else { /* GPIO_INT_MODE_EDGE */
		if (trig == GPIO_INT_TRIG_BOTH) {
			irq->falling |= BIT(pin);
			irq->rising |= BIT(pin);
		} else if (trig == GPIO_INT_TRIG_LOW) {
			irq->falling |= BIT(pin);
			irq->rising &= ~BIT(pin);
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			irq->falling &= ~BIT(pin);
			irq->rising |= BIT(pin);
		}
	}

	k_sem_give(&drv_data->lock);

	return 0;
}

static int gpio_xca953x_manage_callback(const struct device *dev, struct gpio_callback *callback,
					bool set)
{
	struct xca953x_drv_data *data = dev->data;

	return gpio_manage_callback(&data->cb, callback, set);
}

/**
 * @brief Initialization function of XCA953X
 *
 * This sets initial input/ output configuration and output states.
 * The interrupt is configured if this is enabled.
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int xca953x_root_init(const struct device *dev)
{
	const struct xca953x_config *cfg = dev->config;
	struct xca953x_drv_data *drv_data = dev->data;
	int rc = 0;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C bus device not found");
		goto out;
	}

	/* Do an initial read, this clears the interrupt pin and sets
	 * up the initial value of the pin state input data.
	 */
	rc = update_input(dev);
	if (rc) {
		goto out;
	}

	if (cfg->interrupt_enabled) {
		if (!gpio_is_ready_dt(&cfg->gpio_int)) {
			LOG_ERR("Cannot get pointer to gpio interrupt device");
			rc = -EINVAL;
			goto out;
		}

		drv_data->dev = dev;

		k_work_init(&drv_data->work, gpio_xca953x_work_handler);

		rc = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
		if (rc) {
			goto out;
		}

		rc = gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
		if (rc) {
			goto out;
		}

		gpio_init_callback(&drv_data->gpio_cb, gpio_xca953x_init_cb,
				   BIT(cfg->gpio_int.pin));

		rc = gpio_add_callback(cfg->gpio_int.port, &drv_data->gpio_cb);

		if (rc) {
			goto out;
		}

		/* This may not present on all variants of device */
		if (cfg->input_latch > -1) {
			rc = i2c_reg_write_byte_dt(&cfg->i2c, REG_INPUT_LATCH_PORT0,
						   cfg->input_latch);
		}
		if (cfg->interrupt_mask > -1) {
			rc = i2c_reg_write_byte_dt(&cfg->i2c, REG_INT_MASK_PORT0,
						   cfg->interrupt_mask);
		}
	}
out:
	if (rc) {
		LOG_ERR("%s init failed: %d", dev->name, rc);
	} else {
		LOG_INF("%s init ok", dev->name);
	}
	return rc;
}

/* HOG support */
/*
struct xca953x_gpio_config {
	const struct device *root;
	...
};

static inline const struct pca953x_root_config *
get_root_config_from_gpio(const struct device *dev)
{
	const struct pca953x_gpio_config *gpio_config = dev->config;
	return gpio_config->root->config;
}

static int xca953x_gpio_init(const struct device *dev)
{
	const struct pca953x_gpio_config *chan_cfg = dev->config;
	const struct pca953x_root_config *root_cfg =
			get_root_config_from_gpio(dev);

	if (!device_is_ready(chan_cfg->root)) {
		LOG_ERR("I2C mux root %s not ready", chan_cfg->root->name);
		return -ENODEV;
	}

	if (chan_cfg->chan_mask >= BIT(root_cfg->nchans)) {
		LOG_ERR("Wrong DTS address provided for %s", dev->name);
		return -EINVAL;
	}

	return 0;
}

// Not clear at the moment how to use Hogs, it seems 1 hog is 1 port
#define XCA953X_CHILD_DEFINE(node_id, n)				    \
	static const struct xca954x_gpio_config			    \
		xca##n##a_down_config_##node_id = {			    \
		.chan_mask = BIT(DT_REG_ADDR(node_id)),			    \
		.root = DEVICE_DT_GET(DT_PARENT(node_id)),		    \
	};								    \
	DEVICE_DT_DEFINE(node_id,					    \
			 xca953x_gpio_init,				    \
			 NULL,						    \
			 NULL,						    \
			 &xca##n##a_down_config_##node_id,		    \
			 POST_KERNEL, CONFIG_I2C_TCA954X_CHANNEL_INIT_PRIO, \
			 &xca954x_api_funcs);

*/

static const struct gpio_driver_api api_table = {
	.pin_configure = gpio_xca953x_config,
	.port_get_raw = gpio_xca953x_port_read,
	.port_set_masked_raw = gpio_xca953x_port_set_masked,
	.port_set_bits_raw = gpio_xca953x_port_set_bits,
	.port_clear_bits_raw = gpio_xca953x_port_clear_bits,
	.port_toggle_bits = gpio_xca953x_port_toggle_bits,
	.pin_interrupt_configure = gpio_xca953x_pin_interrupt_configure,
	.manage_callback = gpio_xca953x_manage_callback,
};

#define GPIO_XCA953X_INIT(n, inst, n_gpios)                                                        \
	static const struct xca953x_config xca##n##_cfg_##inst = {                                 \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(inst),            \
			},                                                                         \
		.interrupt_enabled = DT_INST_NODE_HAS_PROP(inst, nint_gpios),                      \
		.gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, nint_gpios, {0}),                       \
		.interrupt_mask = DT_INST_PROP_OR(inst, interrupt_mask, -1),                       \
		.input_latch = DT_INST_PROP_OR(inst, input_latch, -1),                             \
		.ngpios = n_gpios};                                                                \
                                                                                                   \
	static struct xca953x_drv_data xca##n##_drvdata_##inst = {                                 \
		.lock = Z_SEM_INITIALIZER(xca##n##_drvdata_##inst.lock, 1, 1),                     \
		.pin_state.dir = ALL_PINS(n_gpios),                                                \
		.pin_state.output = ALL_PINS(n_gpios),                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, xca953x_root_init, NULL, &xca##n##_drvdata_##inst,             \
			      &xca##n##_cfg_##inst, POST_KERNEL,                                   \
			      CONFIG_GPIO_PCA953X_INIT_PRIORITY, &api_table);                      
	/*DT_FOREACH_CHILD_VARGS(                                                                    \
		DT_INST(inst, nxp_pca##n), XCA953X_CHILD_DEFINE,                                   \
		n); // calls XCA954x_CHILD_DEFINE for each child node of xca954x node
		*/

/*
 * xCA9534 : 8 gpios
 */
#define XCA9534_INIT(inst) GPIO_XCA953X_INIT(9534, inst, 8)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9534
DT_INST_FOREACH_STATUS_OKAY(XCA9534_INIT)

/*
 * xCA9535 : 16 gpios
 */
#define XCA9535_INIT(inst) GPIO_XCA953X_INIT(9535, inst, 16)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9535
DT_INST_FOREACH_STATUS_OKAY(XCA9535_INIT)

/*
 * xCA9536 : 4 gpios
 */
#define XCA9536_INIT(inst) GPIO_XCA953X_INIT(9536, inst, 4)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9536
DT_INST_FOREACH_STATUS_OKAY(XCA9536_INIT)

/*
 * xCA9537 : 4 gpios
 */
#define XCA9537_INIT(inst) GPIO_XCA953X_INIT(9537, inst, 4)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9537
DT_INST_FOREACH_STATUS_OKAY(XCA9537_INIT)

/*
 * xCA9538 : 8 gpios
 */
#define XCA9538_INIT(inst) GPIO_XCA953X_INIT(9538, inst, 8)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9538
DT_INST_FOREACH_STATUS_OKAY(XCA9538_INIT)

/*
 * xCA9539: 16 gpios
 */
#define XCA9539_INIT(inst) GPIO_XCA953X_INIT(9539, inst, 16)
#undef DT_DRV_COMPAT
#define DT_DRV_COMPAT nxp_pca9539
DT_INST_FOREACH_STATUS_OKAY(XCA9539_INIT)