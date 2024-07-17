/* bno055.c - Driver for Bosch BNO055 orientation sensor */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>

#include <zephyr/logging/log.h>

#include "bno055.h"

LOG_MODULE_REGISTER(BNO055, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BNO055 driver enabled without any devices"
#endif

struct bno055_data {
	uint8_t chip_id;
	uint8_t page_id;

	struct bno055_euler_t euler_reg_hpr;
	struct bno055_euler_double_t euler_double_hpr;
	struct bno055_accel_t accel_reg_xyz;
};

struct bno055_config {
	union bno055_bus bus;
	const struct bno055_bus_io *bus_io;
};

static inline struct bno055_data *to_data(const struct device *dev)
{
	return dev->data;
}

static inline int bno055_bus_check(const struct device *dev)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->check(&cfg->bus);
}

static inline int bno055_reg_read(const struct device *dev, uint8_t start, uint8_t *buf, int size)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->read(&cfg->bus, start, buf, size);
}

static inline int bno055_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct bno055_config *cfg = dev->config;

	return cfg->bus_io->write(&cfg->bus, reg, val);
}

static int bno055_read_euler_hrp(const struct device *dev)
{
	/* Array holding the Euler hrp value
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] - h->LSB
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] - h->MSB
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] - r->MSB
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] - r->MSB
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] - p->MSB
	 * data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] - p->MSB
	 */
	struct bno055_data *data = to_data(dev);
	uint8_t data_u8[BNO055_EULER_HRP_DATA_SIZE] = {0, 0, 0, 0, 0, 0};
	int ret = BNO055_ERROR;

	if (data->page_id != BNO055_PAGE_ZERO) {
		ret = bno055_reg_write(dev, BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
	}

	if ((ret == BNO055_SUCCESS) || (data->page_id == BNO055_PAGE_ZERO)) {
		ret = bno055_reg_read(dev, BNO055_EULER_H_LSB_ADDR, data_u8,
				      BNO055_EULER_HRP_DATA_SIZE);

		/* Data h*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB], BNO055_EULER_H_LSB_VALUEH);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB], BNO055_EULER_H_MSB_VALUEH);
		data->euler_reg_hpr.h =
			((((int16_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_MSB]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_SENSOR_DATA_EULER_HRP_H_LSB]));

		/* Data r*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB], BNO055_EULER_R_LSB_VALUER);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB], BNO055_EULER_R_MSB_VALUER);
		data->euler_reg_hpr.r =
			((((int16_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_MSB]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_SENSOR_DATA_EULER_HRP_R_LSB]));

		/* Data p*/
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB], BNO055_EULER_P_LSB_VALUEP);
		data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB] = BNO055_GET_BITSLICE(
			data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB], BNO055_EULER_P_MSB_VALUEP);
		data->euler_reg_hpr.p =
			((((int16_t)((int8_t)data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_MSB]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_SENSOR_DATA_EULER_HRP_P_LSB]));
	}

	return ret;
}

static int bno055_read_accel_xyz(const struct device *dev)
{
	/* Array holding the accelerometer xyz value
	 * data_u8[BNO055_ACCEL_DATA_X_lSB_ADDR] - x->LSB
	 * data_u8[BNO055_ACCEL_DATA_X_MSB_ADDR] - x->MSB
	 * data_u8[BNO055_SENSOR_DATA_ACCEL_Y_LSB] - y->LSB
	 * data_u8[BNO055_SENSOR_DATA_ACCEL_Y_MSB] - y->MSB
	 * data_u8[BNO055_SENSOR_DATA_ACCEL_Z_LSB] - z->LSB
	 * data_u8[BNO055_SENSOR_DATA_ACCEL_Z_MSB] - z->MSB
	 */
	struct bno055_data *data = to_data(dev);
	uint8_t data_u8[BNO055_ACCEL_XYZ_DATA_SIZE] = {0, 0, 0, 0, 0, 0};
	int ret = BNO055_ERROR;

	if (data->page_id != BNO055_PAGE_ZERO) {
		ret = bno055_reg_write(dev, BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
	}

	if ((ret == BNO055_SUCCESS) || (data->page_id == BNO055_PAGE_ZERO)) {
		ret = bno055_reg_read(dev, BNO055_ACCEL_DATA_X_LSB_ADDR, data_u8,
							  BNO055_ACCEL_XYZ_DATA_SIZE);

		/* Data x */
		data->accel_reg_xyz.x =
			((((int16_t)((int8_t)data_u8[BNO055_ACCEL_DATA_X_MSB_ADDR]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_ACCEL_DATA_X_LSB_ADDR]));

		/* Data y */
		data->accel_reg_xyz.y =
			((((int16_t)((int8_t)data_u8[BNO055_ACCEL_DATA_Y_MSB_ADDR]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_ACCEL_DATA_Y_LSB_ADDR]));

		/* Data z */
		data->accel_reg_xyz.z =
			((((int16_t)((int8_t)data_u8[BNO055_ACCEL_DATA_Z_MSB_ADDR]))
			  << BNO055_SHIFT_EIGHT_BITS) |
			 (data_u8[BNO055_ACCEL_DATA_Z_LSB_ADDR]));
	}

	return ret;
}

static int bno055_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int ret = 0;

	switch (chan) {
	case SENSOR_CHAN_YRP:
		ret = bno055_read_euler_hrp(dev);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		ret = bno055_read_accel_xyz(dev);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static int bno055_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bno055_data *data = to_data(dev);

	switch (chan) {
	case SENSOR_CHAN_YAW:
		data->euler_double_hpr.h = (double)(data->euler_reg_hpr.h / BNO055_EULER_DIV_DEG);
		sensor_value_from_double(val, data->euler_double_hpr.h);
		break;
	case SENSOR_CHAN_PITCH:
		data->euler_double_hpr.p = (double)(data->euler_reg_hpr.p / BNO055_EULER_DIV_DEG);
		sensor_value_from_double(val, data->euler_double_hpr.p);
		break;
	case SENSOR_CHAN_ROLL:
		data->euler_double_hpr.r = (double)(data->euler_reg_hpr.r / BNO055_EULER_DIV_DEG);
		sensor_value_from_double(val, data->euler_double_hpr.r);
		break;

	case SENSOR_CHAN_ACCEL_X:
		data->accel_reg_xyz.x = (double)(data->accel_reg_xyz.x / BNO055_ACCEL_DIV_MG);
		sensor_value_from_double(val, data->accel_reg_xyz.x);
		break;

	case SENSOR_CHAN_ACCEL_Y:
		data->accel_reg_xyz.y = (double)(data->accel_reg_xyz.y / BNO055_ACCEL_DIV_MG);
		sensor_value_from_double(val, data->accel_reg_xyz.y);
		break;
	
	case SENSOR_CHAN_ACCEL_Z:
		data->accel_reg_xyz.z = (double)(data->accel_reg_xyz.z / BNO055_ACCEL_DIV_MG);
		sensor_value_from_double(val, data->accel_reg_xyz.z);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bno055_api_funcs = {
	.sample_fetch = bno055_sample_fetch,
	.channel_get = bno055_channel_get,
};

static int bno055_chip_init(const struct device *dev)
{
	struct bno055_data *data = to_data(dev);
	int err;

	err = bno055_bus_check(dev);
	if (err < 0) {
		LOG_DBG("bus check failed: %d", err);
		return err;
	}

	err = bno055_reg_read(dev, BNO055_CHIP_ID_ADDR, &data->chip_id, 1);
	if (err < 0) {
		LOG_DBG("ID read failed: %d", err);
		return err;
	}

	if (data->chip_id == BNO055_ID) {
		LOG_DBG("ID OK");
	}
	else {
		LOG_DBG("bad chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	// set mode to CONFIG_MODE
	err = bno055_reg_write(dev, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
	if (err != 0) {
		LOG_DBG("operation mode set failed: %d", err);
		return err;
	}

	// set to normal power mode
	err = bno055_reg_write(dev, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
	if (err != 0) {
		LOG_DBG("power mode set failed: %d", err);
		return err;
	}

	// selects page zero
	err = bno055_reg_write(dev, BNO055_PAGE_ID_ADDR, BNO055_PAGE_ZERO);
	if (err != 0) {
		LOG_DBG("page select failed: %d", err);
		return err;
	}

	// Reads the page id
	err = bno055_reg_read(dev, BNO055_PAGE_ID_ADDR, &data->page_id,
			      BNO055_GEN_READ_WRITE_LENGTH);

	if (err != 0) {
		LOG_DBG("page id read failed: %d", err);
		return err;
	}

	if(data->page_id != BNO055_PAGE_ZERO) {
		LOG_DBG("page id not set to zero: %d", data->page_id);
		return -ENOTSUP;
	}

	// sets operation mode
	err = bno055_reg_write(dev, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE);
	if (err != 0) {
		LOG_DBG("operation mode set failed: %d", err);
		return err;
	}

	k_msleep(600);

	LOG_DBG("\"%s\" OK", dev->name);

	// read SYS_STATUS and check if the system is fully operational
	uint8_t sys_status;
	err = bno055_reg_read(dev, BNO055_SYS_STAT_ADDR, &sys_status, BNO055_GEN_READ_WRITE_LENGTH);
	if (err != 0) {
		LOG_DBG("SYS_STATUS read failed: %d", err);
		return err;
	}

	/*Read: 0 System idle,
	1 System Error,
	2 Initializing peripherals
	3 System Initialization
	4 Executing selftest,
	5 Sensor fusion algorithm running,
	6 System running without fusion algorithm
	*/

	if (sys_status == 1) // System error : read specific register to get the error code
	{
		uint8_t sys_error;
		err = bno055_reg_read(dev, BNO055_SYS_ERR_ADDR, &sys_error,
				      BNO055_GEN_READ_WRITE_LENGTH);
		if (err != 0) {
			LOG_DBG("SYS_ERROR read failed: %d", err);
			return err;
		}
		LOG_ERR("System error: %d", sys_error);
		return -ENOTSUP;
	}

	if (sys_status <= 4) {
		LOG_DBG("System not fully operational: %d", sys_status);
		return -ENOTSUP;
	} else {
		LOG_DBG("System operational: %d", sys_status);
	}

	//read calibration status
	uint8_t calib_status;
	do{
		err = bno055_reg_read(dev, BNO055_CALIB_STAT_ADDR, &calib_status, BNO055_GEN_READ_WRITE_LENGTH);
		if (err != 0) {
			LOG_DBG("CALIB_STAT read failed: %d", err);
			return err;
		}
		LOG_DBG("Calibration status: %02x", calib_status);
		k_msleep(1000);
	}
	while((calib_status | 0xc0) != 0xFF);

	return 0;
}

/* Initializes a struct bno055_config for an instance on an I2C bus. */
#define BNO055_CONFIG_I2C(inst)                                                                    \
	{                                                                                          \
		.bus.i2c = I2C_DT_SPEC_INST_GET(inst), .bus_io = &bno055_bus_io_i2c,               \
	}

/*
 * Main instantiation macro, which selects the correct bus-specific
 * instantiation macros for the instance.
 */
#define BNO055_DEFINE(inst)                                                                        \
	static struct bno055_data bno055_data_##inst;                                              \
	static const struct bno055_config bno055_config_##inst = BNO055_CONFIG_I2C(inst);          \
	DEVICE_DT_INST_DEFINE(inst, bno055_chip_init, bno055_pm_ctrl, &bno055_data_##inst,         \
			      &bno055_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
			      &bno055_api_funcs);

/* Create the struct device for every status "okay" node in the devicetree. */
DT_INST_FOREACH_STATUS_OKAY(BNO055_DEFINE)
