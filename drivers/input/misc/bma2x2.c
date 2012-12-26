/*  Date: 2011/12/2 17:00:00
 *  Revision: 1.0
 */

/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2011 Bosch Sensortec GmbH
 * All Rights Reserved
 */


/* file BMA2X2.c
   brief This file contains all function implementations for the BMA2X2 in linux

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input/bma2x2.h>

unsigned char *sensor_name[] = { "BMA255", "BMA250E", "BMA2x2E", "BMA280" };

struct bma2x2_acc {
	s16 x;
	s16 y;
	s16 z;
};

struct bma2x2_data {
	struct i2c_client *bma2x2_client;
	atomic_t delay;
	atomic_t enable;
	int irq1;
	unsigned int chip_id;
	unsigned char mode;
	signed char sensor_type;
	struct input_dev *input;
	struct bma2x2_acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	struct bma2x2_platform_data *pdata;
};

struct bandwith_delay {
	u8 bandwidth;         /* bandwith reg setting */
	unsigned long delay_ms; /* bandwith in ms */
};

static const struct bandwith_delay bandwidth_delay_table[] = {
	{BMA2X2_BW_125HZ,    4}, /*   125Hz */
	{BMA2X2_BW_62_50HZ,  8}, /*  62.5Hz */
	{BMA2X2_BW_31_25HZ, 16}, /* 31.25Hz */
	{BMA2X2_BW_15_63HZ, 32}, /* 15.63Hz */
	{BMA2X2_BW_7_81HZ,  64}, /*  7.81Hz */
};

static int bma2x2_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 ret;
	ret = i2c_smbus_read_byte_data(client, reg_addr);
	if (ret < 0)
		return -1;
	*data = ret & 0x000000ff;

	return 0;
}

static int bma2x2_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (ret < 0)
		return -1;
	return 0;
}

static int bma2x2_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 ret;
	ret = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (ret < 0)
		return -1;
	return 0;
}

static int bma2x2_get_chip_type(int chip_id)
{
	switch (chip_id) {
	case BMA255_CHIP_ID:
		return BMA255_TYPE;
	case BMA250E_CHIP_ID:
		return BMA250E_TYPE;
	case BMA2x2E_CHIP_ID:
		return BMA2x2E_TYPE;
	case BMA280_CHIP_ID:
		return BMA280_TYPE;
	default:
		return -1;
	}
}

static int bma2x2_set_int_enable(struct i2c_client *client)
{
	int ret;
	unsigned char data1, data2;

	ret = bma2x2_smbus_read_byte(client, BMA2X2_INT_DATA_SEL_REG, &data1);
	if (ret < 0)
		return ret;
	ret = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE2_REG, &data2);
	if (ret < 0)
		return ret;

	data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_INT1_PAD_NEWDATA, 1);
	data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_NEW_DATA_INT, 1);

	ret = bma2x2_smbus_write_byte(client, BMA2X2_INT_DATA_SEL_REG, &data1);
	if (ret < 0)
		return ret;

	ret = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG, &data2);

	return ret;
}

static int bma2x2_set_mode(struct i2c_client *client, unsigned char Mode)
{
	int ret = -1;
	unsigned char data1, data2;

	ret = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG, &data1);
	if (ret < 0)
		return ret;
	ret = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG, &data2);
	if (ret < 0)
		return ret;

	switch (Mode) {
	case BMA2X2_MODE_NORMAL:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 0);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 0);
		break;
	case BMA2X2_MODE_LOWPOWER1:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 2);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 0);
		break;
	case BMA2X2_MODE_SUSPEND:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 4);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 0);
		break;
	case BMA2X2_MODE_DEEP_SUSPEND:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 1);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 1);
		break;
	case BMA2X2_MODE_LOWPOWER2:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 2);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 1);
		break;
	case BMA2X2_MODE_STANDBY:
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_MODE_CTRL, 4);
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_LOW_POWER_MODE, 1);
		break;
	default:
		return ret;
		break;
	}

	ret = bma2x2_smbus_write_byte(client, BMA2X2_MODE_CTRL_REG, &data1);
	if (ret < 0)
		return ret;
	ret = bma2x2_smbus_write_byte(client, BMA2X2_LOW_NOISE_CTRL_REG, &data2);

	return ret;
}

static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned long delay_ms)
{
	int ret = -1;
	int i;
	unsigned char data;
	int bandwidth = 0;

	for (i = 0; i < ARRAY_SIZE(bandwidth_delay_table) ; i++) {
		if (bandwidth_delay_table[i].delay_ms > delay_ms)
			break;
	}

	if (i > 0)
		i--;

	bandwidth = bandwidth_delay_table[i].bandwidth;

	ret = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG, &data);
	if (ret < 0)
		return ret;

	data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, bandwidth);

	ret = bma2x2_smbus_write_byte(client, BMA2X2_BANDWIDTH__REG, &data);

	return ret;
}

static int bma2x2_set_range(struct i2c_client *client, unsigned char Range)
{
	int ret = -1;
	unsigned char data1;

	ret = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL_REG, &data1);
	if (ret < 0)
		return ret;

	switch (Range) {
	case BMA2X2_RANGE_2G:
		data1  = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_RANGE_SEL, 3);
		break;
	case BMA2X2_RANGE_4G:
		data1  = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_RANGE_SEL, 5);
		break;
	case BMA2X2_RANGE_8G:
		data1  = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_RANGE_SEL, 8);
		break;
	case BMA2X2_RANGE_16G:
		data1  = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_RANGE_SEL, 12);
		break;
	default:
		return ret;
		break;
	}

	ret = bma2x2_smbus_write_byte(client, BMA2X2_RANGE_SEL_REG, &data1);
	return ret;
}

static int bma2x2_read_accel_xyz(struct i2c_client *client,
		signed char sensor_type, struct bma2x2_acc *acc)
{
	int comres;
	s16 hw_d[3];
	unsigned char data[6];
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	switch (sensor_type) {
	case 0:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X12_LSB__REG, data, 6);
		hw_d[0] = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X12_LSB__LEN));
		hw_d[0] = hw_d[0] << (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		hw_d[0] = hw_d[0] >> (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		hw_d[1] = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y12_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y12_LSB__LEN));
		hw_d[1] = hw_d[1] << (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		hw_d[1] = hw_d[1] >> (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		hw_d[2] = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z12_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z12_LSB__LEN));
		hw_d[2] = hw_d[2] << (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		hw_d[2] = hw_d[2] >> (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 1:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X10_LSB__REG, data, 6);
		hw_d[0] = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X10_LSB__LEN));
		hw_d[0] = hw_d[0] << (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		hw_d[0] = hw_d[0] >> (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		hw_d[1] = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y10_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y10_LSB__LEN));
		hw_d[1] = hw_d[1] << (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		hw_d[1] = hw_d[1] >> (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		hw_d[2] = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z10_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z10_LSB__LEN));
		hw_d[2] = hw_d[2] << (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		hw_d[2] = hw_d[2] >> (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 2:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X8_LSB__REG, data, 6);
		hw_d[0] = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X8_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X8_LSB__LEN));
		hw_d[0] = hw_d[0] << (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		hw_d[0] = hw_d[0] >> (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		hw_d[1] = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y8_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y8_LSB__LEN));
		hw_d[1] = hw_d[1] << (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		hw_d[1] = hw_d[1] >> (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		hw_d[2] = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z8_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z8_LSB__LEN));
		hw_d[2] = hw_d[2] << (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		hw_d[2] = hw_d[2] >> (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 3:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X14_LSB__REG, data, 6);
		hw_d[0] = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X14_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X14_LSB__LEN));
		hw_d[0] = hw_d[0] << (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));
		hw_d[0] = hw_d[0] >> (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN +
					BMA2X2_ACC_X_MSB__LEN));

		hw_d[1] = BMA2X2_GET_BITSLICE(data[2], BMA2X2_ACC_Y14_LSB)|
			(BMA2X2_GET_BITSLICE(data[3],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y14_LSB__LEN
									));
		hw_d[1] = hw_d[1] << (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));
		hw_d[1] = hw_d[1] >> (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN +
					BMA2X2_ACC_Y_MSB__LEN));

		hw_d[2] = BMA2X2_GET_BITSLICE(data[4], BMA2X2_ACC_Z14_LSB)|
			(BMA2X2_GET_BITSLICE(data[5],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z14_LSB__LEN));
		hw_d[2] = hw_d[2] << (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		hw_d[2] = hw_d[2] >> (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN +
					BMA2X2_ACC_Z_MSB__LEN));
		break;
	default:
		return -1;
		break;
	}

	acc->x = ((bma2x2->pdata->negate_x) ? (-hw_d[bma2x2->pdata->axis_map_x])
		   : (hw_d[bma2x2->pdata->axis_map_x]));
	acc->y = ((bma2x2->pdata->negate_y) ? (-hw_d[bma2x2->pdata->axis_map_y])
		   : (hw_d[bma2x2->pdata->axis_map_y]));
	acc->z = ((bma2x2->pdata->negate_z) ? (-hw_d[bma2x2->pdata->axis_map_z])
		   : (hw_d[bma2x2->pdata->axis_map_z]));

	return comres;
}

static void bma2x2_report_event(struct bma2x2_data *bma2x2)
{
	input_report_abs(bma2x2->input, ABS_RX, bma2x2->value.x);
	input_report_abs(bma2x2->input, ABS_RY, bma2x2->value.y);
	input_report_abs(bma2x2->input, ABS_RZ, bma2x2->value.z);
	input_sync(bma2x2->input);
}

static void bma2x2_delay_work_func(struct work_struct *work)
{
	struct bma2x2_data *bma2x2 = container_of((struct delayed_work *)work,
			struct bma2x2_data, work);
	struct bma2x2_acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&bma2x2->delay));

	bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type, &acc);

	mutex_lock(&bma2x2->value_mutex);
	bma2x2->value = acc;
	bma2x2_report_event(bma2x2);
	mutex_unlock(&bma2x2->value_mutex);

	schedule_delayed_work(&bma2x2->work, delay);
}

static void bma2x2_irq1_work_func(struct work_struct *work)
{
	struct bma2x2_data *bma2x2 = container_of(work, struct bma2x2_data, irq1_work);
	struct bma2x2_acc acc;

	bma2x2_read_accel_xyz(bma2x2->bma2x2_client, bma2x2->sensor_type, &acc);

	mutex_lock(&bma2x2->value_mutex);
	bma2x2->value = acc;
	bma2x2_report_event(bma2x2);
	mutex_unlock(&bma2x2->value_mutex);

	enable_irq(bma2x2->irq1);
}

static irqreturn_t bma2x2_irq1_handler(int irq, void *dev)
{
	struct bma2x2_data *data = dev;
	disable_irq_nosync(irq);
	queue_work(data->irq1_work_queue, &data->irq1_work);

	return IRQ_HANDLED;
}

static ssize_t bma2x2_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma2x2->delay));

}

static ssize_t bma2x2_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if (data > BMA2X2_MAX_DELAY)
		data = BMA2X2_MAX_DELAY;

	bma2x2_set_bandwidth(bma2x2->bma2x2_client, data);
	atomic_set(&bma2x2->delay, (unsigned int) data);

	return count;
}

static ssize_t bma2x2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&bma2x2->enable));

}

static void bma2x2_set_enable(struct device *dev, int enable)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	mutex_lock(&bma2x2->enable_mutex);
	if (enable) {
		bma2x2_set_mode(bma2x2->bma2x2_client, BMA2X2_MODE_NORMAL);
		if (bma2x2->irq1 >= 0) {
			enable_irq(bma2x2->irq1);
			bma2x2_set_int_enable(bma2x2->bma2x2_client);
		} else
			schedule_delayed_work(&bma2x2->work,
				msecs_to_jiffies(atomic_read(&bma2x2->delay)));

		atomic_set(&bma2x2->enable, 1);
	} else {
		bma2x2_set_mode(bma2x2->bma2x2_client, BMA2X2_MODE_SUSPEND);
		if (bma2x2->irq1 >= 0) {
			cancel_work_sync(&bma2x2->irq1_work);
			disable_irq(bma2x2->irq1);
		} else {
			cancel_delayed_work_sync(&bma2x2->work);
		}

		atomic_set(&bma2x2->enable, 0);
	}
	mutex_unlock(&bma2x2->enable_mutex);
}

static ssize_t bma2x2_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = strict_strtoul(buf, 10, &data);
	if (error)
		return error;

	if ((data == 0) || (data == 1))
		bma2x2_set_enable(dev, data);

	return count;
}

static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		bma2x2_delay_show, bma2x2_delay_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		bma2x2_enable_show, bma2x2_enable_store);

static struct attribute *bma2x2_attributes[] = {
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group bma2x2_attribute_group = {
	.attrs = bma2x2_attributes
};

static int bma2x2_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	unsigned char chip_id;
	struct bma2x2_data *data;
	struct input_dev *dev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO "i2c_check_functionality error\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct bma2x2_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	data->pdata = kmalloc(sizeof(*data->pdata), GFP_KERNEL);
	if (data->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for pdata: %d\n", err);
		goto kfree_exit;
	}
	memcpy(data->pdata, client->dev.platform_data, sizeof(*data->pdata));

	/* read chip id */
	bma2x2_smbus_read_byte(client, BMA2X2_CHIP_ID_REG, &chip_id);
	if (chip_id < 0) {
		err = -ENOMEM;
		goto kfree_exit;
	}

	chip_id = i2c_smbus_read_byte_data(client, BMA2X2_CHIP_ID_REG);
	data->sensor_type = bma2x2_get_chip_type(chip_id);
	if (data->sensor_type != -1) {
		data->chip_id = chip_id;
		printk(KERN_INFO "Bosch Sensortec Device detected!\n"
				"%s registered I2C driver!\n",
						sensor_name[data->sensor_type]);
	} else {
		printk(KERN_INFO "Bosch Sensortec Device not found"
				"i2c error %d \n", chip_id);
		err = -ENODEV;
		goto kfree_exit;
	}

	i2c_set_clientdata(client, data);
	data->bma2x2_client = client;

	mutex_init(&data->value_mutex);
	mutex_init(&data->mode_mutex);
	mutex_init(&data->enable_mutex);

	bma2x2_set_range(client, BMA2X2_RANGE_SET);

	atomic_set(&data->delay, BMA2X2_MAX_DELAY);
	atomic_set(&data->enable, 0);

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "acc";
	dev->id.bustype = BUS_I2C;
	dev->id.product = chip_id;

	set_bit(EV_REL, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_RX);
	input_set_capability(dev, EV_ABS, ABS_RY);
	input_set_capability(dev, EV_ABS, ABS_RZ);
	input_set_abs_params(dev, ABS_RX, RELMIN, RELMAX, 0, 0);
	input_set_abs_params(dev, ABS_RY, RELMIN, RELMAX, 0, 0);
	input_set_abs_params(dev, ABS_RZ, RELMIN, RELMAX, 0, 0);

	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		goto kfree_exit;
	}

	data->input = dev;

	err = sysfs_create_group(&data->input->dev.kobj,
			&bma2x2_attribute_group);
	if (err < 0) {
		printk(KERN_INFO "Bosch Sensortec Device create sysfs error");
		goto error_sysfs;
	}

	if (data->pdata->gpio_int1 >= 0) {
		data->irq1 = gpio_to_irq(data->pdata->gpio_int1);
	} else {
		goto use_poll;
	}

	if (data->pdata->gpio_int1 >= 0) {
		err = gpio_request(data->pdata->gpio_int1,"bma2x2_int1");
		if (err) {
			pr_err("%s: unable to request interrupt gpio %d\n",
				__func__, data->pdata->gpio_int1);
			goto use_poll;
		}

		err = gpio_direction_input(data->pdata->gpio_int1);
		if (err) {
			pr_err("%s: unable to set direction for gpio %d\n",
			__func__, data->pdata->gpio_int1);
			gpio_free(data->pdata->gpio_int1);
			goto use_poll;
		}

		INIT_WORK(&data->irq1_work, bma2x2_irq1_work_func);
		data->irq1_work_queue =
			create_singlethread_workqueue("bma2x2_wq1");
		if (!data->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto use_poll;
		}

		err = request_irq(data->irq1, bma2x2_irq1_handler,
				IRQF_TRIGGER_RISING, "bma2x2_irq1_handler", data);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto use_poll;
		}
		disable_irq_nosync(data->irq1);

	}

	return 0;

use_poll:
	data->irq1 = -1;
	INIT_DELAYED_WORK(&data->work, bma2x2_delay_work_func);
	return 0;

error_sysfs:
	input_unregister_device(data->input);
kfree_exit:
	kfree(data);
exit:
	return err;
}

static int __devexit bma2x2_remove(struct i2c_client *client)
{
	struct bma2x2_data *data = i2c_get_clientdata(client);

	bma2x2_set_enable(&client->dev, 0);
	sysfs_remove_group(&data->input->dev.kobj, &bma2x2_attribute_group);
	input_unregister_device(data->input);
	kfree(data);

	return 0;
}

static int bma2x2_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	mutex_lock(&bma2x2->enable_mutex);
	if (atomic_read(&bma2x2->enable) == 1) {
		bma2x2_set_mode(bma2x2->bma2x2_client, BMA2X2_MODE_SUSPEND);
		if (bma2x2->irq1 >= 0) {
			cancel_work_sync(&bma2x2->irq1_work);
			disable_irq(bma2x2->irq1);
		} else {
			cancel_delayed_work_sync(&bma2x2->work);
		}
	}
	mutex_unlock(&bma2x2->enable_mutex);

	return 0;
}

static int bma2x2_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	mutex_lock(&bma2x2->enable_mutex);
	if (atomic_read(&bma2x2->enable) == 1) {
		bma2x2_set_mode(bma2x2->bma2x2_client, BMA2X2_MODE_NORMAL);
		if (bma2x2->irq1 >= 0)
			enable_irq(bma2x2->irq1);
		else
			schedule_delayed_work(&bma2x2->work,
				msecs_to_jiffies(atomic_read(&bma2x2->delay)));
	}
	mutex_unlock(&bma2x2->enable_mutex);

	return 0;
}

static const struct i2c_device_id bma2x2_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma2x2_id);

static const struct dev_pm_ops bma2x2_pm_ops = {
	.suspend = bma2x2_suspend,
	.resume = bma2x2_resume,
};

static struct i2c_driver bma2x2_driver = {
	.probe		= bma2x2_probe,
	.remove		= __devexit_p(bma2x2_remove),
	.driver = {
		.owner	= THIS_MODULE,
		.name	= SENSOR_NAME,
		.pm = &bma2x2_pm_ops,
	},
	.id_table	= bma2x2_id,
};

static int __init BMA2X2_init(void)
{
	return i2c_add_driver(&bma2x2_driver);
}

static void __exit BMA2X2_exit(void)
{
	i2c_del_driver(&bma2x2_driver);
}

MODULE_DESCRIPTION("BMA2X2 accelerometer sensor driver");
MODULE_LICENSE("GPL");

module_init(BMA2X2_init);
module_exit(BMA2X2_exit);

