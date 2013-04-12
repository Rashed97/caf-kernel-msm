/*
 * This file is part of the AL3010 sensor driver.
 *
 * Copyright (c) 2011 Liteon-semi Corporation
 *
 * Contact: YC Hou <yc_hou@liteon-semi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: al3010.c
 *
 * Summary:
 *	AL3010 sensor dirver for kernel version 3.0.8.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 06/13/11 YC		 Original Creation (Test version:1.9)
 *
 */


#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/input/al3010.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>


#define AL3010_DRV_NAME	"al3010"
#define DRIVER_VERSION		"1.9"

#define AL3010_NUM_CACHABLE_REGS	9

#define	AL3010_ALS_COMMAND	0x10
#define	AL3010_RAN_MASK	0x70
#define	AL3010_RAN_SHIFT	(4)

#define AL3010_MODE_COMMAND	0x00
#define AL3010_MODE_SHIFT	(0)
#define AL3010_MODE_MASK	0x07

#define AL3010_POW_MASK		0x01
#define AL3010_POW_UP		0x01
#define AL3010_POW_DOWN		0x00
#define AL3010_POW_SHIFT	(0)

#define AL3010_ALS_LTHL			0x1a
#define AL3010_ALS_LTHL_SHIFT	(0)
#define AL3010_ALS_LTHL_MASK	0xff

#define AL3010_ALS_LTHH			0x1b
#define AL3010_ALS_LTHH_SHIFT	(0)
#define AL3010_ALS_LTHH_MASK	0xff

#define AL3010_ALS_HTHL			0x1c
#define AL3010_ALS_HTHL_SHIFT	(0)
#define AL3010_ALS_HTHL_MASK	0xff

#define AL3010_ALS_HTHH			0x1d
#define AL3010_ALS_HTHH_SHIFT	(0)
#define AL3010_ALS_HTHH_MASK	0xff

#define	AL3010_ADC_LSB	0x0c
#define	AL3010_ADC_MSB	0x0d

static u8 al3010_reg[AL3010_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x0c,0x0d,0x10,0x1a,0x1b,0x1c,0x1d};

//#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif

#define AL3010_MAX_DELAY                1000

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	struct input_dev *input;
	atomic_t delay;
	atomic_t enable;
	struct mutex enable_mutex;
	struct delayed_work work;
};

struct sensor_regulator {
	struct regulator *vreg;
	const char *name;
	u32	min_uV;
	u32	max_uV;
};

struct sensor_regulator al3010_vreg[] = {
	{NULL, "als_vdd", 2850000, 2850000},
	{NULL, "i2c_src", 1800000, 1800000},
	{NULL, "vcc_i2c", 0, 0},
	{NULL, "als_i2c", 0, 0},
};

int cali = 100;


#define ADD_TO_IDX(addr,idx)	{												\
					int i;											\
					for(i = 0; i < AL3010_NUM_CACHABLE_REGS; i++)						\
					{											\
						if (addr == al3010_reg[i])							\
						{										\
							idx = i;								\
							break;									\
						}										\
					}											\
				}
/*
 * register access helpers
 */

static int al3010_config_regulator(struct i2c_client *client, bool on)
{
	int rc = 0, i;
	int num_reg = sizeof(al3010_vreg) / sizeof(struct sensor_regulator);

	if (on) {
		for (i = 0; i < num_reg; i++) {
			al3010_vreg[i].vreg = regulator_get(&client->dev,
						al3010_vreg[i].name);
			if (IS_ERR(al3010_vreg[i].vreg)) {
				rc = PTR_ERR(al3010_vreg[i].vreg);
				pr_err("%s:regulator get failed rc=%d\n",
						__func__, rc);
				goto error_vdd;
			}

			if (regulator_count_voltages(al3010_vreg[i].vreg) > 0) {
				rc = regulator_set_voltage(al3010_vreg[i].vreg,
					al3010_vreg[i].min_uV, al3010_vreg[i].max_uV);
				if (rc) {
					pr_err("%s:set_voltage failed rc=%d\n",
						__func__, rc);
					regulator_put(al3010_vreg[i].vreg);
					goto error_vdd;
				}
			}

			rc = regulator_enable(al3010_vreg[i].vreg);
			if (rc) {
				pr_err("%s: regulator_enable failed rc =%d\n",
						__func__,
						rc);

				if (regulator_count_voltages(
					al3010_vreg[i].vreg) > 0) {
					regulator_set_voltage(al3010_vreg[i].vreg,
						0, al3010_vreg[i].max_uV);
				}
				regulator_put(al3010_vreg[i].vreg);
				goto error_vdd;
			}
		}
		return rc;
	} else {
		i = num_reg;
	}
error_vdd:
	while (--i >= 0) {
		if (regulator_count_voltages(al3010_vreg[i].vreg) > 0) {
			regulator_set_voltage(al3010_vreg[i].vreg, 0,
						al3010_vreg[i].max_uV);
		}
		regulator_disable(al3010_vreg[i].vreg);
		regulator_put(al3010_vreg[i].vreg);
	}
	return rc;
}

static int __al3010_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __al3010_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;

	ADD_TO_IDX(reg,idx)
	if (idx >= AL3010_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[idx];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[idx] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3010_set_range(struct i2c_client *client, int range)
{
	return __al3010_write_reg(client, AL3010_ALS_COMMAND, 
		AL3010_RAN_MASK, AL3010_RAN_SHIFT, range);
}


/* mode */
static int al3010_get_mode(struct i2c_client *client)
{
	return __al3010_read_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT);
}

static int al3010_set_mode(struct i2c_client *client, int mode)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT, mode);
}

/* ALS low threshold */
static int al3010_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AL3010_ALS_LTHL_MASK;

	err = __al3010_write_reg(client, AL3010_ALS_LTHL,
		AL3010_ALS_LTHL_MASK, AL3010_ALS_LTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3010_write_reg(client, AL3010_ALS_LTHH,
		AL3010_ALS_LTHH_MASK, AL3010_ALS_LTHH_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int al3010_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	
	msb = val >> 8;
	lsb = val & AL3010_ALS_HTHL_MASK;
	
	err = __al3010_write_reg(client, AL3010_ALS_HTHL,
		AL3010_ALS_HTHL_MASK, AL3010_ALS_HTHL_SHIFT, lsb);
	if (err)
		return err;

	err = __al3010_write_reg(client, AL3010_ALS_HTHH,
		AL3010_ALS_HTHH_MASK, AL3010_ALS_HTHH_SHIFT, msb);

	return err;
}

/* power_state */
static int al3010_get_adc_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, val;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	val = (msb << 8) | lsb;

	return val;
}

static int al3010_init_client(struct i2c_client *client)
{
	/* set defaults */
	al3010_set_range(client, 2);
	al3010_set_mode(client, 0);
	al3010_set_althres(client, 0);
	al3010_set_ahthres(client, 0);

	return 0;
}

static void al3010_updata_value(struct al3010_data *data)
{
	int value;
	struct input_dev *input_data;

	value = al3010_get_adc_value(data->client);

	LDBG("al3010 lux value: %d\n", value);

	input_data = data->input;
	input_report_abs(input_data, ABS_MISC, value);
	input_sync(input_data);
}

static void al3010_work_func(struct work_struct *work)
{
	struct al3010_data *data = container_of((struct delayed_work *)work,struct al3010_data, work);
	unsigned long delay = msecs_to_jiffies(atomic_read(&data->delay));	
	
	al3010_updata_value(data);
	schedule_delayed_work(&data->work, delay);
}

/*
 * sysfs layer
 */
static int al3010_input_init(struct al3010_data *data)
{
	struct input_dev *dev;
	int err;

	dev = input_allocate_device();
	if (!dev) {
		return -ENOMEM;
	}
	dev->name = "light";
	dev->id.bustype = BUS_I2C;
	
	input_set_capability(dev, EV_ABS, ABS_MISC);
	//input_set_capability(dev, EV_ABS, ABS_RUDDER);
	input_set_abs_params(dev, ABS_MISC, 0, 1, 0, 0);
	input_set_drvdata(dev, data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		LDBG("input device register error! ret = [%d]\n", err)
			return err;
	}
	data->input = dev;

	return 0;
}

static void al3010_input_fini(struct al3010_data *data)
{
    struct input_dev *dev = data->input;

    input_unregister_device(dev);
	input_free_device(dev);
}

static ssize_t al3010_show_delay(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	
	return sprintf(buf, "%d\n", atomic_read(&data->delay));
}

static ssize_t al3010_store_delay(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	long long val;

	if ((strict_strtoull(buf, 10, &val) < 0))
		return -EINVAL;

	if (val > 1000000000)
		val = 1000000000;
	else if (val < 1000000)
		val = 1000000;

	atomic_set(&data->delay, (int)val/1000000);

	return count;
}

static DEVICE_ATTR(poll_delay, S_IWUSR | S_IRUGO,
		   al3010_show_delay, al3010_store_delay);

static ssize_t al3010_show_mode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	
	return sprintf(buf, "%d\n", al3010_get_mode(data->client));
}

static ssize_t al3010_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret = 0;

	if ((strict_strtoul(buf, 10, &val) < 0))
		return -EINVAL;

	if (val !=0)
		val = 1;

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) != val) {
		if (val) {
			al3010_config_regulator(data->client, 1);
			msleep(300);
			al3010_init_client(data->client);
			al3010_set_mode(data->client, val);
			schedule_delayed_work(&data->work, msecs_to_jiffies(atomic_read(&data->delay)));
		} else {
			cancel_delayed_work_sync(&data->work);
			al3010_set_mode(data->client, val);
			al3010_config_regulator(data->client, 0);
		}
		atomic_set(&data->enable, val);
	}
	mutex_unlock(&data->enable_mutex);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(enable, S_IWUSR | S_IRUGO,
		   al3010_show_mode, al3010_store_mode);

static struct attribute *al3010_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
	.attrs = al3010_attributes,
};

/*
 * I2C layer
 */

static int __devinit al3010_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *data;
	int err = 0;

	dev_info(&client->dev, "al3010_probe\n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	mutex_init(&data->enable_mutex);

	atomic_set(&data->enable, 0);
	atomic_set(&data->delay, AL3010_MAX_DELAY);

	/* initialize the AL3010 chip */
	err = al3010_init_client(client);
	if (err)
		goto exit_kfree;

	err = al3010_input_init(data);
	if (err)
		goto exit_kfree;

	/* register sysfs hooks */
	err = sysfs_create_group(&data->input->dev.kobj, &al3010_attr_group);
	if (err)
		goto exit_input;

	INIT_DELAYED_WORK(&data->work, al3010_work_func);

	dev_info(&client->dev, "AL3010 driver version %s enabled\n", DRIVER_VERSION);
	return 0;

exit_input:
	al3010_input_fini(data);

exit_kfree:
	kfree(data);
	return err;
}

static int __devexit al3010_remove(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&data->work);
	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	kfree(i2c_get_clientdata(client));
	return 0;
}

#ifdef CONFIG_PM
static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct al3010_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		cancel_delayed_work_sync(&data->work);
		al3010_set_mode(data->client, 0);
		al3010_config_regulator(data->client, 0);
	}
	mutex_unlock(&data->enable_mutex);
	return 0;
}

static int al3010_resume(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->enable_mutex);
	if (atomic_read(&data->enable) == 1) {
		al3010_config_regulator(data->client, 1);
		msleep(300);
		al3010_init_client(data->client);
		al3010_set_mode(data->client, 1);
		schedule_delayed_work(&data->work, msecs_to_jiffies(atomic_read(&data->delay)));
	}
	mutex_unlock(&data->enable_mutex);
	return 0;
}

#else
#define al3010_suspend	NULL
#define al3010_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id al3010_id[] = {
	{ "al3010", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static struct i2c_driver al3010_driver = {
	.driver = {
		.name	= AL3010_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = al3010_suspend,
	.resume	= al3010_resume,
	.probe	= al3010_probe,
	.remove	= __devexit_p(al3010_remove),
	.id_table = al3010_id,
};

static int __init al3010_init(void)
{
	return i2c_add_driver(&al3010_driver);
}

static void __exit al3010_exit(void)
{
	i2c_del_driver(&al3010_driver);
}

MODULE_AUTHOR("YC Hou, LiteOn-semi corporation.");
MODULE_DESCRIPTION("Test AL3010 driver on mini6410.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3010_init);
module_exit(al3010_exit);

