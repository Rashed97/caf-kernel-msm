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

//#define AL3010_DRV_NAME	"al3010"
#define AL3010_DRV_NAME		"dyna"
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

static int al3010_range[4] = {77806,19452,4863,1216};

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
	int irq;
	struct input_dev *input;
};

int cali = 100;

#define ADD_TO_IDX(addr,idx)	{														\
									int i;												\
									for(i = 0; i < AL3010_NUM_CACHABLE_REGS; i++)		\
									{													\
										if (addr == al3010_reg[i])						\
										{												\
											idx = i;									\
											break;										\
										}												\
									}													\
								}

/*
 * register access helpers
 */

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
static int al3010_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = __al3010_read_reg(client, AL3010_ALS_COMMAND,
											AL3010_RAN_MASK, AL3010_RAN_SHIFT);;
	return al3010_range[tmp];
}

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
static int al3010_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __al3010_read_reg(client, AL3010_ALS_LTHL,
				AL3010_ALS_LTHL_MASK, AL3010_ALS_LTHL_SHIFT);
	msb = __al3010_read_reg(client, AL3010_ALS_LTHH,
				AL3010_ALS_LTHH_MASK, AL3010_ALS_LTHH_SHIFT);
	return ((msb << 8) | lsb);
}

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
static int al3010_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	lsb = __al3010_read_reg(client, AL3010_ALS_HTHL,
				AL3010_ALS_HTHL_MASK, AL3010_ALS_HTHL_SHIFT);
	msb = __al3010_read_reg(client, AL3010_ALS_HTHH,
				AL3010_ALS_HTHH_MASK, AL3010_ALS_HTHH_SHIFT);
	return ((msb << 8) | lsb);
}

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
static int al3010_set_power_state(struct i2c_client *client, int state)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
				AL3010_POW_MASK, AL3010_POW_SHIFT, 
				state ? AL3010_POW_UP : AL3010_POW_DOWN);
}

static int al3010_get_power_state(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[AL3010_MODE_COMMAND];
	return (cmdreg & AL3010_POW_MASK) >> AL3010_POW_SHIFT;
}

static int al3010_get_adc_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range, val;

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

	range = al3010_get_range(client);
	val = (((msb << 8) | lsb) * range) >> 16;
	val *= cali;

	return (val / 100);
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
    dev->name = "LSC_al3010";
    dev->id.bustype = BUS_I2C;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_RUDDER);
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

/* range */
static ssize_t al3010_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	
	return sprintf(buf, "%i\n", al3010_get_range(data->client));
}

static ssize_t al3010_store_range(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
		return -EINVAL;

	ret = al3010_set_range(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   al3010_show_range, al3010_store_range);


/* mode */
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
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 5))
		return -EINVAL;

	ret = al3010_set_mode(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(mode, S_IWUSR | S_IRUGO,
		   al3010_show_mode, al3010_store_mode);


/* power state */
static ssize_t al3010_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	
	return sprintf(buf, "%d\n", al3010_get_power_state(data->client));
}

static ssize_t al3010_store_power_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

	ret = al3010_set_power_state(data->client, val);
	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   al3010_show_power_state, al3010_store_power_state);


/* lux */
static ssize_t al3010_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);

	/* No LUX data if not operational */
	if (al3010_get_power_state(data->client) != 0x01)
		return sprintf((char*)buf, "%s\n", "Please power up first!");

	return sprintf(buf, "%d\n", al3010_get_adc_value(data->client));
}

static DEVICE_ATTR(lux, S_IRUGO, al3010_show_lux, NULL);

/* ALS low threshold */
static ssize_t al3010_show_althres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", al3010_get_althres(data->client));
}

static ssize_t al3010_store_althres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = al3010_set_althres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
		   al3010_show_althres, al3010_store_althres);


/* ALS high threshold */
static ssize_t al3010_show_ahthres(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	return sprintf(buf, "%d\n", al3010_get_ahthres(data->client));
}

static ssize_t al3010_store_ahthres(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	unsigned long val;
	int ret;

	if (strict_strtoul(buf, 10, &val) < 0)
		return -EINVAL;

	ret = al3010_set_ahthres(data->client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
		   al3010_show_ahthres, al3010_store_ahthres);


/* calibration */
static ssize_t al3010_show_calibration_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sprintf(buf, "%d\n", cali);
}

static ssize_t al3010_store_calibration_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct input_dev *input = to_input_dev(dev);
	struct al3010_data *data = input_get_drvdata(input);
	int stdls, lux; 
	char tmp[10];

	/* No LUX data if not operational */
	if (al3010_get_power_state(data->client) != 0x01)
	{
		printk("Please power up first!");
		return -EINVAL;
	}

	cali = 100;
	sscanf(buf, "%d %s", &stdls, tmp);

	if (!strncmp(tmp, "-setcv", 6))
	{
		cali = stdls;
		return -EBUSY;
	}

	if (stdls < 0)
	{
		printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
		return -EBUSY;
	}

	lux = al3010_get_adc_value(data->client);
	cali = stdls * 100 / lux;

	return -EBUSY;
}

static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
		   al3010_show_calibration_state, al3010_store_calibration_state);


#ifdef LSC_DBG
/* engineer mode */
static ssize_t al3010_em_read(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct al3010_data *data = i2c_get_clientdata(client);
	int i;
	u8 tmp;
	
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
	{
		mutex_lock(&data->lock);
		tmp = i2c_smbus_read_byte_data(data->client, al3010_reg[i]);
		mutex_unlock(&data->lock);

		printk("Reg[0x%x] Val[0x%x]\n", al3010_reg[i], tmp);
	}

	return 0;
}

static ssize_t al3010_em_write(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct al3010_data *data = i2c_get_clientdata(client);
	u32 addr,val;
	int ret = 0;

	sscanf(buf, "%x%x", &addr, &val);

	printk("Write [%x] to Reg[%x]...\n",val,addr);
	mutex_lock(&data->lock);

	ret = i2c_smbus_write_byte_data(data->client, addr, val);
	if (!ret)
		data->reg_cache[addr] = val;

	mutex_unlock(&data->lock);

	return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
				   al3010_em_read, al3010_em_write);
#endif

static struct attribute *al3010_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_mode.attr,
	&dev_attr_power_state.attr,
	&dev_attr_lux.attr,
	&dev_attr_althres.attr,
	&dev_attr_ahthres.attr,
	&dev_attr_calibration.attr,
#ifdef LSC_DBG
	&dev_attr_em.attr,
#endif
	NULL
};

static const struct attribute_group al3010_attr_group = {
	.attrs = al3010_attributes,
};

static int al3010_init_client(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, al3010_reg[i]);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	al3010_set_range(client, 0);
	al3010_set_mode(client, 0);
	al3010_set_power_state(client, 0);

	return 0;
}

static irqreturn_t al3010_irq(int irq, void *data_)
{
	struct al3010_data *data = data_;
    int Aval;
	
	Aval = al3010_get_adc_value(data->client);
	printk("ALS lux value: %d\n", Aval);
	
    return IRQ_HANDLED;
}

/*
 * I2C layer
 */

static int __devinit al3010_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);
	data->irq = client->irq;

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

	err = request_threaded_irq(client->irq, NULL, al3010_irq,
                               IRQF_TRIGGER_FALLING,
                               "al3010", data);
    if (err) {
		dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
            goto exit_input;
    }

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
	free_irq(data->irq, data);
	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	al3010_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));
	return 0;
}

#ifdef CONFIG_PM
static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct al3010_data *data = i2c_get_clientdata(client);

	data->power_state_before_suspend = al3010_get_power_state(client);
	
	if(device_may_wakeup(&client->dev))
		enable_irq_wake(data->irq);

	return al3010_set_power_state(client, 0);
}

static int al3010_resume(struct i2c_client *client)
{
	int i;
	struct al3010_data *data = i2c_get_clientdata(client);

	/* restore registers from cache */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
		if (i2c_smbus_write_byte_data(client, i, data->reg_cache[i]))
			return -EIO;

	if(device_may_wakeup(&client->dev))
		disable_irq_wake(data->irq);

	return al3010_set_power_state(client,
		data->power_state_before_suspend);
}

#else
#define al3010_suspend	NULL
#define al3010_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id al3010_id[] = {
	//{ "al3010", 0 },
	{ "dyna", 0 },
	{}
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

