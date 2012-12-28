/* hscdtd007a_i2c.c
 *
 * GeoMagneticField device driver for I2C (HSCDTD007/HSCDTD008)
 *
 * Copyright (C) 2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/input/hscdtd007a_i2c.h>

#define DELAY_MAX        2000

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE
#define EVENT_TYPE_MAGV_STATUS      ABS_THROTTLE

#define ALPS_INPUT_FUZZ       0  /* input event threshold */
#define ALPS_INPUT_FLAT       0

struct hscd_axis {
    s16 x;
    s16 y;
    s16 z;
};

struct hscd_data {
    struct i2c_client *client;
    struct hscd_platform_data *pdata;
    struct input_dev *input_dev;
    struct delayed_work work;
    int irq;
    struct mutex enable_mutex;
    struct work_struct irq_work;
    struct workqueue_struct *irq_work_queue;
    struct hscd_axis values;
    atomic_t enable;
    atomic_t delay;
};

struct output_rate_t {
    u8 output_rate;         /* bandwith reg setting */
    unsigned long delay_ms; /* bandwith in ms */
};

static const struct output_rate_t output_rate_table[] = {
    {ODR_100,   10}, /*   100Hz */
    {ODR_20,    50}, /*    20Hz */
    {ODR_10,   100}, /*    10Hz */
    {ODR_0_5, 2000}, /*   0.5Hz */
};

/*-----------------------------------------------------------------------------------------------*/
/* i2c read/erite function                                                                       */
/*-----------------------------------------------------------------------------------------------*/
static int hscd_i2c_readm(struct i2c_client *client, char *rxData, int length)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr  = client->addr,
            .flags = 0,
            .len   = 1,
            .buf   = rxData,
        },
        {
            .addr  = client->addr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxData,
         },
    };

    do {
        err = i2c_transfer(client->adapter, msgs, 2);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        dev_err(&client->adapter->dev, "read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int hscd_i2c_writem(struct i2c_client *client, char *txData, int length)
{
    int err;
    int tries = 0;
#ifdef ALPS_DEBUG
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr  = client->addr,
            .flags = 0,
            .len   = length,
            .buf   = txData,
         },
    };

#ifdef ALPS_DEBUG
    printk("[HSCD] i2c_writem : ");
    for (i=0; i<length;i++) printk("0X%02X, ", txData[i]);
    printk("\n");
#endif

    do {
        err = i2c_transfer(client->adapter, msg, 1);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        dev_err(&client->adapter->dev, "write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

/*-----------------------------------------------------------------------------------------------*/
/* hscdtd function                                                                               */
/*-----------------------------------------------------------------------------------------------*/
int hscd_get_magnetic_data(struct i2c_client *client, struct hscd_axis *axis)
{
    int err = -1;
    s16 hw_d[3];
    u8 sx[6];
    struct hscd_data *data = i2c_get_clientdata(client);

    sx[0] = HSCD_XOUT;
    err = hscd_i2c_readm(client, sx, 6);
    if (err < 0) return err;

    hw_d[0] = (short)((sx[1] << 8) | (sx[0]));
    hw_d[1] = (short)((sx[3] << 8) | (sx[2]));
    hw_d[2] = (short)((sx[5] << 8) | (sx[4]));

    axis->x = ((data->pdata->negate_x) ? (-hw_d[data->pdata->axis_map_x])
           : (hw_d[data->pdata->axis_map_x]));
    axis->y = ((data->pdata->negate_y) ? (-hw_d[data->pdata->axis_map_y])
           : (hw_d[data->pdata->axis_map_y]));
    axis->z = ((data->pdata->negate_z) ? (-hw_d[data->pdata->axis_map_z])
           : (hw_d[data->pdata->axis_map_z]));
#ifdef ALPS_DEBUG
    printk("Mag_I2C, x:%d, y:%d, z:%d\n", axis->x, axis->y, axis->z);
#endif

    return err;
}


void hscd_set_delay(struct i2c_client *client, unsigned long delay_ms)
{
    u8 buf[2];
    u8 time_delay;
    int i;
    struct hscd_data *data = i2c_get_clientdata(client);

    for (i = 0; i < ARRAY_SIZE(output_rate_table) ; i++) {
        if (output_rate_table[i].delay_ms > delay_ms)
            break;
    }

    if (i > 0)
        i--;

    time_delay = output_rate_table[i].output_rate;

    buf[0] = HSCD_CTRL1;
    buf[1] = (atomic_read(&data->enable) << 7) | time_delay;
    hscd_i2c_writem(client, buf, 2);
}

void hscd_set_enable(struct i2c_client *client, int enable)
{
    u8 buf[2];
    u8 time_delay;

    if (enable != 0)
        enable = 1;

    if (enable) {
        buf[0] = HSCD_CTRL4;                       // 15 bit signed value
        buf[1] = 0x90;
        hscd_i2c_writem(client, buf, 2);
    }

    buf[0] = HSCD_CTRL1;
    hscd_i2c_readm(client, buf, 1);
    time_delay = buf[0] & OUTPUT_RATE_MASK;

    buf[0] = HSCD_CTRL1;
    buf[1] = (enable<<7) | time_delay;
    hscd_i2c_writem(client, buf, 2);
}

static int hscd_register_init(struct hscd_data *data)
{
    int ret = 0;
    u8  buf[2];
    u16 chip_info;

#ifdef ALPS_DEBUG
    printk("[HSCD] register_init\n");
#endif

    buf[0] = HSCD_INFO;
    ret = hscd_i2c_readm(data->client, buf, 2);
    if (ret < 0) {
        printk(KERN_INFO "[HSCD] read error.\n");
        return ret;
    }

    chip_info = (u16)((buf[1]<<8) | buf[0]);
#ifdef ALPS_DEBUG
    printk("[HSCD] chip_info, 0x%04X\n", chip_info);
#endif
    if (chip_info != HSCD_CHIP_ID) {
        printk(KERN_INFO "[HSCD] chip ID is error.\n");
        return -1;
    }

    buf[0] = HSCD_CTRL3;
    buf[1] = 0x80;
    hscd_i2c_writem(data->client, buf, 2);
    mdelay(5);

    buf[0] = HSCD_CTRL2;
    buf[1] = 0x0c;
    hscd_i2c_writem(data->client, buf, 2);

    atomic_set(&data->delay, 100);

    return 0;
}

static void hscd_report_event(struct hscd_data *data)
{
    input_report_abs(data->input_dev, EVENT_TYPE_MAGV_X, data->values.x);
    input_report_abs(data->input_dev, EVENT_TYPE_MAGV_Y, data->values.y);
    input_report_abs(data->input_dev, EVENT_TYPE_MAGV_Z, data->values.z);
    input_sync(data->input_dev);
}

static void hscd_irq_work_func(struct work_struct *work)
{
    struct hscd_data *data = container_of(work, struct hscd_data, irq_work);
    struct hscd_axis axis;

    hscd_get_magnetic_data(data->client, &axis);
    data->values = axis;

    hscd_report_event(data);
    enable_irq(data->irq);
}

static irqreturn_t hscd_irq_handler(int irq, void *dev)
{
    struct hscd_data *data = dev;
    disable_irq_nosync(irq);
    queue_work(data->irq_work_queue, &data->irq_work);

    return IRQ_HANDLED;
}

/*-----------------------------------------------------------------------------------------------*/
/* sysfs                                                                                         */
/*-----------------------------------------------------------------------------------------------*/
static ssize_t hscd_delay_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int interval_ms;
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);

    interval_ms = atomic_read(&data->delay);
    return sprintf(buf, "%d\n", interval_ms);
}

static ssize_t hscd_delay_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t size)
{
    unsigned long interval_ms;
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);

    if (strict_strtoul(buf, 10, &interval_ms) | !interval_ms)
        return -EINVAL;

    if (interval_ms > DELAY_MAX)
        interval_ms = DELAY_MAX;

    hscd_set_delay(data->client, interval_ms);
    atomic_set(&data->delay, interval_ms);

    return size;
}

static ssize_t hscd_enable_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int val;
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);

    val = atomic_read(&data->enable);
    return sprintf(buf, "%d\n", val);
}

static ssize_t hscd_enable_store(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t size)
{
    unsigned long val;
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);

    if (strict_strtoul(buf, 10, &val))
        return -EINVAL;

    mutex_lock(&data->enable_mutex);
    if (val) {
        if (atomic_read(&data->enable) == 0) {
            hscd_set_enable(data->client , val);
            atomic_set(&data->enable, 1);
            enable_irq(data->irq);
        }
    } else {
        if (atomic_read(&data->enable) == 1) {
            hscd_set_enable(data->client, 0);
            atomic_set(&data->enable, 0);
            disable_irq(data->irq);
        }
    }
    mutex_unlock(&data->enable_mutex);

    return size;
}

static DEVICE_ATTR(poll_delay, 0444, hscd_delay_show, hscd_delay_store);
static DEVICE_ATTR(enable, 0444, hscd_enable_show, hscd_enable_store);

static struct attribute *hscd_attributes[] = {
    &dev_attr_poll_delay.attr,
    &dev_attr_enable.attr,
    NULL,
};

static struct attribute_group hscd_attribute_group = {
    .attrs = hscd_attributes,
};

/*-----------------------------------------------------------------------------------------------*/
/* i2c device                                                                                    */
/*-----------------------------------------------------------------------------------------------*/
static int hscd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = -1;
    struct hscd_data *data;

    printk("<0> [HSCD] probe\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->adapter->dev, "client not i2c capable\n");
        return -ENOMEM;
    }
    data = kzalloc(sizeof(struct hscd_data), GFP_KERNEL);
    if (!data) {
        dev_err(&client->adapter->dev,
                "failed to allocate memory for module data\n");
        return -ENOMEM;
    }

    data->pdata = kmalloc(sizeof(*data->pdata), GFP_KERNEL);
    if (data->pdata == NULL) {
        ret = -ENOMEM;
        dev_err(&client->dev,
                "failed to allocate memory for pdata: %d\n", ret);
    }
    memcpy(data->pdata, client->dev.platform_data, sizeof(*data->pdata));


    i2c_set_clientdata(client, data);
    data->client = client;

    /* initialize the input class */
    data->input_dev = input_allocate_device();
    if (!data->input_dev) {
        ret = -ENOMEM;
        goto out_device;
    }
    printk(KERN_INFO "alps-init: input_allocate_device\n");

    data->input_dev->name = "compass";
    data->input_dev->id.bustype = BUS_I2C;

    input_set_capability(data->input_dev, EV_ABS, EVENT_TYPE_MAGV_X);
    input_set_capability(data->input_dev, EV_ABS, EVENT_TYPE_MAGV_Y);
    input_set_capability(data->input_dev, EV_ABS, EVENT_TYPE_MAGV_Z);
    input_set_abs_params(data->input_dev, EVENT_TYPE_MAGV_X, HSCDTD_RANGE_MIN,
            HSCDTD_RANGE_MAX, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(data->input_dev, EVENT_TYPE_MAGV_Y, HSCDTD_RANGE_MIN,
            HSCDTD_RANGE_MAX, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(data->input_dev, EVENT_TYPE_MAGV_Z, HSCDTD_RANGE_MIN,
            HSCDTD_RANGE_MAX, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);

    ret = input_register_device(data->input_dev);
    if (ret) {
        printk(KERN_INFO "alps-init: input_register_polled_device\n");
        goto out_input_dev;
    }

    ret = sysfs_create_group(&data->input_dev->dev.kobj, &hscd_attribute_group);
    if (ret) {
        printk(KERN_INFO "alps-init: sysfs_create_group\n");
    }

    if (data->pdata->gpio_int >= 0) {
        data->irq = gpio_to_irq(data->pdata->gpio_int);
        client->irq = data->irq;
        ret = gpio_request(data->pdata->gpio_int,"hscd_int");
        if (ret) {
            pr_err("%s: unable to request interrupt gpio %d\n",
                __func__, data->pdata->gpio_int);
        }

        ret = gpio_direction_input(data->pdata->gpio_int);
        if (ret) {
            pr_err("%s: unable to set direction for gpio %d\n",
            __func__, data->pdata->gpio_int);
            gpio_free(data->pdata->gpio_int);
        }

        INIT_WORK(&data->irq_work, hscd_irq_work_func);
        data->irq_work_queue =
            create_singlethread_workqueue("hscd_wq");
        if (!data->irq_work_queue) {
            ret = -ENOMEM;
            dev_err(&client->dev,
                    "cannot create work queue: %d\n", ret);
        }
        ret = request_irq(data->irq, hscd_irq_handler,
                IRQF_TRIGGER_RISING, "hscd_irq", data);
        if (ret < 0) {
            dev_err(&client->dev, "request irq failed: %d\n", ret);
        }
        disable_irq_nosync(data->irq);
    }

    i2c_set_clientdata(client, data);
    data->client = client;

    input_set_drvdata(data->input_dev, data);

    mutex_init(&data->enable_mutex);
    atomic_set(&data->enable, 0);
    atomic_set(&data->delay, 2000);

    ret = hscd_register_init(data);
    if (ret != 0) {
        return -ENODEV;
    }
#ifdef ALPS_DEBUG
    printk("%s Init end!!!!\n", __func__);
#endif

    dev_info(&client->adapter->dev, "detected HSCDTD007/008 geomagnetic field sensor\n");

    return 0;

out_input_dev:
    input_free_device(data->input_dev);
    printk(KERN_INFO "alps-init: input_free_device\n");
out_device:
    return ret;
}

static int __devexit hscd_remove(struct i2c_client *client)
{
    struct hscd_data *data = i2c_get_clientdata(client);

    hscd_set_enable(data->client, 0);
    kfree(data);
    return 0;
}

static int hscd_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);
#ifdef ALPS_DEBUG
    printk("[HSCD] suspend\n");
#endif
    mutex_lock(&data->enable_mutex);
    if (atomic_read(&data->enable)) {
        cancel_work_sync(&data->irq_work);
        disable_irq(data->irq);
        hscd_set_enable(data->client, 0);
    }
    mutex_unlock(&data->enable_mutex);

    return 0;
}

static int hscd_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct hscd_data *data = i2c_get_clientdata(client);
#ifdef ALPS_DEBUG
    printk("[HSCD] resume\n");
#endif
    mutex_lock(&data->enable_mutex);
    if (atomic_read(&data->enable)) {
        hscd_set_enable(data->client, 1);
        enable_irq(data->irq);
    }
    mutex_unlock(&data->enable_mutex);

    return 0;
}

static const struct i2c_device_id hscd_id[] = {
    { HSCD_DRIVER_NAME, 0 },
    { }
};

static const struct dev_pm_ops hscd_pm_ops = {
    .suspend  = hscd_suspend,
    .resume   = hscd_resume,
};

static struct i2c_driver hscd_driver = {
    .probe    = hscd_probe,
    .remove   = hscd_remove,
    .driver   = {
        .owner = THIS_MODULE,
        .name = HSCD_DRIVER_NAME,
        .pm = &hscd_pm_ops,
    },
    .id_table = hscd_id,
};

static int __init hscd_init(void)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] init\n");
#endif
    return i2c_add_driver(&hscd_driver);
}

static void __exit hscd_exit(void)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] exit\n");
#endif
    i2c_del_driver(&hscd_driver);
}

module_init(hscd_init);
module_exit(hscd_exit);

MODULE_DESCRIPTION("Alps HSCDTD Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
