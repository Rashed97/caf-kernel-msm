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
#include <linux/delay.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/* If i2c_board_info structure of HSCDTD sensor already registered, 
  please enable the "RESISTER_HSCDTD_I2C".  */
//#define RESISTER_HSCDTD_I2C   1

#define I2C_RETRY_DELAY  5
#define I2C_RETRIES      5

#define I2C_HSCD_ADDR    (0x0c)    /* 000 1100    */
#define I2C_BUS_NUMBER   4

#define HSCD_DRIVER_NAME "hscd_i2c"

#define HSCD_CHIP_ID     0x1511

#define HSCD_STB         0x0C
#define HSCD_INFO        0x0D
#define HSCD_XOUT        0x10
#define HSCD_YOUT        0x12
#define HSCD_ZOUT        0x14
#define HSCD_XOUT_H      0x11
#define HSCD_XOUT_L      0x10
#define HSCD_YOUT_H      0x13
#define HSCD_YOUT_L      0x12
#define HSCD_ZOUT_H      0x15
#define HSCD_ZOUT_L      0x14

#define HSCD_STATUS      0x18
#define HSCD_CTRL1       0x1b
#define HSCD_CTRL2       0x1c
#define HSCD_CTRL3       0x1d
#define HSCD_CTRL4       0x1e

#define STBB_OUTV_THR    3838


static struct i2c_driver hscd_driver;
static struct i2c_client *client_hscd = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend hscd_early_suspend_handler;
#endif

static atomic_t flgEna;
static atomic_t delay;
static atomic_t flgSuspend;

int hscd_get_hardware_data(int *xyz);

/*-----------------------------------------------------------------------------------------------*/
/* i2c read/erite function                                                                       */
/*-----------------------------------------------------------------------------------------------*/
static int hscd_i2c_readm(char *rxData, int length)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr  = client_hscd->addr,
            .flags = 0,
            .len   = 1,
            .buf   = rxData,
        },
        {
            .addr  = client_hscd->addr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxData,
         },
    };

    do {
        err = i2c_transfer(client_hscd->adapter, msgs, 2);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        dev_err(&client_hscd->adapter->dev, "read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int hscd_i2c_writem(char *txData, int length)
{
    int err;
    int tries = 0;
#ifdef ALPS_DEBUG
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr  = client_hscd->addr,
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
        err = i2c_transfer(client_hscd->adapter, msg, 1);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        dev_err(&client_hscd->adapter->dev, "write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

/*-----------------------------------------------------------------------------------------------*/
/* hscdtd function                                                                               */
/*-----------------------------------------------------------------------------------------------*/
int hscd_self_test_A(void)
{
    u8 sx[2], cr1[1];

    if (atomic_read(&flgSuspend) == 1) return -1;
    /* Control resister1 backup  */
    cr1[0] = HSCD_CTRL1;
    if (hscd_i2c_readm(cr1, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] Control resister1 value, %02X\n", cr1[0]);
#endif
    mdelay(1);

    /* Move active mode (force state)  */
    sx[0] = HSCD_CTRL1;
    sx[1] = 0x8A;
    if (hscd_i2c_writem(sx, 2)) return 1;

    /* Get inital value of self-test-A register  */
    sx[0] = HSCD_STB;
    hscd_i2c_readm(sx, 1);
    mdelay(1);
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test A register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0x55) {
        printk("error: self-test-A, initial value is %02X\n", sx[0]);
        return 2;
    }

    /* do self-test*/
    sx[0] = HSCD_CTRL3;
    sx[1] = 0x10;
    if (hscd_i2c_writem(sx, 2)) return 1;
    mdelay(3);

    /* Get 1st value of self-test-A register  */
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0xAA) {
        printk("error: self-test, 1st value is %02X\n", sx[0]);
        return 3;
    }
    mdelay(3);

    /* Get 2nd value of self-test register  */
    sx[0] = HSCD_STB;
    if (hscd_i2c_readm(sx, 1)) return 1;
#ifdef ALPS_DEBUG
    else printk("[HSCD] self test  register value, %02X\n", sx[0]);
#endif
    if (sx[0] != 0x55) {
        printk("error: self-test, 2nd value is %02X\n", sx[0]);
        return 4;
    }

    /* Resume */
    sx[0] = HSCD_CTRL1;
    sx[1] = cr1[0];
    if (hscd_i2c_writem(sx, 2)) return 1;

    return 0;
}

int hscd_self_test_B(void)
{
    int rc = 0, xyz[3];
    u8 sx = 0;

    if (atomic_read(&flgSuspend) == 1) return -1;

    /* Measurement sensor value */
    if (hscd_get_hardware_data(xyz)) return 1;

    /* Check output value */
    if ((xyz[0] <= -STBB_OUTV_THR) || (xyz[0] >= STBB_OUTV_THR)) sx |= 0x01;
    if ((xyz[1] <= -STBB_OUTV_THR) || (xyz[1] >= STBB_OUTV_THR)) sx |= 0x02;
    if ((xyz[2] <= -STBB_OUTV_THR) || (xyz[2] >= STBB_OUTV_THR)) sx |= 0x04;
    if (sx) {
        printk("error: self-test-B, 1st value is %02X\n", sx);
        rc = (int)(sx | 0x10);
    }

    return rc;
}

int hscd_get_magnetic_field_data(int *xyz)
{
    int err = -1;
    int i;
    u8 sx[6];

    if (atomic_read(&flgSuspend) == 1) return err;
    sx[0] = HSCD_XOUT;
    err = hscd_i2c_readm(sx, 6);
    if (err < 0) return err;
    for (i=0; i<3; i++) {
        xyz[i] = (int) ((short)((sx[2*i+1] << 8) | (sx[2*i])));
    }

#ifdef ALPS_DEBUG
    printk("Mag_I2C, x:%d, y:%d, z:%d\n",xyz[0], xyz[1], xyz[2]);
#endif

    return err;
}

void hscd_activate(int flgatm, int active, int dtime)
{
    u8 buf[2];

    if (active != 0) active = 1;

    if (active) {
        buf[0] = HSCD_CTRL4;                       // 15 bit signed value
        buf[1] = 0x90;
        hscd_i2c_writem(buf, 2);
    }
    mdelay(1);
    
    if      (dtime <=  20) buf[1] = (3<<3);        // 100Hz- 10msec
    else if (dtime <=  70) buf[1] = (2<<3);        //  20Hz- 50msec
    else                   buf[1] = (1<<3);        //  10Hz-100msec
    buf[0]  = HSCD_CTRL1;
    buf[1] |= (active<<7);
    hscd_i2c_writem(buf, 2);
    mdelay(3);

    if (flgatm) {
        atomic_set(&flgEna, active);
        atomic_set(&delay, dtime);
    }
}

static int hscd_register_init(void)
{
    int v[3], ret = 0;
    u8  buf[2];
    u16 chip_info;

#ifdef ALPS_DEBUG
    printk("[HSCD] register_init\n");
#endif

    buf[0] = HSCD_INFO;
    ret = hscd_i2c_readm(buf, 2);
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
    hscd_i2c_writem(buf, 2);
    mdelay(5);

    atomic_set(&delay, 100);
    hscd_activate(0, 1, atomic_read(&delay));
    hscd_get_magnetic_field_data(v);
    printk("[HSCD] x:%d y:%d z:%d\n", v[0], v[1], v[2]);
    hscd_activate(0, 0, atomic_read(&delay));

    return 0;
}

int hscd_get_hardware_data(int *xyz)
{
    int ret = 0;
#ifdef ALPS_DEBUG
    printk("[HSCD] hscd_get_hardware_data\n");
#endif
    if (atomic_read(&flgSuspend) == 1) return -1;
    hscd_activate(0, 1, 10);
    mdelay(15);
    ret = hscd_get_magnetic_field_data(xyz);
    hscd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
    return ret;
}

/*-----------------------------------------------------------------------------------------------*/
/* sysfs                                                                                         */
/*-----------------------------------------------------------------------------------------------*/
static ssize_t hscd_self_test_A_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret = -1;
#ifdef ALPS_DEBUG
    printk("hscd_self_test_A_show\n");
#endif
    mutex_lock(&alps_lock);
    ret = hscd_self_test_A();
    mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
    printk("[HSCD] Self test-A result : %d\n", ret);
#endif

    return sprintf(buf, "%d\n", ret);
}

static ssize_t hscd_self_test_B_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret = -1;
#ifdef ALPS_DEBUG
    printk("hscd_self_test_B_show\n");
#endif
    mutex_lock(&alps_lock);
    ret = hscd_self_test_B();
    mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
    printk("[HSCD] Self test-B result : %d\n", ret);
#endif

    return sprintf(buf, "%d\n", ret);
}

static ssize_t hscd_get_hw_data_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int xyz[3], ret = -1;
#ifdef ALPS_DEBUG
    printk("hscd_get_hw_data\n");
#endif
    mutex_lock(&alps_lock);
    ret = hscd_get_hardware_data(xyz);
    mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
    printk("[HSCD] get hw data, %d, %d, %d\n", xyz[0], xyz[1], xyz[2]);
#endif

    return sprintf(buf, "%d,%d,%d\n", xyz[0], xyz[1], xyz[2]);
}

static ssize_t hscd_get_delay(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int interval_ms = atomic_read(&delay);
    return sprintf(buf, "%d\n", interval_ms);
}

static ssize_t hscd_set_delay(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t size)
{
    unsigned long interval_ms;

    if (kstrtoul(buf, 10, &interval_ms) | !interval_ms)
        return -EINVAL;

    atomic_set(&delay, interval_ms);

    return size;
}

static ssize_t hscd_get_enable(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int val = atomic_read(&flgEna);
    return sprintf(buf, "%d\n", val);
}

static ssize_t hscd_set_enable(struct device *dev,
                    struct device_attribute *attr,
                    const char *buf, size_t size)
{
    unsigned long val;

    if (kstrtoul(buf, 10, &val))
        return -EINVAL;

    if (val)
        hscd_activate(1, 1, atomic_read(&delay));
    else
        hscd_activate(1, 0, atomic_read(&delay));

    return size;
}

static DEVICE_ATTR(self_test_A, 0444, hscd_self_test_A_show, NULL);
static DEVICE_ATTR(self_test_B, 0444, hscd_self_test_B_show, NULL);
static DEVICE_ATTR(get_hw_data, 0444, hscd_get_hw_data_show, NULL);
static DEVICE_ATTR(delay, 0444, hscd_get_delay, hscd_set_delay);
static DEVICE_ATTR(enable, 0444, hscd_get_enable, hscd_set_enable);

static struct attribute *hscd_attributes[] = {
    &dev_attr_self_test_A.attr,
    &dev_attr_self_test_B.attr,
    &dev_attr_get_hw_data.attr,
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
    printk("[HSCD] probe\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->adapter->dev, "client not i2c capable\n");
        return -ENOMEM;
    }

    client_hscd = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if (!client_hscd) {
        dev_err(&client->adapter->dev, "failed to allocate memory for module data\n");
        return -ENOMEM;
    }

    rc = hscd_register_init();
    if (rc != 0) {
        return -ENOTSUPP;
    }

#ifdef ALPS_DEBUG
    printk("%s Init end!!!!\n", __func__);
#endif


    alps_idev = input_allocate_polled_device();
    if (!alps_idev) {
        ret = -ENOMEM;
        goto out_device;
    }
    printk(KERN_INFO "alps-init: input_allocate_polled_device\n");

    alps_idev->poll = alps_poll;
    alps_idev->poll_interval = ALPS_POLL_INTERVAL;

    /* initialize the input class */
    idev = alps_idev->input;
    idev->name = "alps_compass";
    idev->id.bustype = BUS_HOST;
    idev->dev.parent = &pdev->dev;
    idev->evbit[0] = BIT_MASK(EV_ABS);

    input_set_abs_params(idev, EVENT_TYPE_MAGV_X,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_Y,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_Z,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_STATUS,
                0,    3, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);

    ret = input_register_polled_device(alps_idev);
    if (ret)
        goto out_alc_poll;
    printk(KERN_INFO "alps-init: input_register_polled_device\n");

    ret = sysfs_create_group(&alps_idev->input->dev.kobj, &hscd_attribute_group);
    if (ret)
        goto out_misc_ad;
    printk(KERN_INFO "alps-init: sysfs_create_group\n");

    atomic_set(&flgEna, 0);
    atomic_set(&delay, 200);
    atomic_set(&flgSuspend, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&hscd_early_suspend_handler);
#endif

    dev_info(&client->adapter->dev, "detected HSCDTD007/008 geomagnetic field sensor\n");

    return 0;
}

static int __devexit hscd_remove(struct i2c_client *client)
{
    printk("[HSCD] remove\n");
    hscd_activate(0, 0, atomic_read(&delay));
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&hscd_early_suspend_handler);
#endif
    kfree(client_hscd);
    return 0;
}

static int hscd_suspend(struct i2c_client *client, pm_message_t mesg)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] suspend\n");
#endif
    atomic_set(&flgSuspend, 1);
    hscd_activate(0, 0, atomic_read(&delay));
    return 0;
}

static int hscd_resume(struct i2c_client *client)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] resume\n");
#endif
    atomic_set(&flgSuspend, 0);
    hscd_activate(0, atomic_read(&flgEna), atomic_read(&delay));
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hscd_early_suspend(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] early_suspend\n");
#endif
    hscd_suspend(client_hscd, PMSG_SUSPEND);
}

static void hscd_early_resume(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("[HSCD] early_resume\n");
#endif
    hscd_resume(client_hscd);
}
#endif

static const struct i2c_device_id ALPS_id[] = {
    { HSCD_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver hscd_driver = {
    .probe    = hscd_probe,
    .remove   = hscd_remove,
    .id_table = ALPS_id,
    .driver   = {
        .name = HSCD_DRIVER_NAME,
    },
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend  = hscd_suspend,
    .resume   = hscd_resume,
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend hscd_early_suspend_handler = {
    .suspend = hscd_early_suspend,
    .resume  = hscd_early_resume,
};
#endif

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
