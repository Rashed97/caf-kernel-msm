/* bma222_i2c.c
 *
 * Accelerometer device driver for I2C
 *
 * Copyright (C) 2011-2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
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
#include <linux/err.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/regulator/consumer.h>

#define I2C_RETRY_DELAY    5
#define I2C_RETRIES        5

/* If i2c_board_info structure of acceleration sensor already registered, 
  please enable the "RESISTER_ACC_I2C".  */
//#define RESISTER_ACC_I2C

/* Register Name for accsns */
#define ACC_XOUT           0x02
#define ACC_YOUT           0x04
#define ACC_ZOUT           0x06
#define ACC_TEMP           0x08
#define ACC_REG0F          0x0F
#define ACC_REG10          0x10
#define ACC_REG11          0x11
#define ACC_REG14          0x14

#define ACC_DRIVER_NAME    "bma222"
#define I2C_ACC_ADDR       (0x18)        /* 001 1000    */
#define I2C_BUS_NUMBER     12

#ifdef ACC_DEBUG
#define ALPS_ACC_DEBUG
#endif

static struct i2c_driver accsns_driver;
static struct i2c_client *client_accsns = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend accsns_early_suspend_handler;
#endif

static atomic_t flgEna;
static atomic_t flgSuspend;

struct sensor_regulator {
    struct regulator *vreg;
    const char *name;
    u32 min_uV;
    u32 max_uV;
};

struct sensor_regulator accsns_vreg[] = {
    {NULL, "acc_vdd", 2850000, 2850000},
    {NULL, "i2c_src", 1800000, 1800000},
    {NULL, "vcc_i2c", 0, 0},
    {NULL, "acc_i2c", 0, 0},
};

static int accsns_config_regulator(struct i2c_client *client, bool on)
{
    int rc = 0, i;
    int num_reg = sizeof(accsns_vreg) / sizeof(struct sensor_regulator);

    if (on) {
        for (i = 0; i < num_reg; i++) {
            accsns_vreg[i].vreg = regulator_get(&client->dev,
                    accsns_vreg[i].name);
            if (IS_ERR(accsns_vreg[i].vreg)) {
                rc = PTR_ERR(accsns_vreg[i].vreg);
                pr_err("%s:regulator get failed rc=%d\n",
                        __func__, rc);
                goto error_vdd;
            }

            if (regulator_count_voltages(accsns_vreg[i].vreg) > 0) {
                rc = regulator_set_voltage(accsns_vreg[i].vreg,
                        accsns_vreg[i].min_uV, accsns_vreg[i].max_uV);
                if (rc) {
                    pr_err("%s:set_voltage failed rc=%d\n",
                        __func__, rc);
                    regulator_put(accsns_vreg[i].vreg);
                    goto error_vdd;
                }
            }

            rc = regulator_enable(accsns_vreg[i].vreg);
            if (rc) {
                pr_err("%s: regulator_enable failed rc =%d\n",
                        __func__,
                        rc);

                if (regulator_count_voltages(
                    accsns_vreg[i].vreg) > 0) {
                    regulator_set_voltage(accsns_vreg[i].vreg,
                            0, accsns_vreg[i].max_uV);
                }
                regulator_put(accsns_vreg[i].vreg);
                goto error_vdd;
            }
        }
        return rc;
    } else {
        i = num_reg;
    }
error_vdd:
    while (--i >= 0) {
        if (regulator_count_voltages(accsns_vreg[i].vreg) > 0) {
            regulator_set_voltage(accsns_vreg[i].vreg, 0,
                    accsns_vreg[i].max_uV);
        }
        regulator_disable(accsns_vreg[i].vreg);
        regulator_put(accsns_vreg[i].vreg);
    }
    return rc;
}

static int accsns_i2c_readm(u8 *rxData, int length)
{
    int err;
    int tries = 0;

    struct i2c_msg msgs[] = {
        {
            .addr  = client_accsns->addr,
            .flags = 0,
            .len   = 1,
            .buf   = rxData,
        },
        {
            .addr  = client_accsns->addr,
            .flags = I2C_M_RD,
            .len   = length,
            .buf   = rxData,
         },
    };

    do {
        err = i2c_transfer(client_accsns->adapter, msgs, 2);
    } while ((err != 2) && (++tries < I2C_RETRIES));

    if (err != 2) {
        dev_err(&client_accsns->adapter->dev, "read transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

static int accsns_i2c_writem(u8 *txData, int length)
{
    int err;
    int tries = 0;
#ifdef ALPS_ACC_DEBUG
    int i;
#endif

    struct i2c_msg msg[] = {
        {
            .addr  = client_accsns->addr,
            .flags = 0,
            .len   = length,
            .buf   = txData,
        },
    };

#ifdef ALPS_ACC_DEBUG
    printk("[ACC] i2c_writem : ");
    for (i=0; i<length;i++) printk("0X%02X, ", txData[i]);
    printk("\n");
#endif

    do {
        err = i2c_transfer(client_accsns->adapter, msg, 1);
    } while ((err != 1) && (++tries < I2C_RETRIES));

    if (err != 1) {
        dev_err(&client_accsns->adapter->dev, "write transfer error\n");
        err = -EIO;
    } else {
        err = 0;
    }

    return err;
}

int accsns_get_acceleration_data(int *xyz)
{
    int err = -1;
    int i;
    u8 sx[6];

    if (atomic_read(&flgSuspend) == 1) return err;
    sx[0] = ACC_XOUT;
    err = accsns_i2c_readm(sx, 6);
    if (err < 0) return err;
    for (i=0; i<3; i++) {
        xyz[i] = (int)((s8)sx[2 * i + 1]);
    }

#ifdef ALPS_ACC_DEBUG
    /*** DEBUG OUTPUT - REMOVE ***/
    printk("Acc_I2C, x:%d, y:%d, z:%d\n", xyz[0], xyz[1], xyz[2]);
    /*** <end> DEBUG OUTPUT - REMOVE ***/
#endif

    return err;
}

void accsns_activate(int flgatm, int flg)
{
    u8 buf[2];

    if (atomic_read(&flgSuspend))
        return;

    if (flg != 0) flg = 1;

    buf[0] = ACC_REG0F    ; buf[1] = 0x03;    /*  g-range +/-2g   */
    accsns_i2c_writem(buf, 2);
    buf[0] = ACC_REG10    ; buf[1] = 0x0D;    /*  Bandwidth 250Hz */
    accsns_i2c_writem(buf, 2);
    buf[0] = ACC_REG11;                       /*  Power modes     */
    if (flg == 0) buf[1] = 0x80;              /*    sleep         */
    else          buf[1] = 0x12;              /*    6ms           */
    accsns_i2c_writem(buf, 2);
    mdelay(2);
    if (flgatm) atomic_set(&flgEna, flg);
}

static void accsns_register_init(void)
{
    int d[3];
    u8  buf[2];

#ifdef ALPS_ACC_DEBUG
    printk("[ACC] register_init\n");
#endif

    buf[0] = ACC_REG14;
    buf[1] = 0xB6;
    accsns_i2c_writem(buf, 2);
    mdelay(4);

    accsns_activate(0, 1);
    accsns_get_acceleration_data(d);
    printk("[ACC] x:%d y:%d z:%d\n",d[0],d[1],d[2]);
    accsns_activate(0, 0);
}

static int accsns_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("[ACC] probe\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->adapter->dev, "client not i2c capable\n");
        return -ENOMEM;
    }

    client_accsns = kzalloc(sizeof(struct i2c_client), GFP_KERNEL);
    if (!client_accsns) {
        dev_err(&client->adapter->dev, "failed to allocate memory for module data\n");
        return -ENOMEM;
    }

    dev_info(&client->adapter->dev, "detected accelerometer\n");

    return 0;
}

static int __devexit accsns_remove(struct i2c_client *client)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] remove\n");
#endif
    accsns_activate(0, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&accsns_early_suspend_handler);
#endif
    kfree(client_accsns);
    return 0;
}

static int accsns_suspend(struct i2c_client *client,pm_message_t mesg)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] suspend\n");
#endif
    if (atomic_read(&flgSuspend))
        return 0;

    accsns_activate(0, 0);

    if (!atomic_read(&flgSuspend))
        accsns_config_regulator(client_accsns, false);

    atomic_set(&flgSuspend, 1);
    return 0;
}

static int accsns_resume(struct i2c_client *client)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] resume\n");
#endif
    if (!atomic_read(&flgSuspend))
        return 0;

    if (atomic_read(&flgSuspend))
        accsns_config_regulator(client_accsns, true);

    accsns_activate(0, atomic_read(&flgEna));

    atomic_set(&flgSuspend, 0);

    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void accsns_early_suspend(struct early_suspend *handler)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] early_suspend\n");
#endif
    //accsns_suspend(client_accsns, PMSG_SUSPEND);
}

static void accsns_early_resume(struct early_suspend *handler)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] early_resume\n");
#endif
    //accsns_resume(client_accsns);
}
#endif

static const struct i2c_device_id accsns_id[] = {
    { ACC_DRIVER_NAME, 0 },
    { }
};

static struct i2c_driver accsns_driver = {
    .probe     = accsns_probe,
    .remove    = accsns_remove,
    .id_table  = accsns_id,
    .driver    = {
        .name  = ACC_DRIVER_NAME,
    },
    .suspend   = accsns_suspend,
    .resume    = accsns_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend accsns_early_suspend_handler = {
    .suspend = accsns_early_suspend,
    .resume  = accsns_early_resume,
};
#endif

static int __init accsns_init(void)
{
#ifndef RESISTER_ACC_I2C
    struct i2c_board_info i2c_info;
    struct i2c_adapter *adapter;
#endif
    int rc;

#ifdef ALPS_ACC_DEBUG
    printk("[ACC] init\n");
#endif
    atomic_set(&flgEna, 0);
    atomic_set(&flgSuspend, 0);
    rc = i2c_add_driver(&accsns_driver);
    if (rc != 0) {
        printk("can't add i2c driver\n");
        rc = -ENOTSUPP;
        return rc;
    }

#ifndef RESISTER_ACC_I2C
    /* It is adding i2c_bord_info in adapter. If you already have added 
     i2c_board_info in adapter, you need to remove this code.           */
    memset(&i2c_info, 0, sizeof(struct i2c_board_info));
    i2c_info.addr = I2C_ACC_ADDR;
    strlcpy(i2c_info.type, ACC_DRIVER_NAME , I2C_NAME_SIZE);
    adapter = i2c_get_adapter(I2C_BUS_NUMBER);
    if (!adapter) {
        printk("can't get i2c adapter %d\n", I2C_BUS_NUMBER);
        rc = -ENOTSUPP;
        goto probe_done;
    }

    /* It is adding i2c_bord_info in adapter. If you already have added 
     i2c_board_info in adapter, you need to remove this code.           */
    client_accsns = i2c_new_device(adapter, &i2c_info);
    if (!client_accsns) {
        printk("can't add i2c device at 0x%x\n",(unsigned int)i2c_info.addr);
        rc = -ENOTSUPP;
        goto probe_done;
    }
    client_accsns->adapter->timeout = 0;
    client_accsns->adapter->retries = 0;

    /* It is adding i2c_bord_info in adapter. If you already have added 
     i2c_board_info in adapter, you need to remove this code.           */
    i2c_put_adapter(adapter);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&accsns_early_suspend_handler);
#endif

    accsns_config_regulator(client_accsns, true);
    accsns_register_init();

#ifdef ALPS_ACC_DEBUG
    printk("accsns_open end !!!!\n");
#endif
    
    probe_done: 

    return rc;
}

static void __exit accsns_exit(void)
{
#ifdef ALPS_ACC_DEBUG
    printk("[ACC] exit\n");
#endif
    i2c_del_driver(&accsns_driver);
}

module_init(accsns_init);
module_exit(accsns_exit);

EXPORT_SYMBOL(accsns_get_acceleration_data);
EXPORT_SYMBOL(accsns_activate);

MODULE_DESCRIPTION("Alps Accelerometer Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
