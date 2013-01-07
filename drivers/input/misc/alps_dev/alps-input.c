/* alps-input.c
 *
 * Input device driver for alps sensor
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
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/sched.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <asm/uaccess.h> 
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/ioctl.h>
#include "alps_compass_d_io.h"

extern int accsns_get_acceleration_data(int *xyz);
extern int hscd_get_magnetic_field_data(int *xyz);
extern void hscd_activate(int flgatm, int flg, int dtime);
extern void accsns_activate(int flgatm, int flg);
extern int hscd_self_test_A(void);
extern int hscd_self_test_B(void);
extern int hscd_get_hardware_data(int *xyz);
static void report_value(void);

static DEFINE_MUTEX(alps_lock);
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);

static struct platform_device *pdev;
static struct input_polled_dev *alps_idev;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend alps_early_suspend_handler;
#endif

#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Y
#define EVENT_TYPE_ACCEL_Z          ABS_Z
#define EVENT_TYPE_ACCEL_STATUS     ABS_WHEEL

#define EVENT_TYPE_MAGV_X           ABS_HAT0X
#define EVENT_TYPE_MAGV_Y           ABS_HAT0Y
#define EVENT_TYPE_MAGV_Z           ABS_BRAKE
#define EVENT_TYPE_MAGV_STATUS      ABS_THROTTLE

#define EVENT_TYPE_YAW              ABS_RX
#define EVENT_TYPE_PITCH            ABS_RY
#define EVENT_TYPE_ROLL             ABS_RZ
#define EVENT_TYPE_ORIENT_STATUS    ABS_RUDDER

#define ALPS_POLL_INTERVAL  100  /* msecs */
#define ALPS_INPUT_FUZZ       0  /* input event threshold */
#define ALPS_INPUT_FLAT       0

#define POLL_STOP_TIME      400  /* (msec) */
#define _HSCDTD_RANGE       4800

static int flgActivate = ACTIVE_SS_NUL;
static int flgSuspend = 0;
static int delay = 100;
static int poll_stop_cnt = 0;
static struct TAIFD_HW_DATA sns_hw_data;
static struct TAIFD_SW_DATA sns_sw_data;
static atomic_t data_ready;

/*-----------------------------------------------------------------------------------------------*/
/* I/O Control for SensoeService                                                                 */
/*-----------------------------------------------------------------------------------------------*/
static long alps_ss_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = -1;
    int tmpval;

    switch (cmd) {
        case ALPSIO_SS_SET_ACCACTIVATE:
            ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
            if (ret) {
                printk("error : alps_ss_ioctl(cmd = ALPSIO_SS_SET_ACCACTIVATE)\n");
                return -EFAULT;
            }
#ifdef ALPS_DEBUG
            printk("alps_ss_ioctl(cmd = ALPSIO_SS_SET_ACCACTIVATE), val = %d\n", tmpval);
#endif
            mutex_lock(&alps_lock);
            if (tmpval) flgActivate |=  ACTIVE_SS_ACC;
            else        flgActivate &= ~ACTIVE_SS_ACC;
            if ((tmpval == 0) && (flgActivate & ACTIVE_SS_ORI));
            else accsns_activate(1, tmpval);
            mutex_unlock(&alps_lock);
            break;

        case ALPSIO_SS_SET_MAGACTIVATE:
            ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
            if (ret) {
                printk("error : alps_ss_ioctl(cmd = ALPSIO_SS_SET_MAGACTIVATE)\n" );
                return -EFAULT;
            }
#ifdef ALPS_DEBUG
            printk("alps_ss_ioctl(cmd = ALPSIO_SS_SET_MAGACTIVATE), val = %d\n", tmpval);
#endif
            mutex_lock(&alps_lock);
            if (tmpval) flgActivate |=  ACTIVE_SS_MAG;
            else        flgActivate &= ~ACTIVE_SS_MAG;
            if ((tmpval == 0) && (flgActivate & ACTIVE_SS_ORI));
            else hscd_activate(1, tmpval, delay);
            mutex_unlock(&alps_lock);
            break;

        case ALPSIO_SS_SET_ORIACTIVATE:
            ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
            if (ret) {
                printk("error : alps_ss_ioctl(cmd = ALPSIO_SS_SET_ORIACTIVATE)\n" );
                return -EFAULT;
            }
#ifdef ALPS_DEBUG
            printk("alps_ss_ioctl(cmd = ALPSIO_SS_SET_ORIACTIVATE), val = %d\n", tmpval);
#endif
            mutex_lock(&alps_lock);
            if (tmpval) flgActivate |=  ACTIVE_SS_ORI;
            else        flgActivate &= ~ACTIVE_SS_ORI;
            if ((tmpval == 0) && (flgActivate & ACTIVE_SS_ACC));
            else accsns_activate(1, tmpval);
            if ((tmpval == 0) && (flgActivate & ACTIVE_SS_MAG));
            else hscd_activate(1, tmpval, delay);
            mutex_unlock(&alps_lock);
            break;

        case ALPSIO_SS_SET_DELAY:
            ret = copy_from_user(&tmpval, argp, sizeof(tmpval));
            if (ret) {
                printk( "error : alps_ss_ioctl(cmd = ALPSIO_SS_SET_DELAY)\n" );
                return -EFAULT;
            }
#ifdef ALPS_DEBUG
            printk("alps_ss_ioctl(cmd = ALPSIO_SS_SET_DELAY)\n");
#endif
            mutex_lock(&alps_lock);
            if (flgActivate == ACTIVE_SS_ACC) {
                if      (tmpval <=  10) tmpval =  10;
            }
            else {
                if      (tmpval <=  10) tmpval =  10;
                else if (tmpval <=  20) tmpval =  20;
                else if (tmpval <=  70) tmpval =  50;
                else                    tmpval = 100;
            }
            delay = tmpval;
            poll_stop_cnt = POLL_STOP_TIME / tmpval;
            if (flgActivate & (ACTIVE_SS_MAG | ACTIVE_SS_ORI)) hscd_activate(1, 1, delay);
            else                                               hscd_activate(1, 0, delay);
            mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
            printk("     delay = %d\n", delay);
#endif
            break;

        case ALPSIO_SS_GET_HWDATA:
            {
                int xyz[3];
#ifdef ALPS_DEBUG
               printk("alps_ss_ioctl(cmd = ALPSIO_SS_GET_HWDATA)\n");
#endif
                mutex_lock(&alps_lock);
                ret = hscd_get_hardware_data(xyz);
                mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
                printk("[HSCD] get hw data, %d, %d, %d\n", xyz[0], xyz[1], xyz[2]);
#endif
                if (copy_to_user(argp, xyz, sizeof xyz)) {
                    printk( "error : alps_ss_ioctl(cmd = ALPSIO_SS_GET_HWDATA)\n" );
                    return -EFAULT;
                }
            }
            break;

        default:
            return -ENOTTY;
    }
    return 0;
}

static int 
alps_ss_io_open( struct inode* inode, struct file* filp )
{
    return 0;
}

static int 
alps_ss_io_release( struct inode* inode, struct file* filp )
{
    return 0;
}

static struct file_operations alps_ss_fops = {
    .owner   = THIS_MODULE,
    .open    = alps_ss_io_open,
    .release = alps_ss_io_release,
    .unlocked_ioctl = alps_ss_ioctl,
};

static struct miscdevice alps_ss_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "alps_ss_io",
    .fops  = &alps_ss_fops,
};


/*-----------------------------------------------------------------------------------------------*/
/* I/O Control for AcdapiDaemon                                                                  */
/*-----------------------------------------------------------------------------------------------*/
static long alps_ad_ioctl(struct file* filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = -1;
    int tmpval;
    struct TAIFD_HW_DATA tmpval_hw;
    struct TAIFD_SW_DATA tmpval_sw;

    switch (cmd) {
        case ALPSIO_AD_GET_ACTIVATE:
#ifdef ALPS_DEBUG
            printk("alps_ad_ioctl(cmd = ALPSIO_AD_GET_ACTIVATE)\n");
#endif
            mutex_lock(&alps_lock);
            tmpval = flgActivate;
            mutex_unlock(&alps_lock);
            if (copy_to_user(argp, &tmpval, sizeof(tmpval))) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_GET_ACTIVATE)\n" );
                return -EFAULT;
            }
            break;

        case ALPSIO_AD_GET_DELAY:
#ifdef ALPS_DEBUG
            printk("alps_ad_ioctl(cmd = ALPSIO_AD_GET_DELAY)\n");
#endif
            mutex_lock(&alps_lock);
            tmpval = delay;
            mutex_unlock(&alps_lock);
            if (copy_to_user(argp, &tmpval, sizeof(tmpval))) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_GET_DELAY)\n" );
                return -EFAULT;
            }
            break;

        case ALPSIO_AD_GET_DATA:
#ifdef ALPS_DEBUG
            printk("alps_ad_ioctl(cmd = ALPSIO_AD_GET_DATA)\n");
#endif
            ret = wait_event_interruptible_timeout(data_ready_wq, atomic_read(&data_ready), 1000);
            if (ret < 0) {
                printk( "error : wait_event_interruptible_timeout, %d\n", ret);
                return -EFAULT;
            }
            mutex_lock(&alps_lock);
            if (ret == 0) memset(&tmpval_hw, 0, sizeof tmpval_hw);
            else          memcpy(&tmpval_hw, &sns_hw_data, sizeof tmpval_hw);
            tmpval_hw.activate = flgActivate;
            tmpval_hw.delay    = delay;
            atomic_set(&data_ready, 0);
            mutex_unlock(&alps_lock);
            if (copy_to_user(argp, &tmpval_hw, sizeof tmpval_hw)) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_GET_DATA)\n" );
                return -EFAULT;
            }
            break;

        case ALPSIO_AD_SET_DATA:
            ret = copy_from_user(&tmpval_sw, argp, sizeof(tmpval_sw));
            if (ret) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_SET_DATA)\n" );
                return -EFAULT;
            }
#ifdef ALPS_DEBUG
            printk("alps_ad_ioctl(cmd = ALPSIO_AD_SET_DATA)\n");
#endif
            mutex_lock(&alps_lock);
            memcpy(&sns_sw_data, &tmpval_sw, sizeof sns_sw_data);
            report_value();
            mutex_unlock(&alps_lock);
            break;

        case ALPSIO_AD_EXE_SELF_TEST_A:
#ifdef ALPS_DEBUG
               printk("alps_ad_ioctl(cmd = ALPSIO_AD_EXE_SELF_TEST_A)\n");
#endif
            mutex_lock(&alps_lock);
            ret = hscd_self_test_A();
            mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
            printk("[HSCD] Self test-A result : %d\n", ret);
#endif
            if (copy_to_user(argp, &ret, sizeof(ret))) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_EXE_SELF_TEST_A)\n" );
                return -EFAULT;
            }
            break;

        case ALPSIO_AD_EXE_SELF_TEST_B:
#ifdef ALPS_DEBUG
               printk("alps_ad_ioctl(cmd = ALPSIO_AD_EXE_SELF_TEST_B)\n");
#endif
            mutex_lock(&alps_lock);
            ret = hscd_self_test_B();
            mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
            printk("[HSCD] Self test-B result : %d\n", ret);
#endif
            if (copy_to_user(argp, &ret, sizeof(ret))) {
                printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_EXE_SELF_TEST_B)\n" );
                return -EFAULT;
            }
            break;

        case ALPSIO_AD_GET_HWDATA:
            {
                int xyz[3];
#ifdef ALPS_DEBUG
               printk("alps_ad_ioctl(cmd = ALPSIO_AD_GET_HWDATA)\n");
#endif
                mutex_lock(&alps_lock);
                ret = hscd_get_hardware_data(xyz);
                mutex_unlock(&alps_lock);
#ifdef ALPS_DEBUG
                printk("[HSCD] get hw data, %d, %d, %d\n", xyz[0], xyz[1], xyz[2]);
#endif
                if (copy_to_user(argp, xyz, sizeof xyz)) {
                    printk( "error : alps_ad_ioctl(cmd = ALPSIO_AD_GET_HWDATA)\n" );
                    return -EFAULT;
                }
            }
            break;

        default:
            return -ENOTTY;
    }
    return 0;
}

static int 
alps_ad_io_open( struct inode* inode, struct file* filp )
{
    return 0;
}

static int 
alps_ad_io_release( struct inode* inode, struct file* filp )
{
    return 0;
}

static struct file_operations alps_ad_fops = {
    .owner   = THIS_MODULE,
    .open    = alps_ad_io_open,
    .release = alps_ad_io_release,
    .unlocked_ioctl = alps_ad_ioctl,
};

static struct miscdevice alps_ad_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "alps_ad_io",
    .fops  = &alps_ad_fops,
};

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


static DEVICE_ATTR(self_test_A, 0444, hscd_self_test_A_show, NULL);
static DEVICE_ATTR(self_test_B, 0444, hscd_self_test_B_show, NULL);
static DEVICE_ATTR(get_hw_data, 0444, hscd_get_hw_data_show, NULL);

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
/* input device                                                                                  */
/*-----------------------------------------------------------------------------------------------*/
static int alps_probe(struct platform_device *dev)
{
    printk(KERN_INFO "alps: alps_probe\n");
    return 0;
}

static int alps_remove(struct platform_device *dev)
{
    printk(KERN_INFO "alps: alps_remove\n");
    return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void alps_early_suspend(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("alps-input: early_suspend\n");
#endif
    mutex_lock(&alps_lock);
    flgSuspend = 1;
    mutex_unlock(&alps_lock);
}

static void alps_early_resume(struct early_suspend *handler)
{
#ifdef ALPS_DEBUG
    printk("alps-input: early_resume\n");
#endif
    mutex_lock(&alps_lock);
    poll_stop_cnt = POLL_STOP_TIME / delay;
    flgSuspend = 0;
    mutex_unlock(&alps_lock);
}
#endif

static struct platform_driver alps_driver = {
    .driver    = {
        .name  = "alps-input",
        .owner = THIS_MODULE,
    },
    .probe     = alps_probe,
    .remove    = alps_remove,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend alps_early_suspend_handler = {
    .suspend = alps_early_suspend,
    .resume  = alps_early_resume,
};
#endif

static void accsns_poll(struct input_dev *idev)
{
    int xyz[3];

    if (accsns_get_acceleration_data(xyz) == 0) {
        sns_hw_data.acc[0] = 1;
        memcpy(&sns_hw_data.acc[1], xyz, sizeof xyz);
    }
}

static void hscd_poll(struct input_dev *idev)
{
    int xyz[3];

    if (hscd_get_magnetic_field_data(xyz) == 0) {
        sns_hw_data.mag[0] = 1;
        memcpy(&sns_hw_data.mag[1], xyz, sizeof xyz);
    }
}

static void alps_poll(struct input_polled_dev *dev)
{
    struct input_dev *idev = dev->input;

    mutex_lock(&alps_lock);
    dev->poll_interval = delay;
    if (!flgSuspend) {
        if (poll_stop_cnt-- < 0) {
            poll_stop_cnt = -1;
            memset(&sns_hw_data, 0, sizeof sns_hw_data);
            if (flgActivate & (ACTIVE_SS_MAG | ACTIVE_SS_ORI)) hscd_poll(idev);
            if (flgActivate & (ACTIVE_SS_ACC | ACTIVE_SS_ORI)) accsns_poll(idev);
            if (flgActivate & (ACTIVE_SS_ACC | ACTIVE_SS_MAG | ACTIVE_SS_ORI)) {
                if ((sns_hw_data.acc[0] == 1) || (sns_hw_data.mag[0] == 1)) {
                    atomic_set(&data_ready, 1);
                    wake_up(&data_ready_wq);
                }
            }
        }
#ifdef ALPS_DEBUG
        else printk("polling stop. delay = %d, poll_stop_cnt = %d\n", delay, poll_stop_cnt);
#endif
    }
    mutex_unlock(&alps_lock);
}

static void report_value(void)
{
    if (!flgSuspend) {
        if (poll_stop_cnt < 0) {
            if ((flgActivate & ACTIVE_SS_ACC) && sns_sw_data.acc[0]) {
                input_report_abs(alps_idev->input, EVENT_TYPE_ACCEL_X      , sns_sw_data.acc[1]);
                input_report_abs(alps_idev->input, EVENT_TYPE_ACCEL_Y      , sns_sw_data.acc[2]);
                input_report_abs(alps_idev->input, EVENT_TYPE_ACCEL_Z      , sns_sw_data.acc[3]);
                input_report_abs(alps_idev->input, EVENT_TYPE_ACCEL_STATUS , sns_sw_data.acc[4]);
                alps_idev->input->sync = 0;
                input_event(alps_idev->input, EV_SYN, SYN_REPORT, 1);
            }
            if ((flgActivate & ACTIVE_SS_MAG) && sns_sw_data.mag[0]) {
                input_report_abs(alps_idev->input, EVENT_TYPE_MAGV_X       , sns_sw_data.mag[1]);
                input_report_abs(alps_idev->input, EVENT_TYPE_MAGV_Y       , sns_sw_data.mag[2]);
                input_report_abs(alps_idev->input, EVENT_TYPE_MAGV_Z       , sns_sw_data.mag[3]);
                input_report_abs(alps_idev->input, EVENT_TYPE_MAGV_STATUS  , sns_sw_data.mag[4]);
                alps_idev->input->sync = 0;
                input_event(alps_idev->input, EV_SYN, SYN_REPORT, 2);
            }
            if ((flgActivate & ACTIVE_SS_ORI) && sns_sw_data.ori[0]) {
                input_report_abs(alps_idev->input, EVENT_TYPE_YAW          , sns_sw_data.ori[1]);
                input_report_abs(alps_idev->input, EVENT_TYPE_PITCH        , sns_sw_data.ori[2]);
                input_report_abs(alps_idev->input, EVENT_TYPE_ROLL         , sns_sw_data.ori[3]);
                input_report_abs(alps_idev->input, EVENT_TYPE_ORIENT_STATUS, sns_sw_data.ori[4]);
                alps_idev->input->sync = 0;
                input_event(alps_idev->input, EV_SYN, SYN_REPORT, 3);
            }
        }
    }
    memset(&sns_sw_data, 0, sizeof(sns_sw_data));
}

static int __init alps_init(void)
{
    struct input_dev *idev;
    int ret;

    ret = platform_driver_register(&alps_driver);
    if (ret)
        goto out_region;
    printk(KERN_INFO "alps-init: platform_driver_register\n");

    pdev = platform_device_register_simple("alps_compass", -1, NULL, 0);
    if (IS_ERR(pdev)) {
        ret = PTR_ERR(pdev);
        goto out_driver;
    }
    printk(KERN_INFO "alps-init: platform_device_register_simple\n");

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
    idev->phys = "alps_compass/input0";
    idev->id.bustype = BUS_HOST;
    idev->dev.parent = &pdev->dev;
    idev->evbit[0] = BIT_MASK(EV_ABS);

    input_set_abs_params(idev, EVENT_TYPE_ACCEL_X,
            -2048, 2048, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_ACCEL_Y,
            -2048, 2048, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_ACCEL_Z,
            -2048, 2048, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_ACCEL_STATUS,
                0,    3, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);

    input_set_abs_params(idev, EVENT_TYPE_MAGV_X,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_Y,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_Z,
            -_HSCDTD_RANGE, _HSCDTD_RANGE, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_MAGV_STATUS,
                0,    3, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);


    input_set_abs_params(idev, EVENT_TYPE_YAW,
                0,  360, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_PITCH,
             -180,  180, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_ROLL,
             -180,  180, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);
    input_set_abs_params(idev, EVENT_TYPE_ORIENT_STATUS,
                0,    3, ALPS_INPUT_FUZZ, ALPS_INPUT_FLAT);

    ret = input_register_polled_device(alps_idev);
    if (ret)
        goto out_alc_poll;
    printk(KERN_INFO "alps-init: input_register_polled_device\n");

    ret = misc_register(&alps_ss_device);
    if (ret) {
        printk("alps-init: alps_ss_device register failed\n");
        goto out_reg_poll;
    }
    printk("alps-init: alps_ss_device misc_register\n");

    ret = misc_register(&alps_ad_device);
    if (ret) {
        printk("alps-init: alps_ad_device register failed\n");
        goto out_misc_ss;
    }
    printk("alps-init: alps_as_device misc_register\n");

    ret = sysfs_create_group(&alps_idev->input->dev.kobj, &hscd_attribute_group);
    if (ret)
        goto out_misc_ad;
    printk(KERN_INFO "alps-init: sysfs_create_group\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
    register_early_suspend(&alps_early_suspend_handler);
    printk("alps-init: early_suspend_register\n");
#endif

    mutex_lock(&alps_lock);
    flgSuspend = 0;
    memset(&sns_hw_data, 0, sizeof sns_hw_data);
    memset(&sns_sw_data, 0, sizeof sns_sw_data);
    mutex_unlock(&alps_lock);
    init_waitqueue_head(&data_ready_wq);

    return 0;

out_misc_ad:
    misc_deregister(&alps_ad_device);
    printk(KERN_INFO "alps-init: input_unregister_polled_device(alps_ad_device)\n");
out_misc_ss:
    misc_deregister(&alps_ss_device);
    printk(KERN_INFO "alps-init: input_unregister_polled_device(alps_ss_device)\n");
out_reg_poll:
    input_unregister_polled_device(alps_idev);
    printk(KERN_INFO "alps-init: input_unregister_polled_device\n");
out_alc_poll:
    input_free_polled_device(alps_idev);
    printk(KERN_INFO "alps-init: input_free_polled_device\n");
out_device:
    platform_device_unregister(pdev);
    printk(KERN_INFO "alps-init: platform_device_unregister\n");
out_driver:
    platform_driver_unregister(&alps_driver);
    printk(KERN_INFO "alps-init: platform_driver_unregister\n");
out_region:
    return ret;
}

static void __exit alps_exit(void)
{
    sysfs_remove_group(&alps_idev->input->dev.kobj, &hscd_attribute_group);
    printk(KERN_INFO "alps-exit: sysfs_remove_group\n");
    misc_deregister(&alps_ad_device);
    printk(KERN_INFO "alps-exit: alps_ad_device misc_deregister\n");
    misc_deregister(&alps_ss_device);
    printk(KERN_INFO "alps-exit: alps_ss_device misc_deregister\n");
    input_unregister_polled_device(alps_idev);
    printk(KERN_INFO "alps-exit: input_unregister_polled_device\n");
    input_free_polled_device(alps_idev);
    printk(KERN_INFO "alps-exit: input_free_polled_device\n");
    platform_device_unregister(pdev);
    printk(KERN_INFO "alps-exit: platform_device_unregister\n");
    platform_driver_unregister(&alps_driver);
    printk(KERN_INFO "alps-exit: platform_driver_unregister\n");
}

module_init(alps_init);
module_exit(alps_exit);

MODULE_DESCRIPTION("Alps Input Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
