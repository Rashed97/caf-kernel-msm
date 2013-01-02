/*
 * Copyright (C) Gigabyte 2013 1 1
 *
 * License Terms: GNU General Public License, version 2
 * Author: Karl Chen <karl.chen@gigabyte.com.tw> for Gigabyte
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/input/mic_biase_switch.h>

struct mic_biase_switch_platform_data *mic_data;
void mic_biase_switch_set_enable(int enable)
{
    static int switch_enable = MIC_BIASE_SWITCH_L;
    if (enable == MIC_BIASE_SWITCH_INV)
        switch_enable = ~switch_enable;
    else
        switch_enable = enable;
    gpio_set_value(mic_data->gpio_switch, switch_enable);
}

static int __devinit mic_biase_switch_probe(struct platform_device *pdev)
{
    static struct regulator *regulator_l17;
    mic_data = kzalloc(sizeof(struct mic_biase_switch_platform_data), GFP_KERNEL);

    regulator_l17= regulator_get(NULL, "8038_l17");
    msleep(10);
    regulator_set_voltage(regulator_l17, 2950000, 2950000);
    msleep(10);
    regulator_enable(regulator_l17);
    msleep(10);

    mic_data = pdev->dev.platform_data;
    mic_biase_switch_set_enable(MIC_BIASE_SWITCH_L);

    return 0;
}

static int __devexit mic_biase_switch_remove(struct platform_device *pdev)
{
    mic_data = pdev->dev.platform_data;
    mic_biase_switch_set_enable(MIC_BIASE_SWITCH_L);

    return 0;
}

static struct platform_driver mic_biase_switch_driver = {
    .driver.name    = "mic_biase_switch",
    .driver.owner   = THIS_MODULE,
    .probe          = mic_biase_switch_probe,
    .remove         = __devexit_p(mic_biase_switch_remove),
};

static int __init mic_biase_switch_init(void)
{
    return platform_driver_register(&mic_biase_switch_driver);
}
subsys_initcall(mic_biase_switch_init);

static void __exit mic_biase_switch_exit(void)
{
    platform_driver_unregister(&mic_biase_switch_driver);
}
module_exit(mic_biase_switch_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MIC_BIASE_SWITCH GPIO driver");
MODULE_AUTHOR("Karl Chen <karl.chen@gigabyte.com.tw>");
