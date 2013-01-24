/*
 * Copyright (c) 2012-2013, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/iqs128.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct iqs128_data {
	struct iqs128_platform_data *pdata;
	struct input_dev *input;
	struct work_struct work;
};

enum {
	DEBUG_ENABLE = 1U,
};
static int debug_mask = 0;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static void iqs128_report_event(struct iqs128_data *iqs128_data, int value)
{
	input_report_abs(iqs128_data->input, ABS_DISTANCE, value);
	input_sync(iqs128_data->input);
}

static void iqs128_work_func(struct work_struct *work)
{
	int i;
	int object_detect = 0;
	struct iqs128_data *iqs128_data =
			container_of(work, struct iqs128_data, work);

	for (i = 0; i < iqs128_data->pdata->num_data; i++) {
		if (gpio_get_value_cansleep(iqs128_data->pdata->gpio_data[i].irq_gpio)) {
			if (debug_mask & DEBUG_ENABLE)
				printk(KERN_INFO "iqs128 psensor detected"
						"the object is approaching. gpio:%d\n",
						iqs128_data->pdata->gpio_data[i].irq_gpio);
			object_detect |= 1;
		} else {
			if (debug_mask & DEBUG_ENABLE)
				printk(KERN_INFO "iqs128 psensor detected"
						"the object is away. gpio:%d\n",
						iqs128_data->pdata->gpio_data[i].irq_gpio);
		}
	}

	iqs128_report_event(iqs128_data ,object_detect);
}

static irqreturn_t iqs128_int_handler(int irq, void *dev_id)
{
	int i;
	int gpio = -1;
	struct iqs128_data *iqs128_data = dev_id;

	for (i = 0; i < iqs128_data->pdata->num_data; i++) {
		if (iqs128_data->pdata->gpio_data[i].irq == irq) {
			gpio = iqs128_data->pdata->gpio_data[i].irq_gpio;
			break;
		}
	}

	schedule_work(&iqs128_data->work);

	if (debug_mask & DEBUG_ENABLE)
		printk(KERN_INFO "iqs128 psensor gpio %d send interrupt!!!!\n", gpio);

	return IRQ_HANDLED;
}

static int __devinit iqs128_probe(struct platform_device *pdev)
{
	int i;
	int error;
	struct input_dev *dev;
	struct iqs128_data *iqs128_data;
	struct iqs128_platform_data *pdata = pdev->dev.platform_data;

	printk(KERN_INFO "psensor iqs128 probe!!\n");

	iqs128_data = kzalloc(sizeof(struct iqs128_data), GFP_KERNEL);
	if (!iqs128_data) {
		error = -ENOMEM;
		goto exit;
	}

	iqs128_data->pdata = pdata;

	platform_set_drvdata(pdev, iqs128_data);

	dev = input_allocate_device();
	if (!dev) {
		error = -ENOMEM;
		goto kfree;
	}

	dev->name = "capacitive p-sensor";

	set_bit(EV_REL, dev->evbit);
	input_set_capability(dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(dev, ABS_DISTANCE, 0, 2, 0, 0);

	input_set_drvdata(dev, iqs128_data);

	error = input_register_device(dev);
	if (error < 0) {
		input_free_device(dev);
		goto kfree;
	}

	iqs128_data->input = dev;

	for (i = 0; i < pdata->num_data; i++) {
		if (gpio_is_valid(pdata->gpio_data[i].irq_gpio)) {
			error = gpio_request(pdata->gpio_data[i].irq_gpio,
					"iqs128_irq_gpio");
			if (error) {
				dev_err(&pdev->dev, "unable to request gpio [%d]\n",
						pdata->gpio_data[i].irq_gpio);
				goto kfree;
			}
			error = gpio_direction_input(pdata->gpio_data[i].irq_gpio);
			if (error) {
				dev_err(&pdev->dev, "unable to set direction for gpio [%d]\n",
						pdata->gpio_data[i].irq_gpio);
				goto kfree;
			}

			error = request_threaded_irq(pdata->gpio_data[i].irq, NULL,
					iqs128_int_handler, pdata->gpio_data[i].irqflags,
					pdev->dev.driver->name, iqs128_data);
			if (error) {
				dev_err(&pdev->dev, "Failed to register interrupt\n");
				goto kfree;
			}
		}
	}

	INIT_WORK(&iqs128_data->work, iqs128_work_func);

	return 0;
kfree:
	kfree(iqs128_data);
exit:
	return error;
}

static int __devexit iqs128_remove(struct platform_device *pdev)
{
	int i;
	struct iqs128_data *iqs128_data = platform_get_drvdata(pdev);

	for (i = 0; i < iqs128_data->pdata->num_data; i++) {
		free_irq(iqs128_data->pdata->gpio_data[i].irqflags, iqs128_data);
		gpio_free(iqs128_data->pdata->gpio_data[i].irq_gpio);
	}
	kfree(iqs128_data);

	printk(KERN_INFO "psensor iqs128 remove!!\n");

	return 0;
}


static struct of_device_id iqs128_of_match[] = {
	{ .compatible = "iqs128", },
	{ },
};
MODULE_DEVICE_TABLE(of, iqs128_psensor_of_match);

static struct platform_driver iqs128_device_driver = {
	.probe		= iqs128_probe,
	.remove		= __devexit_p(iqs128_remove),
	.driver		= {
		.name	= "iqs128",
		.owner	= THIS_MODULE,
		.of_match_table = iqs128_of_match,
	}
};

static int __init iqs128_init(void)
{
	return platform_driver_register(&iqs128_device_driver);
}

static void __exit iqs128_exit(void)
{
	platform_driver_unregister(&iqs128_device_driver);
}

MODULE_DESCRIPTION("iqs128 proximity sensor driver");
MODULE_LICENSE("GPL");

module_init(iqs128_init);
module_exit(iqs128_exit);

