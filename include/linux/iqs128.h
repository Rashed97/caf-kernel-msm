
#ifndef _SQR128_H_
#define _SQR128_H_

enum {
	TYPE_RF,
	TYPE_WIFI,
};

struct iqs128_gpio_data {
	int irq_gpio;
	int irq;
	int irqflags;
	int status;
	int type;
};

struct iqs128_platform_data {
	int num_data;
	struct iqs128_gpio_data *gpio_data;
};
#endif
