#ifndef	__HSCDTD007A_I2C_H__
#define	__HSCDTD007A_I2C_H__

#include <linux/device.h>

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
#define HSCDTD_RANGE_MAX 4800
#define HSCDTD_RANGE_MIN (-4800)

#define OUTPUT_RATE_MASK 0x18
#define ODR_0_5          0x00
#define ODR_10           0x01
#define ODR_20           0x10
#define ODR_100          0x11

#ifdef	__KERNEL__
struct hscd_platform_data {
	int poll_interval;
	int min_interval;

	u8 g_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(struct device *);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio_int;
};
#endif	/* __KERNEL__ */

#endif	/* __HSCDTD007A_I2C_H__ */
