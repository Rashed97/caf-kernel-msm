/*
 * Summit Microelectronics SMB347 Battery Charger Driver
 *
 * Copyright (C) 2011, Intel Corporation
 *
 * Authors: Bruce E. Robertson <bruce.e.robertson@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/power/smb347-charger.h>
#include <linux/seq_file.h>
#include <linux/wakelock.h>
#include <linux/delay.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS				0x01
#define CFG_VARIOUS_VCHG			BIT(0)
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_USB_HC			0x10
#define CFG_REG_EN				0x20
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_LOW_BATTERY			BIT(2)
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_LOW_BATTERY		BIT(0)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_B_HC_ENABLE				0x01
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_B				0x36
#define IRQSTAT_B_LOW_BATTERY_IRQ		BIT(3)
#define IRQSTAT_B_LOW_BATTERY_STAT		BIT(2)
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
#define IRQSTAT_F				0x3a

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_E					0x3f

#define SW_COLD_PROTECT_CURRENT 		700000
#define SW_HOT_PROTECT_CURRENT			1200000
#define I2C_RETRY_TIMES				10

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @dentry: for debugfs
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	bool			mains_online;
	bool			usb_online;
	bool			charging_enabled;
	bool			is_suspend;
	bool			is_early_suspend;
	bool			is_temperature_protect;
	struct dentry		*dentry;
	const struct 		smb347_charger_platform_data *pdata;
	int			charger_type_flags;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct			early_suspend early_suspend;
#endif
};

/* Fast charge current in uA */
static const unsigned int fcc_tbl[] = {
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Pre-charge current in uA */
static const unsigned int pcc_tbl[] = {
	100000,
	150000,
	200000,
	250000,
};

/* Termination current in uA */
static const unsigned int tc_tbl[] = {
	37500,
	50000,
	100000,
	150000,
	200000,
	250000,
	500000,
	600000,
};

/* Input current limit in uA */
static const unsigned int icl_tbl[] = {
	300000,
	500000,
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Charge current compensation in uA */
static const unsigned int ccc_tbl[] = {
	250000,
	700000,
	900000,
	1200000,
};

struct smb347_charger *the_chip;
static struct wake_lock smb_lock;
static bool wakelock_smb_count;

/* USB calls these to tell us how much max usb current the system can draw */
void smb347_charger_vbus_draw(unsigned int mA)
{
	bool charge	= false;
	const struct smb347_charger_platform_data *pdata = the_chip->pdata;

	pr_debug("Enter charge=%d\n", mA);

	if (mA == IDEV_CHG_MIN){
		the_chip->charger_type_flags = POWER_SUPPLY_CHARGER_USB;
		the_chip->usb_online = 1;
		power_supply_set_online(&the_chip->usb, the_chip->usb_online);
		power_supply_changed(&the_chip->usb);
		wakelock_smb_count = true;
		wake_lock(&smb_lock);
	}
	else if (mA == IDEV_CHG_MAX){
		the_chip->charger_type_flags = POWER_SUPPLY_CHARGER_AC;
		the_chip->mains_online = 1;
		power_supply_set_online(&the_chip->mains, the_chip->mains_online);
		power_supply_changed(&the_chip->mains);
		wakelock_smb_count = true;
		wake_lock(&smb_lock);
	}
	else{
		charge = pm8921_is_usb_chg_plugged_in();
		if (charge == -EINVAL)
			charge = 0;

		if (!charge){
				if (wakelock_smb_count == true){
					wake_unlock(&smb_lock);
					wakelock_smb_count = false;
				}
				the_chip->mains_online = 0;
				the_chip->usb_online = 0;
				the_chip->charger_type_flags = POWER_SUPPLY_CHARGER_REMOVE;
                                power_supply_set_online(&the_chip->mains, the_chip->mains_online);
                                power_supply_set_online(&the_chip->usb, the_chip->usb_online);
                                power_supply_changed(&the_chip->mains);
                                power_supply_changed(&the_chip->usb);
				power_supply_changed(&the_chip->battery);

				if(the_chip->is_early_suspend){
					pdata->enable_power(0);
					the_chip->is_suspend = true;
					pr_info("power off smb347\n");
				}	
		}
	}
}
EXPORT_SYMBOL_GPL(smb347_charger_vbus_draw);

/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return -EINVAL;
	return tbl[val];
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
	size_t i;
	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

static int smb347_read(struct smb347_charger *smb, u8 reg)
{
	int retry, ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);

	if (ret < 0)
	{
		for(retry = 0; retry < I2C_RETRY_TIMES; retry++)
		{
			msleep(20);
			ret = i2c_smbus_read_byte_data(smb->client, reg);
			pr_info("%s i2c retry %d, reg 0x%x, ret 0x=%x\n", __FUNCTION__, retry, reg, ret);
			if (ret < 0)
				continue;
			else
				break;
		}
	}

	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val)
{
	int retry, ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);

	if (ret < 0)
	{
		for(retry = 0; retry < I2C_RETRY_TIMES; retry++)
		{
			msleep(20);
			ret = i2c_smbus_write_byte_data(smb->client, reg, val);
			pr_info("%s i2c retry %d, reg 0x%x, ret 0x=%x\n", __FUNCTION__, retry, reg, ret);
			if (ret < 0)
				continue;
			else
				break;
		}
	}

	return ret;
}


/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

void update_charger_type(struct smb347_charger *smb)
{
	int ret, cfg_ret, cmd_ret;
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	static bool charging_gpio = false;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
                return;

	cfg_ret = smb347_read(smb, CFG_PIN);
	cfg_ret &= ~CFG_PIN_EN_CTRL_MASK;

	cmd_ret = smb347_read(smb, CMD_B);

	if(smb->mains_online){
		cfg_ret &= ~CFG_PIN_EN_CTRL_USB_HC;
		cfg_ret |= CFG_REG_EN;
		cmd_ret |= CMD_B_HC_ENABLE;
	}
	else if(smb->usb_online){
		cfg_ret |= CFG_PIN_EN_CTRL_USB_HC;
		cfg_ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		cmd_ret &= ~CMD_B_HC_ENABLE;

		if (charging_gpio == false){
			pdata->enable_charging(1);
			charging_gpio = true;
		}
	}
	else{
		cfg_ret |= CFG_PIN_EN_CTRL_USB_HC;
		cfg_ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		cmd_ret &= ~CMD_B_HC_ENABLE;
		if (charging_gpio == true){
			pdata->enable_charging(0);
			charging_gpio = false;
		}
	}

	/* Disable Automatic Power Source Detection (APSD) interrupt. */
	cfg_ret &= ~CFG_PIN_EN_APSD_IRQ;

	smb347_write(smb, CFG_PIN, cfg_ret);
	smb347_write(smb, CMD_B, cmd_ret);
}

/**
 * smb347_update_status - updates the charging status
 * @smb: pointer to smb347 charger instance
 *
 * Function checks status of the charging and updates internal state
 * accordingly. Returns %0 if there is no change in status, %1 if the
 * status has changed and negative errno in case of failure.
 */
static int smb347_update_status(struct smb347_charger *smb)
{
	bool charge	= false;
	bool usb	= false;
	bool dc		= false;
	int ret = 0;

	/*
	 * Dc and usb are set depending on whether they are enabled in
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if ((smb->pdata->use_usb) || (smb->pdata->use_mains))		
		charge = pm8921_is_usb_chg_plugged_in();

        if (charge == -EINVAL)
               charge = 0;

	if ((the_chip->charger_type_flags == POWER_SUPPLY_CHARGER_AC) && charge)
	{
		dc  = 1;
		usb = 0;
	}
	else if ((the_chip->charger_type_flags == POWER_SUPPLY_CHARGER_USB) && charge)
	{
		dc  = 0;
		usb = 1;
	}
	else if (!charge) {
		dc  = 0;
		usb = 0;
	}

	pr_debug("%s dc=%d usb=%d charge=%d charger_type=%d\n", __FUNCTION__, dc,
		usb, charge, the_chip->charger_type_flags);
	pr_debug(" wakelock_smb_count %d mains_online %d usb_online %d\n", wakelock_smb_count, the_chip->mains_online, the_chip->usb_online);
	mutex_lock(&smb->lock);	

        if ((smb->usb_online != usb) || (smb->mains_online != dc))
		ret = 1;

	if (!(smb->is_suspend))
		update_charger_type(smb);

	mutex_unlock(&smb->lock);

	return ret;
}

/*
 * smb347_is_online - returns whether input power source is connected
 * @smb: pointer to smb347 charger instance
 *
 * Returns %true if input power source is connected. Note that this is
 * dependent on what platform has configured for usable power sources. For
 * example if USB is disabled, this will return %false even if the USB
 * cable is connected.
 */
static bool smb347_is_online(struct smb347_charger *smb)
{
	bool ret;

	mutex_lock(&smb->lock);
	ret = smb->usb_online || smb->mains_online;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_charging_status - returns status of charging
 * @smb: pointer to smb347 charger instance
 *
 * Function returns charging status. %0 means no charging is in progress,
 * %1 means pre-charging, %2 fast-charging and %3 taper-charging.
 */
static int smb347_charging_status(struct smb347_charger *smb)
{
	int ret;

	if(smb->is_suspend)
		return 0;

	if (!smb347_is_online(smb))
		return 0;

	ret = smb347_read(smb, STAT_C);
	if (ret < 0)
		return 0;

	return (ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
}

static int smb347_charging_set(struct smb347_charger *smb, bool enable)
{
	int ret = 0;

	if (smb->pdata->enable_control != SMB347_CHG_ENABLE_SW) {
		dev_dbg(&smb->client->dev,
			"charging enable/disable in SW disabled\n");
		return 0;
	}

	mutex_lock(&smb->lock);
	if (smb->charging_enabled != enable) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb->charging_enabled = enable;

		if (enable)
			ret |= CMD_A_CHG_ENABLED;
		else
			ret &= ~CMD_A_CHG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
	}
out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_charging_enable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, true);
}

static inline int smb347_charging_disable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, false);
}

static int smb347_update_online(struct smb347_charger *smb)
{
	int ret;
	const struct smb347_charger_platform_data *pdata = smb->pdata;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (smb347_is_online(smb)) {	
		pdata->enable_charging(1);
		ret = smb347_charging_enable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to enable charging\n");

	} else {
		pdata->enable_charging(0);
		ret = smb347_charging_disable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to disable charging\n");
	}
	return ret;
}

static int smb347_set_charge_current(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CHARGE_CURRENT);
	if (ret < 0)
		return ret;

	if (smb->pdata->max_charge_current) {
		val = current_to_hw(fcc_tbl, ARRAY_SIZE(fcc_tbl),
				    smb->pdata->max_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_FCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_FCC_SHIFT;
	}

	if (smb->pdata->pre_charge_current) {
		val = current_to_hw(pcc_tbl, ARRAY_SIZE(pcc_tbl),
				    smb->pdata->pre_charge_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_PCC_MASK;
		ret |= val << CFG_CHARGE_CURRENT_PCC_SHIFT;
	}

	if (smb->pdata->termination_current) {
		val = current_to_hw(tc_tbl, ARRAY_SIZE(tc_tbl),
				    smb->pdata->termination_current);
		if (val < 0)
			return val;

		ret &= ~CFG_CHARGE_CURRENT_TC_MASK;
		ret |= val;
	}
	return smb347_write(smb, CFG_CHARGE_CURRENT, ret);
}

static int smb347_set_current_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->mains_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->mains_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_DC_MASK;
		ret |= val << CFG_CURRENT_LIMIT_DC_SHIFT;
	}

	if (smb->pdata->usb_hc_current_limit) {
		val = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				    smb->pdata->usb_hc_current_limit);
		if (val < 0)
			return val;

		ret &= ~CFG_CURRENT_LIMIT_USB_MASK;
		ret |= val;
	}
	return smb347_write(smb, CFG_CURRENT_LIMIT, ret);
}

static int smb347_set_voltage_limits(struct smb347_charger *smb)
{
	int ret, val;

	ret = smb347_read(smb, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		return ret;

	if (smb->pdata->pre_to_fast_voltage) {
		val = smb->pdata->pre_to_fast_voltage;

		/* uV */
		val = clamp_val(val, 2400000, 3000000) - 2400000;
		val /= 200000;

		ret &= ~CFG_FLOAT_VOLTAGE_THRESHOLD_MASK;
		ret |= val << CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT;
	}

	if (smb->pdata->max_charge_voltage) {
		val = smb->pdata->max_charge_voltage;

		/* uV */
		val = clamp_val(val, 3500000, 4500000) - 3500000;
		val /= 20000;

		ret |= val;
	}

	return smb347_write(smb, CFG_FLOAT_VOLTAGE, ret);
}

static int smb347_set_temp_limits(struct smb347_charger *smb)
{
	bool enable_therm_monitor = false;
	int ret, val;

	if (smb->pdata->chip_temp_threshold) {
		val = smb->pdata->chip_temp_threshold;

		/* degree C */
		val = clamp_val(val, 100, 130) - 100;
		val /= 10;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_TEMP_THRESHOLD_MASK;
		ret |= val << CFG_OTG_TEMP_THRESHOLD_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	ret = smb347_read(smb, CFG_TEMP_LIMIT);
	if (ret < 0)
		return ret;

	if (smb->pdata->soft_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_cold_temp_limit;

		val = clamp_val(val, 0, 15);
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_SOFT_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->soft_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->soft_hot_temp_limit;

		val = clamp_val(val, 40, 55) - 40;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_SOFT_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_SOFT_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_cold_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_cold_temp_limit;

		val = clamp_val(val, -5, 10) + 5;
		val /= 5;
		/* this goes from higher to lower so invert the value */
		val = ~val & 0x3;

		ret &= ~CFG_TEMP_LIMIT_HARD_COLD_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_COLD_SHIFT;

		enable_therm_monitor = true;
	}

	if (smb->pdata->hard_hot_temp_limit != SMB347_TEMP_USE_DEFAULT) {
		val = smb->pdata->hard_hot_temp_limit;

		val = clamp_val(val, 50, 65) - 50;
		val /= 5;

		ret &= ~CFG_TEMP_LIMIT_HARD_HOT_MASK;
		ret |= val << CFG_TEMP_LIMIT_HARD_HOT_SHIFT;

		enable_therm_monitor = true;
	}

	ret = smb347_write(smb, CFG_TEMP_LIMIT, ret);
	if (ret < 0)
		return ret;

	/*
	 * If any of the temperature limits are set, we also enable the
	 * thermistor monitoring.
	 *
	 * When soft limits are hit, the device will start to compensate
	 * current and/or voltage depending on the configuration.
	 *
	 * When hard limit is hit, the device will suspend charging
	 * depending on the configuration.
	 */
	if (enable_therm_monitor) {
		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_MONITOR_DISABLED;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->suspend_on_hard_temp_limit) {
		ret = smb347_read(smb, CFG_SYSOK);
		if (ret < 0)
			return ret;

		ret &= ~CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED;

		ret = smb347_write(smb, CFG_SYSOK, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->soft_temp_limit_compensation !=
	    SMB347_SOFT_TEMP_COMPENSATE_DEFAULT) {
		val = smb->pdata->soft_temp_limit_compensation & 0x3;

		ret = smb347_read(smb, CFG_THERM);
		if (ret < 0)
			return ret;

		ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT;

		ret &= ~CFG_THERM_SOFT_COLD_COMPENSATION_MASK;
		ret |= val << CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_THERM, ret);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->charge_current_compensation) {

		val = current_to_hw(ccc_tbl, ARRAY_SIZE(ccc_tbl),
				    smb->pdata->charge_current_compensation);

		if (battery_temperature <= smb->pdata->soft_cold_temp_limit)
			val = current_to_hw(ccc_tbl, ARRAY_SIZE(ccc_tbl), SW_COLD_PROTECT_CURRENT);
		else
			val = current_to_hw(ccc_tbl, ARRAY_SIZE(ccc_tbl), SW_HOT_PROTECT_CURRENT);

		if (val < 0)
			return val;

		ret = smb347_read(smb, CFG_OTG);
		if (ret < 0)
			return ret;

		ret &= ~CFG_OTG_CC_COMPENSATION_MASK;
		ret |= (val & 0x3) << CFG_OTG_CC_COMPENSATION_SHIFT;

		ret = smb347_write(smb, CFG_OTG, ret);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int smb347_hw_init(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Program the platform specific configuration values to the device
	 * first.
	 */
	ret = smb347_read(smb, CFG_VARIOUS);
	if (ret < 0)
		goto fail;
	ret &= ~CFG_VARIOUS_VCHG;
	ret = smb347_write(smb, CFG_VARIOUS, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_set_charge_current(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_current_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_voltage_limits(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_set_temp_limits(smb);
	if (ret < 0)
		goto fail;

	/* If USB charging is disabled we put the USB in suspend mode */
	if (!smb->pdata->use_usb) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto fail;

		ret |= CMD_A_SUSPEND_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto fail;
	}

	ret = smb347_read(smb, CFG_OTHER);
	if (ret < 0)
		goto fail;

	/*
	 * If configured by platform data, we enable hardware Auto-OTG
	 * support for driving VBUS. Otherwise we disable it.
	 */
	ret &= ~CFG_OTHER_RID_MASK;
	if (smb->pdata->use_usb_otg)
		ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;

	ret &= ~CFG_OTHER_LOW_BATTERY;

	ret = smb347_write(smb, CFG_OTHER, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_read(smb, CFG_PIN);
	if (ret < 0)
		goto fail;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;

	switch (smb->pdata->enable_control) {
	case SMB347_CHG_ENABLE_SW:
		/* Do nothing, 0 means i2c control */
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_LOW:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
		break;
	case SMB347_CHG_ENABLE_PIN_ACTIVE_HIGH:
		ret |= CFG_PIN_EN_CTRL_ACTIVE_HIGH;
		break;
	}

	/* Disable Automatic Power Source Detection (APSD) interrupt. */
	ret &= ~CFG_PIN_EN_APSD_IRQ;

	ret = smb347_write(smb, CFG_PIN, ret);
	if (ret < 0)
		goto fail;

	ret = smb347_update_status(smb);
	if (ret < 0)
		goto fail;

	ret = smb347_update_online(smb);

fail:

	return ret;
}

static irqreturn_t smb347_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;
	int irqstat_b;
	int stat_c, irqstat_e, irqstat_c;
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	irqreturn_t ret = IRQ_NONE;

  	if(the_chip->is_suspend){
		pdata->enable_power(1);
		msleep(500);
	}

	irqstat_b = smb347_read(smb, IRQSTAT_B);
	if (irqstat_b < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_B failed\n");
		return IRQ_NONE;
	}

	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_c = smb347_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	/*
	 * If we get charger error we report the error back to user and
	 * disable charging.
	 */
	if (stat_c & STAT_C_CHARGER_ERROR) {
		dev_err(&smb->client->dev,
			"error in charger, disabling charging\n");
		smb347_charging_disable(smb);
		power_supply_changed(&smb->battery);

		ret = IRQ_HANDLED;
	}

	/*
	 * When Low battery, SMB347 will send a interrupt to wake up devices
	 * so that devices can power down when ultra-low battery
	 */
	if (irqstat_b & (IRQSTAT_B_LOW_BATTERY_IRQ | IRQSTAT_B_LOW_BATTERY_STAT)) {
		if (!pm8921_is_usb_chg_plugged_in())
			pm_power_off();
	}


	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 */
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ | IRQSTAT_C_TAPER_IRQ)) {
		if (irqstat_c & IRQSTAT_C_TERMINATION_STAT)
			power_supply_changed(&smb->battery);
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (!(irqstat_e & (IRQSTAT_E_USBIN_UV_STAT))){
		if (smb347_update_status(smb) > 0) {
			smb347_update_online(smb);
		}
		ret = IRQ_HANDLED;
	}

  	if(the_chip->is_suspend){
		pdata->enable_power(0);
	}
	
	return ret;
}

static int smb347_irq_set(struct smb347_charger *smb, bool enable)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Enable/disable interrupts for:
	 *	- under voltage
	 *	- termination current reached
	 *	- charger error
	 */
	if (enable) {
		ret = smb347_write(smb, CFG_FAULT_IRQ, CFG_FAULT_IRQ_DCIN_UV);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ,
				   CFG_STATUS_IRQ_TERMINATION_OR_TAPER | 
				   CFG_STATUS_IRQ_LOW_BATTERY);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	} else {
		ret = smb347_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	}

fail:
	return ret;
}

static inline int smb347_irq_enable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, true);
}

static inline int smb347_irq_disable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, false);
}

static int smb347_irq_init(struct smb347_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	gpio_tlmm_config(GPIO_CFG(pdata->irq_gpio, 0, GPIO_CFG_INPUT,
                       GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING, smb->client->name,
				   smb);
	if (ret < 0)
		goto fail_gpio;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto fail_irq;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb347_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb347_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb347_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

	smb->client->irq = irq;
	return 0;

fail_readonly:

fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb347_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, mains);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->mains_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->usb_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb347_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb347_charger *smb =
			container_of(psy, struct smb347_charger, battery);
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;
	bool charge;

	charge = pm8921_is_usb_chg_plugged_in();
	if (charge == -EINVAL)
		charge = 0;

	if((!(the_chip->is_suspend)) || charge)
		ret = smb347_update_status(smb);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!smb347_is_online(smb)) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		}
		if (smb347_charging_status(smb))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_FULL;
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		/*
		 * We handle trickle and pre-charging the same, and taper
		 * and none the same.
		 */
		switch (smb347_charging_status(smb)) {
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = 3200000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = 4200000;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_mvolts;
		break;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if(smb->is_suspend){
			val->intval = 0;
			break;
		}

		ret = smb347_read(smb, STAT_B);
		/*
		 * The current value is composition of FCC and PCC values
		 * and we can detect which table to use from bit 5.
		 */
		if (ret & 0x20) {
			val->intval = hw_to_current(fcc_tbl,
						    ARRAY_SIZE(fcc_tbl),
						    ret & 7);
		} else {
			ret >>= 3;
			val->intval = hw_to_current(pcc_tbl,
						    ARRAY_SIZE(pcc_tbl),
						    ret & 7);
		}
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = battery_capacity;
		break;

	case POWER_SUPPLY_PROP_TEMP:
		val->intval = battery_temperature;

		if(smb->is_suspend)
			break;

		if (((battery_temperature <= (smb->pdata->soft_cold_temp_limit + 5)) ||
			(battery_temperature >= (smb->pdata->soft_hot_temp_limit -5))) &&
			(smb->is_temperature_protect == false))
		{
			smb->is_temperature_protect = true;
			smb347_set_temp_limits(smb);
		}
		else if ((battery_temperature > smb->pdata->soft_cold_temp_limit) &&
			(battery_temperature < smb->pdata->soft_hot_temp_limit))
			smb->is_temperature_protect = false;

		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb347_debugfs_show(struct seq_file *s, void *data)
{
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	ret = smb347_read(smb, CMD_A);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_A, ret);
	ret = smb347_read(smb, CMD_B);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_B, ret);
	ret = smb347_read(smb, CMD_C);
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_C, ret);
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release        = single_release,
};

static int smb347_enable_set(void *data, u64 val)
{
	int ret;

	the_chip->charging_enabled = val;

        ret = smb347_read(the_chip, CMD_A);

	if (!the_chip->charging_enabled)
        	ret |= CMD_A_SUSPEND_ENABLED;
	else
        	ret &= ~CMD_A_SUSPEND_ENABLED;

        ret = smb347_write(the_chip, CMD_A, ret);

	return 0;
}

static int smb347_enable_get(void *data, u64 *val)
{
	*val = the_chip->charging_enabled;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(smb_enable_fops, smb347_enable_get,
			smb347_enable_set, "%llu\n");

static void smb347_debugfs_init(void)
{
  static struct dentry *debugfs_smb;

  the_chip->charging_enabled = 1;

  debugfs_smb = debugfs_create_dir("smb347", NULL);

  debugfs_create_file("enable", 0644, debugfs_smb, NULL,
			    &smb_enable_fops);

}

#ifdef CONFIG_HAS_EARLYSUSPEND
void smb347_early_suspend(struct early_suspend *h)
{
	const struct smb347_charger_platform_data *pdata = the_chip->pdata;

	pr_info("%s: enter\n", __func__);
	the_chip->is_early_suspend = true;

	if(!((wakelock_smb_count) || (smb347_is_online(the_chip)))){
		pdata->enable_power(0);
		the_chip->is_suspend = true;
		pr_info("power off smb347\n");
	}
}

void smb347_late_resume(struct early_suspend *h)
{
	const struct smb347_charger_platform_data *pdata = the_chip->pdata;

	pr_info("%s: enter\n", __func__);
	the_chip->is_early_suspend = false;

	if(the_chip->is_suspend){
		pdata->enable_power(1);
		msleep(200);
		smb347_hw_init(the_chip);
		the_chip->is_suspend = false;
		pr_info("power on smb347\n");
	}
}
#endif

static int smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	static char *battery[] = { "smb347-battery" };
	const struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;

	pdata = dev->platform_data;

	pdata->platform_init(&client->dev);

	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	the_chip = devm_kzalloc(dev, sizeof(*the_chip), GFP_KERNEL);
	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);

	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

	the_chip = smb;

	ret = smb347_hw_init(smb);
	if (ret < 0)
		return ret;

	smb->mains.name = "smb347-mains";
	smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
	smb->mains.get_property = smb347_mains_get_property;
	smb->mains.properties = smb347_mains_properties;
	smb->mains.num_properties = ARRAY_SIZE(smb347_mains_properties);
	smb->mains.supplied_to = battery;
	smb->mains.num_supplicants = ARRAY_SIZE(battery);

	smb->usb.name = "smb347-usb";
	smb->usb.type = POWER_SUPPLY_TYPE_USB;
	smb->usb.get_property = smb347_usb_get_property;
	smb->usb.properties = smb347_usb_properties;
	smb->usb.num_properties = ARRAY_SIZE(smb347_usb_properties);
	smb->usb.supplied_to = battery;
	smb->usb.num_supplicants = ARRAY_SIZE(battery);

	smb->battery.name = "smb347-battery";
	smb->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	smb->battery.get_property = smb347_battery_get_property;
	smb->battery.properties = smb347_battery_properties;
	smb->battery.num_properties = ARRAY_SIZE(smb347_battery_properties);

	ret = power_supply_register(dev, &smb->mains);
	if (ret < 0)
		return ret;

	ret = power_supply_register(dev, &smb->usb);
	if (ret < 0) {
		return ret;
	}

	ret = power_supply_register(dev, &smb->battery);
	if (ret < 0) {
		power_supply_unregister(&smb->usb);
		power_supply_unregister(&smb->mains);
		return ret;
	}

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb347_irq_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	smb->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	smb->early_suspend.suspend = smb347_early_suspend;
	smb->early_suspend.resume = smb347_late_resume;
	register_early_suspend(&smb->early_suspend);
#endif

	smb->dentry = debugfs_create_file("smb347-regs", S_IRUSR, NULL, smb,
					  &smb347_debugfs_fops);
	
	smb->is_suspend=false;

	smb347_debugfs_init();

	wake_lock_init(&smb_lock, WAKE_LOCK_SUSPEND, "smb_lock");

	return 0;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	if (client->irq) {
		smb347_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}

	power_supply_unregister(&smb->battery);
	power_supply_unregister(&smb->usb);
	power_supply_unregister(&smb->mains);
	return 0;
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_driver = {
	.driver = {
		.name = "smb347",
	},
	.probe        = smb347_probe,
	.remove       = __devexit_p(smb347_remove),
	.id_table     = smb347_id,
};

static int __init smb347_init(void)
{
	return i2c_add_driver(&smb347_driver);
}
module_init(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
