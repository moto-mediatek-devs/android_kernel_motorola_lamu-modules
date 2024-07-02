#ifndef __TINNO_CHARGER_H__
#define __TINNO_CHARGER_H__

#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0))
#include "charger_class.h"
#elif (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#include <mt-plat/v1/charger_class.h>
#else
#include <mt-plat/charger_class.h>
#endif /* LINUX_VERSION_CODE */


#define NORMAL_UPDATE_MS			5000 /*ms*/
#define SHORT_UPDATE_MS				2500 /*ms*/

#define TINNO_CHARGER_ERR_LEVEL			1
#define TINNO_CHARGER_INFO_LEVEL		2
#define TINNO_CHARGER_DBG_LEVEL			3

static int log_level = TINNO_CHARGER_DBG_LEVEL;
module_param(log_level, int, 0644);

int tinno_charger_get_log_level(void)
{
	return log_level;
}

#define TINNO_CHARGER_DBG(fmt, ...) \
	do { \
		if (tinno_charger_get_log_level() >= TINNO_CHARGER_DBG_LEVEL) \
			pr_info("[TINNO_CHARGER]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define TINNO_CHARGER_INFO(fmt, ...) \
	do { \
		if (tinno_charger_get_log_level() >= TINNO_CHARGER_INFO_LEVEL) \
			pr_info("[TINNO_CHARGER]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

#define TINNO_CHARGER_ERR(fmt, ...) \
	do { \
		if (tinno_charger_get_log_level() >= TINNO_CHARGER_ERR_LEVEL) \
			pr_info("[TINNO_CHARGER]%s " fmt, __func__, ##__VA_ARGS__); \
	} while (0)

struct switch_device {
	bool	is_enabled;
	bool	charger_exist;
	int	charger_type;
	int	charger_voltage;
	int	prop_status;
	int	boot_mode;
};

struct charger_pump_device {
	bool	is_enabled;
};

struct battery_device {
	int		temp;
	int		vbat;
	int		ibat;
	int		vbus;
	int		ibus;
	int		ui_soc;
	int		soc;

	int		batt_exist;
	int		batt_full;
	int		chging_on;
	int		in_rechging;
	int		charging_state;
};

struct tinno_charger_info {
	struct device			*dev;

	/* work */
	struct delayed_work		charger_monitor_work;

	/* charger device */
	struct charger_device		*sw_chg;
	struct charger_device		*cp_chg;

	struct switch_device		sw_dev;
	struct charger_pump_device	cp_dev;
	struct battery_device		batt_dev;

	/* power supply device */
	struct power_supply		*batt_psy;
	struct power_supply		*sw_psy;
	struct power_supply		*cp_psy;
};

#endif /* __TINNO_CHARGER_H__ */
