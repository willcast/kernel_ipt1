/*
 *  linux/arch/arm/mach-apple_iphone/iphone.c
 *
 *  Copyright (C) 2008 Yiduo Wang
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/time.h>
#include <asm/mach/irq.h>

#include "core.h"
#include "lcd.h"
#include <mach/iphone-dma.h>
#include <mach/iphone-i2c.h>
#include <mach/usb.h>

#include <linux/platform_device.h>

#include <ftl/nand.h>

#ifdef CONFIG_IPHONE_3G
#include <linux/mfd/pcf50633/core.h>
#include <linux/mfd/pcf50633/pmic.h>
#include <linux/mfd/pcf50633/adc.h>
#include <linux/mfd/pcf50633/mbc.h>
#endif

static struct map_desc iphone_io_desc[] __initdata = {
	{
		.virtual	=  IO_ADDRESS(0x3CC00000),
		.pfn		= __phys_to_pfn(0x3CC00000),
		.length		= SZ_2M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3C500000),
		.pfn		= __phys_to_pfn(0x3C500000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38E00000),
		.pfn		= __phys_to_pfn(0x38E00000),
		.length		= SZ_8K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38E02000),
		.pfn		= __phys_to_pfn(0x38E02000),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3E300000),
		.pfn		= __phys_to_pfn(0x3E300000),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3E200000),
		.pfn		= __phys_to_pfn(0x3E200000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38400000),
		.pfn		= __phys_to_pfn(0x38400000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38D00000),
		.pfn		= __phys_to_pfn(0x38D00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3C400000),
		.pfn		= __phys_to_pfn(0x3C400000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x39A00000),
		.pfn		= __phys_to_pfn(0x39A00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38900000),
		.pfn		= __phys_to_pfn(0x38900000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38A00000),
		.pfn		= __phys_to_pfn(0x38A00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38F00000),
		.pfn		= __phys_to_pfn(0x38F00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x38200000),
		.pfn		= __phys_to_pfn(0x38200000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x39900000),
		.pfn		= __phys_to_pfn(0x39900000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3C600000),
		.pfn		= __phys_to_pfn(0x3C600000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3C900000),
		.pfn		= __phys_to_pfn(0x3C900000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x39A00000),
		.pfn		= __phys_to_pfn(0x39A00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3E400000),
		.pfn		= __phys_to_pfn(0x3E400000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3CA00000),
		.pfn		= __phys_to_pfn(0x3CA00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3CD00000),
		.pfn		= __phys_to_pfn(0x3CD00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3E500000),
		.pfn		= __phys_to_pfn(0x3E500000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3C300000),
		.pfn		= __phys_to_pfn(0x3C300000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3CE00000),
		.pfn		= __phys_to_pfn(0x3CE00000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(0x3D200000),
		.pfn		= __phys_to_pfn(0x3D200000),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
};

void __init iphone_map_io(void)
{
	printk("iphone: initializing io map\n");
	iotable_init(iphone_io_desc, ARRAY_SIZE(iphone_io_desc));
}

#if (defined(CONFIG_IPHONE_3G)||defined(CONFIG_IPHONE_2G)) && defined(CONFIG_MFD_PCF50633)

struct platform_device iphone_backlight = {
	.name           = "pcf50633-backlight",
	.id             = -1,
	.num_resources  = 0,
};

static struct power_supply iphone_battery;
static struct pcf50633 *pcf50633;
static struct {
	struct delayed_work monitor_work;

	int voltage;
	int level;
} iphone_battery_info;

static void iphone_battery_update_status(struct pcf50633 *pcf, void *unused, int res)
{
	const int interval = msecs_to_jiffies(60 * 1000);

	int voltage = (res*6000)/1023;
	int level = ((voltage - 3500) * 100) / (4200 - 3500);
	if(level < 0)
		level = 0;
	if(level > 100)
		level = 100;
	
	dev_info(pcf50633->dev, "V:%d, L:%d\n", voltage, level);

	iphone_battery_info.voltage = voltage;
	iphone_battery_info.level = level;

	power_supply_changed(&iphone_battery);

	schedule_delayed_work(&iphone_battery_info.monitor_work, interval);
}

static void iphone_battery_work(struct work_struct* work)
{
	const int interval = msecs_to_jiffies(60 * 1000);

	if(platform_get_drvdata(pcf50633->adc_pdev) == NULL || pcf50633_adc_async_read(pcf50633, PCF50633_ADCC1_MUX_BATSNS_RES, PCF50633_ADCC1_AVERAGE_16, &iphone_battery_update_status, NULL) < 0)
	{
		dev_err(pcf50633->dev, "failed to get battery level\n");
		schedule_delayed_work(&iphone_battery_info.monitor_work, interval);
	}
}

static int iphone_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:
			if(pcf50633  && platform_get_drvdata(pcf50633->mbc_pdev) && (pcf50633_mbc_get_status(pcf50633) & (PCF50633_MBC_USB_ACTIVE | PCF50633_MBC_ADAPTER_ACTIVE)))
			{
				if(iphone_battery_info.level == 100)
				       	val->intval = POWER_SUPPLY_STATUS_FULL;
				else
				       	val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
			else
			{
			    val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			break;
		case POWER_SUPPLY_PROP_HEALTH:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			/* TODO: we might need to set POWER_SUPPLY_HEALTH_OVERHEAT if we figure out the battery temperature stuff */
			break;
		case POWER_SUPPLY_PROP_PRESENT:
			val->intval = 1;
			break;
		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;
		case POWER_SUPPLY_PROP_CAPACITY:
			val->intval = iphone_battery_info.level;
			break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			val->intval = iphone_battery_info.voltage * 1000;
			break;
		default:
			return -EINVAL;
	}

	return 0;
}

static enum power_supply_property iphone_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};

static char* iphone_batteries[] = {
	"battery",
};

static struct power_supply iphone_battery = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = iphone_battery_properties,
	.num_properties = ARRAY_SIZE(iphone_battery_properties),
	.get_property = iphone_battery_get_property,
};

static void pcf50633_probe_done(struct pcf50633 *_pcf)
{
	int ret;

	pcf50633 = _pcf;

	iphone_battery_info.voltage = 0;
	iphone_battery_info.level = 50;

	ret = power_supply_register(_pcf->dev, &iphone_battery);
	if(ret)
		dev_err(_pcf->dev, "failed to register battery power supply!\n");

	INIT_DELAYED_WORK(&iphone_battery_info.monitor_work, iphone_battery_work);

	iphone_backlight.dev.platform_data = _pcf;
	if(platform_device_register(&iphone_backlight) < 0)
		dev_err(_pcf->dev, "failed to create backlight driver!\n");
	
	iphone_battery_work(NULL);
}

static struct pcf50633_platform_data pcf50633_pdata = {
	.resumers = {
		[0] =	PCF50633_INT1_USBINS |
			PCF50633_INT1_USBREM |
			PCF50633_INT1_ALARM,
		[1] =	PCF50633_INT2_ONKEYF,
		[2] =	PCF50633_INT3_ONKEY1S,
		[3] =	PCF50633_INT4_LOWSYS |
			PCF50633_INT4_LOWBAT |
			PCF50633_INT4_HIGHTMP,
	},

	.reg_init_data = {
		[PCF50633_REGULATOR_AUTO] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.always_on = 1,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
		},
		[PCF50633_REGULATOR_DOWN1] = {
			.constraints = {
				.min_uV = 1300000,
				.max_uV = 1600000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.always_on = 1,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_DOWN2] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.always_on = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
		},
		[PCF50633_REGULATOR_HCLDO] = {
			.constraints = {
				.min_uV = 2000000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE,
				.always_on = 1,
			},
		},
		[PCF50633_REGULATOR_LDO1] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 0,
				},
			},
		},
		[PCF50633_REGULATOR_LDO2] = {
			.constraints = {
				.min_uV = 3300000,
				.max_uV = 3300000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO3] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO4] = {
			.constraints = {
				.min_uV = 3200000,
				.max_uV = 3200000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
			},
		},
		[PCF50633_REGULATOR_LDO5] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.apply_uV = 1,
				.state_mem = {
					.enabled = 1,
				},
			},
		},
		[PCF50633_REGULATOR_LDO6] = {
			.constraints = {
				.min_uV = 3000000,
				.max_uV = 3000000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
			},
		},
		[PCF50633_REGULATOR_MEMLDO] = {
			.constraints = {
				.min_uV = 1800000,
				.max_uV = 1800000,
				.valid_modes_mask = REGULATOR_MODE_NORMAL,
				.state_mem = {
					.enabled = 1,
				},
			},
		},

	},

	.probe_done = &pcf50633_probe_done,
	.mbc_event_callback = NULL,

	.batteries = iphone_batteries,
	.num_batteries = ARRAY_SIZE(iphone_batteries),
};
#endif

static struct i2c_board_info __initdata iphone_i2c0[] = {
#ifdef CONFIG_IPHONE_2G
	{
		I2C_BOARD_INFO("iphone-accel", 0x3a),
	},
	{
#ifdef CONFIG_MFD_PCF50633
		I2C_BOARD_INFO("pcf50633", 0xe6),
		.irq = IPHONE_GPIO_IRQS + 0x55,
		.platform_data = &pcf50633_pdata,
#else
		I2C_BOARD_INFO("iphone-pmu", 0xe6),
#endif
	},
	{
		I2C_BOARD_INFO("tsl2561", 0x92),
	},
	{
		I2C_BOARD_INFO("wm8758", 0x34),
	},
#endif
#ifdef CONFIG_IPHONE_3G
	{
		I2C_BOARD_INFO("iphone-accel", 0x3a),
	},
	{
#ifdef CONFIG_MFD_PCF50633
		I2C_BOARD_INFO("pcf50633", 0xe6),
		.irq = IPHONE_GPIO_IRQS + 0x55,
		.platform_data = &pcf50633_pdata,
#else
		I2C_BOARD_INFO("iphone-pmu", 0xe6),
#endif
	},
	{
		I2C_BOARD_INFO("wm8991", 0x36),
	},
	{
		I2C_BOARD_INFO("isl29003", 0x88),
	},
#endif
#ifdef CONFIG_IPODTOUCH_1G
	{
		I2C_BOARD_INFO("iphone-accel", 0x3a),
	},
#endif
};

static struct i2c_board_info __initdata iphone_i2c1[] = {
#ifdef CONFIG_IPODTOUCH_1G
	{
		I2C_BOARD_INFO("iphone-pmu", 0xe6),
	},
	{
		I2C_BOARD_INFO("wm8758", 0x34),
	},
#endif
};

void __init iphone_init(void)
{
	printk("iphone: platform init\r\n");
	iphone_dma_setup();

	i2c_register_board_info(0, iphone_i2c0, ARRAY_SIZE(iphone_i2c0));
	i2c_register_board_info(1, iphone_i2c1, ARRAY_SIZE(iphone_i2c1));

	platform_device_register(&iphone_dma);
	platform_device_register(&iphone_nand);
	platform_device_register(&iphone_i2c);
}

MACHINE_START(APPLE_IPHONE, "Apple iPhone")
	/* Maintainer: iPhone Linux */
	.phys_io	= 0x38000000,
	.io_pg_offst	= (IO_ADDRESS(0x38000000) >> 18) & 0xfffc,
	.boot_params	= 0x09000000,
	.map_io		= iphone_map_io,
	.init_irq	= iphone_init_irq,
	.timer		= &iphone_timer,
	.init_machine	= iphone_init,
MACHINE_END
