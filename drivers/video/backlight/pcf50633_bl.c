/*
 *  Backlight Driver for the PCF50633 PMU
 *
 *  Copyright (c) 2010 Ricky Taylor
 *  Portions (c) Patrick Wildt, Claudio Nieder.
 *
 *  Based on iphone_bl.c by Patrick Wildt
 *  Based on kb6886_bl.c by Claudio Nieder
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include <linux/backlight.h>

#include <linux/mfd/pcf50633/core.h>

#include <mach/pmu.h>

#define LCD_MAX_BACKLIGHT 45
#define LCD_DEFAULT_BACKLIGHT 25
#define LCD_BACKLIGHT_REGMASK 0x3F

static struct pcf50633 *pcf50633;
static int pcf50633_bl_intensity;
static struct backlight_device *pcf50633_bl_device;

static int pcf50633_bl_send_intensity(struct backlight_device *bd)
{
	int intensity = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		intensity = 0;

	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		intensity = 0;

	if(intensity <= 0)
	{
		pcf50633_reg_write(pcf50633, PCF50633_REG_LEDENA, 0);
		pcf50633_reg_write(pcf50633, PCF50633_REG_LEDCTL, 0);
	}
	else
	{
		pcf50633_reg_write(pcf50633, PCF50633_REG_LEDENA, 1);
		pcf50633_reg_write(pcf50633, PCF50633_REG_LEDOUT, intensity & LCD_BACKLIGHT_REGMASK);
		pcf50633_reg_write(pcf50633, PCF50633_REG_LEDCTL, 6);
	}

	pcf50633_bl_intensity = intensity;
	return 0;
}

static int pcf50633_bl_get_intensity(struct backlight_device *bd)
{
	return pcf50633_bl_intensity;
}

static struct backlight_ops pcf50633_bl_ops = {
	.get_brightness = pcf50633_bl_get_intensity,
	.update_status  = pcf50633_bl_send_intensity,
};

static int pcf50633_bl_probe(struct platform_device *pdev)
{
	pcf50633 = pdev->dev.platform_data;

	//pcf50633_bl_device = backlight_device_register("pcf50633-backlight",
	pcf50633_bl_device = backlight_device_register("iphone-bl",
		&pdev->dev, NULL, &pcf50633_bl_ops);
	if (IS_ERR(pcf50633_bl_device))
		return PTR_ERR(pcf50633_bl_device);

	platform_set_drvdata(pdev, pcf50633_bl_device);

	pcf50633_bl_device->props.max_brightness = LCD_MAX_BACKLIGHT;
	pcf50633_bl_device->props.power = FB_BLANK_UNBLANK;
	pcf50633_bl_device->props.brightness = LCD_DEFAULT_BACKLIGHT;
	backlight_update_status(pcf50633_bl_device);

	pcf50633_reg_write(pcf50633, PCF50633_REG_LEDENA, 1);
	pcf50633_reg_write(pcf50633, PCF50633_REG_LEDOUT, LCD_DEFAULT_BACKLIGHT);
	pcf50633_reg_write(pcf50633, PCF50633_REG_LEDCTL, 6);
	pcf50633_reg_write(pcf50633, PCF50633_REG_LEDDIM, 0x20); // Dimming curve

	dev_info(&pdev->dev, "loaded backlight driver.\n");

	return 0;
}

static int pcf50633_bl_remove(struct platform_device *pdev)
{
	struct backlight_device *bd = platform_get_drvdata(pdev);

	backlight_device_unregister(bd);

	return 0;
}

static struct platform_driver pcf50633_bl_driver = {
	.probe		= pcf50633_bl_probe,
	.remove		= pcf50633_bl_remove,
	.driver		= {
		.name	= "pcf50633-backlight",
	},
};

static int __init mod_init(void)
{
	return platform_driver_register(&pcf50633_bl_driver);
}

static void __exit mod_exit(void)
{
	platform_driver_unregister(&pcf50633_bl_driver);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_AUTHOR("Ricky Taylor <rickytaylor26@gmail.com>");
MODULE_DESCRIPTION("PCF50633 Backlight Driver");
MODULE_LICENSE("GPL");
