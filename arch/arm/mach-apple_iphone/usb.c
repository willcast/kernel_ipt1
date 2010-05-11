/*
 *  arch/arm/mach-apple_iphone/usb.c
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

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#include <mach/map.h>

#include "core.h"

static struct resource s3c_usb_hsotg_resources[] = {
	[0] = {
		.start	= S3C_PA_USB_HSOTG,
		.end	= S3C_PA_USB_HSOTG + 0x10000 - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 0x13,
		.end	= 0x13,
		.flags	= IORESOURCE_IRQ,
        },
};

struct platform_device s3c_device_usb_hsotg = {
	.name		= "s3c-hsotg",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(s3c_usb_hsotg_resources),
	.resource	= s3c_usb_hsotg_resources,
};

#ifdef CONFIG_USB_ANDROID
char *android_usb_functions[] = {
#ifdef CONFIG_ANDROID_USB_ADB
	"adb",
#endif
#ifdef CONFIG_ANDROID_USB_ACM
	"acm",
#endif
#ifdef CONFIG_ANDROID_USB_MASS_STORAGE
	"usb_mass_storage",
#endif
#ifdef CONFIG_ANDROID_RNDIS
	"rndis",
#endif
};

static struct android_usb_product android_products[] = {
	{
		.product_id	= 0x1234,
		.num_functions	= ARRAY_SIZE(android_usb_functions),
		.functions	= android_usb_functions,
	},
};

struct android_usb_platform_data android_usb_config = {
#ifdef CONFIG_IPHONE_3G
	.product_name		= "iPhone3G",
#endif
#ifdef CONFIG_IPHONE_2G
	.product_name		= "iPhone2G",
#endif

	//.vendor_id			= TODO,
	.product_id			= 0x1234,
	.manufacturer_name	= "Apple",
	.serial_number		= "0123456789", // TODO: Do we need to bother with this?

	.version			= 0x0100,

	.products			= android_products,
	.num_products		= ARRAY_SIZE(android_products),

	.functions			= android_usb_functions,
	.num_functions		= ARRAY_SIZE(android_usb_functions),
};

struct platform_device android_usb = {
	.name			= "android_usb",
	.dev			= {
		.platform_data = &android_usb_config,
	}
};

struct usb_mass_storage_platform_data android_usb_storage_config = {
	.vendor		= "Apple",

#ifdef CONFIG_IPHONE_3G
	.product	= "iPhone3G",
#endif

#ifdef CONFIG_IPHONE_2G
	.product	= "iPhone2G",
#endif

	.release	= 1,

	.nluns		= 1, // TODO: What the hell does this number mean?
};

struct platform_device android_usb_storage = {
	.name	= "usb_mass_storage",
	.dev	= {
		.platform_data = &android_usb_storage_config,
	}
};

struct usb_ether_platform_data android_usb_ether_config = {
	.vendorDescr	= "Apple",
	//.vendorID		= TODO,
};

struct platform_device android_usb_ether = {
	.name			= "rndis",
	.dev			= {
		.platform_data = &android_usb_ether_config,
	}
};
#endif


void init_iphone_usb(void)
{
	platform_device_register(&s3c_device_usb_hsotg);
#ifdef CONFIG_USB_ANDROID
	platform_device_register(&android_usb_ether);
	platform_device_register(&android_usb_storage);
	platform_device_register(&android_usb);
#endif
}
module_init(init_iphone_usb);
