/*
 * drivers/hwmon/iphone_accel.c - driver for iPhone/iPod Accelerometer
 *
 * Copyright (C) 2010 Patrick Wildt <webmaster@patrick-wildt.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <mach/accel.h>

static struct i2c_client *iphone_accel_i2c;

int accel_get_reg(int reg) {
	return i2c_smbus_read_byte_data(iphone_accel_i2c, reg);
}

int accel_write_reg(int reg, int data, int verify) {
	i2c_smbus_write_byte_data(iphone_accel_i2c, reg, data);
}

/* Sysfs methods */

static int iphone_accel_read(int *x, int *y, int *z) {
	*x = (signed char)accel_get_reg(ACCEL_OUTX);
	*y = (signed char)accel_get_reg(ACCEL_OUTY);
	*z = (signed char)accel_get_reg(ACCEL_OUTZ);
	return 0;
}

/* Sysfs Files */

static ssize_t iphone_accel_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, x, y, z;

	ret = iphone_accel_read(&x, &y, &z);
	if (ret)
		return ret;

	return sprintf(buf, "%d:%d:%d\n", x, y, z);
}

static DEVICE_ATTR(acceleration, 0444, iphone_accel_show, NULL);

static struct attribute *iphone_accel_attributes[] = {
	&dev_attr_acceleration.attr,
	NULL,
};

static struct attribute_group iphone_accel_attribute_group = {
	.attrs = iphone_accel_attributes,
};


/* Device model stuff */

static int __devinit iphone_accel_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	int ret;
	int whoami;
	
	iphone_accel_i2c = i2c;

	ret = sysfs_create_group(&iphone_accel_i2c->dev.kobj, &iphone_accel_attribute_group);
	if (ret)
		goto out;

	whoami = accel_get_reg(ACCEL_WHOAMI);
	if(whoami != ACCEL_WHOAMI_VALUE)
		goto out_sysfs;

	accel_write_reg(ACCEL_CTRL_REG2, ACCEL_CTRL_REG2_BOOT);
	accel_write_reg(ACCEL_CTRL_REG1, ACCEL_CTRL_REG1_PD | ACCEL_CTRL_REG1_XEN | ACCEL_CTRL_REG1_YEN | ACCEL_CTRL_REG1_ZEN);

	printk(KERN_INFO "iphone_accel: device successfully initialized.\n");
	return 0;

out_sysfs:
	sysfs_remove_group(&iphone_accel_i2c->dev.kobj, &iphone_accel_attribute_group);
	printk(KERN_INFO "iphone-accel: incorrect whoami value\n");
out:
	printk(KERN_INFO "iphone_accel: error initializing\n");
	return -1;
}

static int __devexit iphone_accel_remove(struct i2c_client *client)
{
	sysfs_remove_group(&iphone_accel_i2c->dev.kobj, &iphone_accel_attribute_group);
	iphone_accel_i2c = NULL;
	return 0;
}

static const struct i2c_device_id iphone_accel_i2c_id[] = {
	{ "iphone-accel", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, iphone_accel_i2c_id);

static struct i2c_driver iphone_accel_driver = {
	.driver	= {
		.name = "iphone-accel",
		.owner = THIS_MODULE,
	},
	.probe = iphone_accel_probe,
	.remove = iphone_accel_remove,
	.id_table = iphone_accel_i2c_id,
};


/* Module stuff */

static int __init iphone_accel_init(void)
{
	int ret;

	ret = i2c_add_driver(&iphone_accel_driver);
	if (ret) {
		printk("iphone_accel: Unable to register I2C driver: %d\n", ret);
		goto out;
	}


	printk(KERN_INFO "iphone_accel: driver successfully loaded.\n");
	return 0;

out:
	i2c_del_driver(&iphone_accel_driver);
	printk(KERN_WARNING "iphone_accel: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static void __exit iphone_accel_exit(void)
{
	i2c_del_driver(&iphone_accel_driver);
	printk(KERN_INFO "iphone_accel: driver unloaded.\n");
}

module_init(iphone_accel_init);
module_exit(iphone_accel_exit);

MODULE_AUTHOR("Patrick Wildt <webmaster@patrick-wildt.de>");
MODULE_DESCRIPTION("iPhone Acceleration Driver");
MODULE_LICENSE("GPL v2");
