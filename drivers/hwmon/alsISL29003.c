/*
 * drivers/hwmon/alsISL29003.c - driver for Intersil ISL29003 Ambient Light Sensor
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
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/i2c.h>
#include <mach/als.h>
#include <mach/multitouch.h>

#define COMMAND 0x0
#define SENSORLOW 0x4

static struct i2c_client *alsISL29003_i2c;
static int use_channel;

static DEFINE_MUTEX(alsISL29003_mtx);


static void als_writeb(u8 addr, u8 b)
{
        u8 buf[2];
        buf[0] = addr | (1 << 6);
        buf[1] = b;
	i2c_master_send(alsISL29003_i2c, buf, sizeof(buf));
}

static u8 als_readb(u8 addr)
{
        u8 registers[1];
        u8 ret[1];

        registers[0] = addr | (1 << 6);

        ret[0] = 0;

	i2c_master_send(alsISL29003_i2c, registers, 1);
	i2c_master_recv(alsISL29003_i2c, ret, sizeof(ret));

        return ret[0];
}

static u16 als_readw(u8 addr)
{
        struct i2c_msg xfer[2];
        u8 registers;
	u16 ret;

        registers = addr | (1 << 6);
        ret = 0;

//        i2c_master_send(alsISL29003_i2c, (u8 *)&registers, 1);
//        i2c_master_recv(alsISL29003_i2c, (u16 *)&ret, sizeof(ret));

        xfer[0].addr = ALS_ADDR;
        xfer[0].flags = 0;
        xfer[0].len = 1;
        xfer[0].buf = (u8 *)&registers;

        xfer[1].addr = ALS_ADDR;
        xfer[1].flags = I2C_M_RD;
        xfer[1].len = sizeof(ret);
        xfer[1].buf = (u16 *)&ret;

        i2c_transfer(alsISL29003_i2c->adapter, xfer, 2);

        return ret;
}

void als_setchannel(int channel)
{
        use_channel = channel;
        als_writeb(COMMAND, 1 << 7 | 0 << 5 | (channel == 0 ? 0 : 1) << 2);
        udelay(1000);
}

/* Sysfs methods */

static int alsISL29003_read(int *x) {
	*x = als_readw(SENSORLOW);
	return 0;
}

/* Sysfs Files */

static ssize_t alsISL29003_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	int ret, x;

	ret = alsISL29003_read(&x);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", x);
}

static DEVICE_ATTR(light, 0444, alsISL29003_show, NULL);

static struct attribute *alsISL29003_attributes[] = {
	&dev_attr_light.attr,
	NULL,
};

static struct attribute_group alsISL29003_attribute_group = {
	.attrs = alsISL29003_attributes,
};


/* Device model stuff */

static int __devinit alsISL29003_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
	int ret;
	
	alsISL29003_i2c = i2c;

	ret = sysfs_create_group(&alsISL29003_i2c->dev.kobj, &alsISL29003_attribute_group);
	if (ret)
		goto out;

        multitouch_on();

        // disable ADC-core
        als_writeb(COMMAND, 0<<7);
        udelay(1000);
        // powerdown chip
        als_writeb(COMMAND, 1<<6);
        udelay(1000);
        // power up chip
        als_writeb(COMMAND, 0<<6);
        udelay(1000);
        als_setchannel(0);

        if(als_readb(COMMAND) != (1 << 7 | 0 << 5 | (use_channel == 0 ? 0 : 1) << 2))
		goto out_sysfs;

//        gpio_register_interrupt(ALS_INT, 1, 0, 0, als_int, 0);

	printk(KERN_INFO "alsISL29003: device successfully initialized.\n");
	return 0;

out_sysfs:
        sysfs_remove_group(&alsISL29003_i2c->dev.kobj, &alsISL29003_attribute_group);
out:
	printk(KERN_INFO "alsISL29003: error initializing\n");
	return -1;
}

static int __devexit alsISL29003_remove(struct i2c_client *client)
{
	sysfs_remove_group(&alsISL29003_i2c->dev.kobj, &alsISL29003_attribute_group);
        alsISL29003_i2c = NULL;
        return 0;
}

static const struct i2c_device_id alsISL29003_i2c_id[] = {
        { "alsISL29003", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, alsISL29003_i2c_id);

static struct i2c_driver alsISL29003_driver = {
	.driver	= {
		.name = "alsISL29003",
		.owner = THIS_MODULE,
	},
	.probe = alsISL29003_probe,
	.remove = alsISL29003_remove,
	.id_table = alsISL29003_i2c_id,
};


/* Module stuff */

static int __init alsISL29003_init(void)
{
	int ret;

	ret = i2c_add_driver(&alsISL29003_driver);
	if (ret) {
		printk("alsISL29003: Unable to register I2C driver: %d\n", ret);
		goto out;
	}


	printk(KERN_INFO "alsISL29003: driver successfully loaded.\n");
	return 0;

out:
        i2c_del_driver(&alsISL29003_driver);
	printk(KERN_WARNING "alsISL29003: driver init failed (ret=%d)!\n", ret);
	return ret;
}

static void __exit alsISL29003_exit(void)
{
	i2c_del_driver(&alsISL29003_driver);
	printk(KERN_INFO "alsISL29003: driver unloaded.\n");
}

module_init(alsISL29003_init);
module_exit(alsISL29003_exit);

MODULE_AUTHOR("Patrick Wildt <webmaster@patrick-wildt.de>");
MODULE_DESCRIPTION("Intersil ISL29003 Ambient Light Sensor");
MODULE_LICENSE("GPL v2");
