// SPDX-License-Identifier: GPL-2.0
/*
 * ST54SPI GPIO driver
 * Copyright (C) 2021 ST Microelectronics S.A.
 *  * This program is free software; you can redistribute it and/or modify
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
 *
 **********************************************************************************/

#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/types.h>

#define ST54SPI_GPIO__MAGIC  0xEB
#define ST54SPI_GET_GPIO	_IOW(ST54SPI_GPIO__MAGIC, 0x01, unsigned int)
#define ST54SPI_SET_GPIO	_IOW(ST54SPI_GPIO__MAGIC, 0x02, unsigned int)

struct st54spi_gpio_device {
	dev_t st54spi_gpio_dev_t;
	struct cdev c_dev;
	struct class *class;
	struct device *device;
	struct spi_device *spi_dev;
	/* GPIO for st54spi Reset pin (output) */
	int gpiod_reset;
};

static ssize_t st54spi_gpio_dev_read(struct file *filp,
						char __user *buf, size_t count, loff_t *off)
{
	uint8_t gpio_state = 0;
	struct st54spi_gpio_device *st54spi_gpio_dev = filp->private_data;

	//reading GPIO value
	gpio_state = gpio_get_value(st54spi_gpio_dev->gpiod_reset);

	//write to user
	count = 1;
	if (copy_to_user(buf, &gpio_state, count) > 0)
		pr_err("ERROR: Not all the bytes have been copied to user\n");

	return 0;
}

static ssize_t st54spi_gpio_dev_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *offset)
{
	uint8_t rec_buf[10] = {0};

	struct st54spi_gpio_device *st54spi_gpio_dev = filp->private_data;

	if (copy_from_user(rec_buf, buf, count) > 0)
		pr_err("ERROR: Not all the bytes have been copied from user\n");

	if (rec_buf[0] == 1) {
		//set the GPIO value to HIGH
		gpio_set_value(st54spi_gpio_dev->gpiod_reset, 1);
	} else if (rec_buf[0] == 0) {
		//set the GPIO value to LOW
		gpio_set_value(st54spi_gpio_dev->gpiod_reset, 0);
	} else {
		pr_err("Unknown command : Please provide either 1 or 0\n");
	}

	return count;
}


/** @brief   IOCTL function  to be used to set or get data from upper layer.
 *
 *  @param   pfile  fil node for opened device.
 *  @cmd     IOCTL type from upper layer.
 *  @arg     IOCTL arg from upper layer.
 *
 *  @return 0 on success, error code for failures.
 */
long st54spi_gpio_dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct st54spi_gpio_device *st54spi_gpio_dev = pfile->private_data;

	if (!st54spi_gpio_dev) {
		pr_err("%s ENODEV! st54spi_gpio_dev is NULL\n", __func__);
		return -ENODEV;
}

	switch (cmd) {
	case ST54SPI_GET_GPIO:
		ret = gpio_get_value(st54spi_gpio_dev->gpiod_reset);
		break;
	case ST54SPI_SET_GPIO:
		if ((arg == 0) || (arg == 1)) {
			gpio_set_value(st54spi_gpio_dev->gpiod_reset, arg);
	} else {
		pr_err("%s bad arg %lu\n", __func__, arg);
		ret = -ENOIOCTLCMD;
	}
		break;
	default:
		pr_err("%s Unsupported ioctl cmd 0x%x, arg %lu\n",
						__func__, cmd, arg);
		ret = -ENOIOCTLCMD;
	}
	return ret;
}

/* This function will be called when we open the Device file*/

static int st54spi_gpio_dev_open(struct inode *inode, struct file *pfile)
{
	struct st54spi_gpio_device *st54spi_gpio_dev = container_of(inode->i_cdev,
					struct st54spi_gpio_device, c_dev);

	pr_info("%s : Device File Opened\n", __func__);
	if (!st54spi_gpio_dev) {
		pr_err("%s ENODEV NULL\n", __func__);
		return -ENODEV;
	}

	pfile->private_data = st54spi_gpio_dev;
	return 0;
}

/* This function will be called when we close the Device file*/

static int st54spi_gpio_dev_release(struct inode *inode, struct file *pfile)
{
	pr_info("%s: Device File Closed\n", __func__);
	pfile->private_data = NULL;
	return 0;
}


static const struct file_operations st54spi_gpio_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st54spi_gpio_dev_read,
	.write = st54spi_gpio_dev_write,
	.open = st54spi_gpio_dev_open,
	.release = st54spi_gpio_dev_release,
	.unlocked_ioctl = st54spi_gpio_dev_ioctl,
};

static int st54spi_gpio_probe(struct platform_device *pdev)
{
	int rc;
	struct st54spi_gpio_device *st54spi_gpio_dev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (np == NULL)
		pr_err("%s : struct np is null\n", __func__);

	pr_info("%s : enter", __func__);
	st54spi_gpio_dev = devm_kzalloc(dev, sizeof(*st54spi_gpio_dev), GFP_KERNEL);
	if (!st54spi_gpio_dev)
		return -ENOMEM;

/* Create device node */
	rc = alloc_chrdev_region(&st54spi_gpio_dev->st54spi_gpio_dev_t, 0, 1, "st54spi_gpio");
	if (rc < 0) {
		pr_err("alloc_chrdev_region() failed\n");
		return rc;
	}

	st54spi_gpio_dev->class = class_create(THIS_MODULE, "st54spi_gpio");
	if (IS_ERR(st54spi_gpio_dev->class)) {
		rc = PTR_ERR(st54spi_gpio_dev->class);
		pr_err("Error creating st54spi_gpio_dev->class: %d\n", rc);
		goto fail_class_create;
	}

	cdev_init(&st54spi_gpio_dev->c_dev, &st54spi_gpio_dev_fops);
	rc = cdev_add(&st54spi_gpio_dev->c_dev, st54spi_gpio_dev->st54spi_gpio_dev_t, 1);
	if (rc) {
		pr_err("Error calling cdev_add: %d\n", rc);
		goto fail_cdev_add;
	}

	st54spi_gpio_dev->device = device_create(st54spi_gpio_dev->class,
							NULL, st54spi_gpio_dev->st54spi_gpio_dev_t, st54spi_gpio_dev, "st54spi_gpio");
	if (IS_ERR(st54spi_gpio_dev->device)) {
		rc = PTR_ERR(st54spi_gpio_dev->device);
		pr_err("device_create failed: %d\n", rc);
		goto fail_device_create;
	}

	/* Setup gpio-power_nreset */
	st54spi_gpio_dev->gpiod_reset = of_get_named_gpio(np, "gpio-power_nreset", 0);
	if (!gpio_is_valid(st54spi_gpio_dev->gpiod_reset)) {
		pr_err("%s : Unable to request gpio-power_nreset\n", __func__);
		rc = -EFAULT;
		goto fail_gpiod_get;
	}

	rc = gpio_request(st54spi_gpio_dev->gpiod_reset, "gpio-power_nreset");
	if (rc < 0)
		pr_err("request gpio failed, cannot wake up controller: %d\n", __func__, rc);

	rc = gpio_direction_output(st54spi_gpio_dev->gpiod_reset, 0);
	if (rc < 0)
		pr_err("gpio cannot set the output %d\n", __func__, rc);

	platform_set_drvdata(pdev, st54spi_gpio_dev);

	return 0;

fail_gpiod_get:
	device_destroy(st54spi_gpio_dev->class, st54spi_gpio_dev->st54spi_gpio_dev_t);
fail_device_create:
	cdev_del(&st54spi_gpio_dev->c_dev);
fail_cdev_add:
	class_destroy(st54spi_gpio_dev->class);
fail_class_create:
	unregister_chrdev_region(st54spi_gpio_dev->st54spi_gpio_dev_t, 1);
	devm_kfree(dev, st54spi_gpio_dev);

	return rc;
}

static int st54spi_gpio_remove(struct platform_device *pdev)
{
	struct st54spi_gpio_device *st54spi_gpio_dev;
	struct device *dev = &pdev->dev;

	pr_info("%s : enter", __func__);
	st54spi_gpio_dev = platform_get_drvdata(pdev);
	device_destroy(st54spi_gpio_dev->class, st54spi_gpio_dev->st54spi_gpio_dev_t);
	unregister_chrdev_region(st54spi_gpio_dev->st54spi_gpio_dev_t, 1);
	class_destroy(st54spi_gpio_dev->class);
	cdev_del(&st54spi_gpio_dev->c_dev);
	devm_kfree(dev, st54spi_gpio_dev);
	dev_set_drvdata(dev, NULL);

	return 0;
}

static const struct of_device_id st54spi_gpio_of_match[] = {
	{ .compatible = "st,st54spi_gpio", },
	{},
};
MODULE_DEVICE_TABLE(of, st54spi_gpio_of_match);

static struct platform_driver st54spi_gpio_driver = {
	.driver = {
		   .name = "st54spi_gpio",
		   .owner = THIS_MODULE,
		   .of_match_table = st54spi_gpio_of_match,
		   },
	.probe = st54spi_gpio_probe,
	.remove = st54spi_gpio_remove,
};

/* module load/unload record keeping */
static int __init st54spi_gpio_dev_init(void)
{
	pr_err("%s : Loading st54spi gpio_driver 1.6\n", __func__);
	return platform_driver_register(&st54spi_gpio_driver);
}

module_init(st54spi_gpio_dev_init);

static void __exit st54spi_gpio_dev_exit(void)
{
	pr_info("%s : Unloading st54spi gpio_driver\n", __func__);
	platform_driver_unregister(&st54spi_gpio_driver);
}

module_exit(st54spi_gpio_dev_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("ST54SPI GPIO driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("st54spi_gpio");
