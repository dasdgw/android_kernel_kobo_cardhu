/*
 *  Copyright (C) 2010, Lars-Peter Clausen <lars@metafoo.de>
 *  Driver for chargers which report their online status through a GPIO pin
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under  the terms of the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the License, or (at your
 *  option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <linux/power/gpio-charger.h>

//add for sa pcb
extern int pcb_ver;
extern int usb_running;
extern int capacity_is_low;
int ac_is_online = 0;
extern bool wait_usb_verify;
bool ac_irq_wait = false ;
extern bool system_suspend;
extern bool otg_irq;
extern char androidboot_mode[16];
bool IsChargerMode = false ;
extern void report_power_event(void);

struct gpio_charger {
	const struct gpio_charger_platform_data *pdata;
	unsigned int irq;

	struct power_supply ac_charger;
	struct power_supply usb_charger;
};

static irqreturn_t gpio_charger_irq(int irq, void *devid)
{
	struct gpio_charger *gpio_charger = devid; 	
	int wait_times;

	wait_times = 0;
	ac_irq_wait = true;
	power_supply_changed(&gpio_charger->usb_charger);
	power_supply_changed(&gpio_charger->ac_charger);

	
	if(IsChargerMode == true)
	{
		return IRQ_HANDLED;
	}
	
	while(!system_suspend)
	{
		msleep(1);

		if(wait_times>=30)
			break;

		wait_times++;
	}
	
	if(system_suspend)
	{
		if(!otg_irq)
		{
			pr_info("%s:report_power_event\n", __func__);
			report_power_event();
		}
	}

	return IRQ_HANDLED;
}

static inline struct gpio_charger *psy_to_gpio_charger(struct power_supply *psy)
{
	return container_of(psy, struct gpio_charger, ac_charger);
}

static int ac_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct gpio_charger *gpio_charger = psy_to_gpio_charger(psy);
	const struct gpio_charger_platform_data *pdata = gpio_charger->pdata;
	int ac_ok_is_high;
	int otg_enable;
	int charger_stat;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		if(ac_irq_wait == true)
		{
			msleep(300);
			ac_irq_wait = false;
		}
		
		if(wait_usb_verify||usb_running)
		{
			val->intval = 0;
			ac_is_online = val->intval;
			return 0;
		}
		
		
		ac_ok_is_high = gpio_get_value_cansleep(pdata->gpio);
		val->intval = ac_ok_is_high;

		//add for sa pcb
		if(pcb_ver == 0)
		{
			if((ac_ok_is_high)||usb_running)
				val->intval = 0;
			else
				val->intval = 1;
		}
		else
		{
			otg_enable = gpio_get_value(pdata->otg_enable_pin);

			if(otg_enable||usb_running)
				val->intval = 0;
			else
			{
				if((pcb_ver == 1)&&capacity_is_low)
				{
					charger_stat = gpio_get_value(pdata->charger_stat_gpio);
					if(charger_stat)
						val->intval = 0;
					else
						val->intval = 1;
				}
				else
				{
					if(ac_ok_is_high)
					{
						val->intval = 1;
					}
					else
					{
						val->intval = 0;
					}
				}
			}
		}

		ac_is_online = val->intval;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int usb_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		
		if(usb_running)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property gpio_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int __devinit gpio_charger_probe(struct platform_device *pdev)
{
	const struct gpio_charger_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_charger *gpio_charger;
	struct power_supply *ac_charger;
	struct power_supply *usb_charger;
	int ret;
	int irq;

	if (!pdata) {
		dev_err(&pdev->dev, "No platform data\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(pdata->gpio)) {
		dev_err(&pdev->dev, "Invalid gpio pin\n");
		return -EINVAL;
	}

	gpio_charger = kzalloc(sizeof(*gpio_charger), GFP_KERNEL);
	if (!gpio_charger) {
		dev_err(&pdev->dev, "Failed to alloc driver structure\n");
		return -ENOMEM;
	}

	ac_charger = &gpio_charger->ac_charger;

	ac_charger->name = pdata->name ? pdata->name : "ac_charger";
	ac_charger->type = pdata->type;
	ac_charger->properties = gpio_charger_properties;
	ac_charger->num_properties = ARRAY_SIZE(gpio_charger_properties);
	ac_charger->get_property = ac_charger_get_property;
	ac_charger->supplied_to = pdata->supplied_to;
	ac_charger->num_supplicants = pdata->num_supplicants;

	usb_charger = &gpio_charger->usb_charger;

	usb_charger->name = "usb_charger";
	usb_charger->type = POWER_SUPPLY_TYPE_USB;
	usb_charger->properties = gpio_charger_properties;
	usb_charger->num_properties = ARRAY_SIZE(gpio_charger_properties);
	usb_charger->get_property = usb_charger_get_property;
	usb_charger->supplied_to = pdata->supplied_to;
	usb_charger->num_supplicants = pdata->num_supplicants;

	ret = gpio_request(pdata->gpio, dev_name(&pdev->dev));
	if (ret) {
		dev_err(&pdev->dev, "Failed to request gpio pin: %d\n", ret);
		goto err_free;
	}
	ret = gpio_direction_input(pdata->gpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to set gpio to input: %d\n", ret);
		goto err_gpio_free;
	}

	gpio_charger->pdata = pdata;

	ret = power_supply_register(&pdev->dev, ac_charger);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register power supply: %d\n",
			ret);
		goto err_gpio_free;
	}

       ret = power_supply_register(&pdev->dev, usb_charger);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register power supply: %d\n",
			ret);
		goto err_gpio_free;
	}

	
	if(strcmp(androidboot_mode, "charger")==0)
	{
		IsChargerMode = true;
	}
	else
	{
		IsChargerMode = false;
	}
	
	
	irq = gpio_to_irq(pdata->gpio);
	if (irq > 0) {
		ret = request_any_context_irq(irq, gpio_charger_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&pdev->dev), gpio_charger);
		if (ret < 0)
			dev_warn(&pdev->dev, "Failed to request irq: %d\n", ret);
		else
			gpio_charger->irq = irq;
	}

	device_init_wakeup(&pdev->dev, 1);
	enable_irq_wake(irq);
	
	platform_set_drvdata(pdev, gpio_charger);

	return 0;

err_gpio_free:
	gpio_free(pdata->gpio);
err_free:
	kfree(gpio_charger);
	return ret;
}

static int __devexit gpio_charger_remove(struct platform_device *pdev)
{
	struct gpio_charger *gpio_charger = platform_get_drvdata(pdev);

	if (gpio_charger->irq)
		free_irq(gpio_charger->irq, &gpio_charger->ac_charger);

	power_supply_unregister(&gpio_charger->ac_charger);
	power_supply_unregister(&gpio_charger->usb_charger);

	gpio_free(gpio_charger->pdata->gpio);

	platform_set_drvdata(pdev, NULL);
	kfree(gpio_charger);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_charger_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct gpio_charger *gpio_charger = platform_get_drvdata(pdev);

	power_supply_changed(&gpio_charger->ac_charger);
	power_supply_changed(&gpio_charger->usb_charger);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(gpio_charger_pm_ops, NULL, gpio_charger_resume);

static struct platform_driver gpio_charger_driver = {
	.probe = gpio_charger_probe,
	.remove = __devexit_p(gpio_charger_remove),
	.driver = {
		.name = "gpio-charger",
		.owner = THIS_MODULE,
		.pm = &gpio_charger_pm_ops,
	},
};

static int __init gpio_charger_init(void)
{
	return platform_driver_register(&gpio_charger_driver);
}
module_init(gpio_charger_init);

static void __exit gpio_charger_exit(void)
{
	platform_driver_unregister(&gpio_charger_driver);
}
module_exit(gpio_charger_exit);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for chargers which report their online status through a GPIO");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio-charger");
