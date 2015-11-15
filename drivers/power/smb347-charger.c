/*
 * drivers/power/smb347-charger.c
 *
 * Battery charger driver for smb347 from summit microelectronics
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb347-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>

//#define SMB_DEBUG                       1
#if SMB_DEBUG
#define SMB_INFO(format, arg...)        \
        printk(KERN_INFO "smb347_charger: [%s] " format , __FUNCTION__ , ## arg)
#else
#define SMB_INFO(format, arg...)
#endif

/* Configuration registers */
#define SMB347_CHARGE           0x00
#define SMB347_CHRG_CRNTS       0x01
#define SMB347_VRS_FUNC         0x02
#define SMB347_FLOAT_VLTG       0x03
#define SMB347_CHRG_CTRL        0x04
#define SMB347_STAT_TIME_CTRL   0x05
#define SMB347_PIN_CTRL         0x06
#define SMB347_THERM_CTRL       0x07
#define SMB347_SYSOK_USB3       0x08
#define SMB347_CTRL_REG         0x09

#define SMB347_OTG_TLIM_REG     0x0A
#define SMB347_HRD_SFT_TEMP     0x0B
#define SMB347_FAULT_INTR       0x0C
#define SMB347_STS_INTR_1       0x0D
#define SMB347_I2C_ADDR 0x0E
#define SMB347_IN_CLTG_DET      0x10
#define SMB347_STS_INTR_2       0x11

/* Command registers */
#define SMB347_CMD_REG          0x30
#define SMB347_CMD_REG_B        0x31
#define SMB347_CMD_REG_C        0x33

/* Interrupt Status registers */
#define SMB347_INTR_STS_A       0x35
#define SMB347_INTR_STS_B       0x36
#define SMB347_INTR_STS_C       0x37
#define SMB347_INTR_STS_D       0x38
#define SMB347_INTR_STS_E       0x39
#define SMB347_INTR_STS_F       0x3A

/* Status registers */
#define SMB347_STS_REG_A        0x3B
#define SMB347_STS_REG_B        0x3C
#define SMB347_STS_REG_C        0x3D
#define SMB347_STS_REG_D        0x3E
#define SMB347_STS_REG_E        0x3F

#define SMB347_ENABLE_WRITE	1
#define SMB347_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define THERM_CTRL		0x10
#define BATTERY_MISSING		0x10
#define CHARGING		0x06
#define DEDICATED_CHARGER	0x02
#define CHRG_DOWNSTRM_PORT	0x01
#define ENABLE_CHARGE		0x02
#define ENABLE_USBIN_SUSPEND		0x04

#define high		0x01
#define low		0x00

//add for sa pcb
extern int pcb_ver;

static struct smb347_charger *charger;
static unsigned long stop_charging_flag = 0;
static int smb347_configure_charger(struct i2c_client *client, int value);
static int smb347_suspend(struct i2c_client *client, int value);
static int smb347_read(struct i2c_client *client, int reg);

static u8 regs[] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
       0x0B, 0x0C, 0x0D, 0x0E, 0x30, 0x31, 0x33, 0x35, 0x36, 0x37, 0x38, 
	0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
};

static void show_smb347_regs(void)
{
	u8 Counti;
	struct i2c_client *client = charger->client;

	for(Counti=0;Counti<ARRAY_SIZE(regs);)
	{
		SMB_INFO("Reg 0x%x = 0x%x\r\n",regs[Counti],smb347_read(client, regs[Counti]));
	
		Counti++;
	}	
}
static int stop_charging_control(void)
{
	struct i2c_client *client = charger->client;
	int ret = 0;
	int val;
	
	//msleep(200);

	if (stop_charging_flag == 1) {

		SMB_INFO("enable charger suspend\n");
		
		//enable charger suspend
		ret = smb347_suspend(client, 1);

		if (ret < 0) {
				dev_err(&client->dev, "%s() error in"
					"configuring charger..\n", __func__);
				goto error;
		}
		
	}
	else if (stop_charging_flag == 0) {

		SMB_INFO("disable charger suspend\n");
		
		//disable charger suspend
		ret = smb347_suspend(client, 0);

		if (ret < 0) {
				dev_err(&client->dev, "%s() error in"
					"configuring charger..\n", __func__);
				goto error;	
		}
	}

	#if SMB_DEBUG
	show_smb347_regs();
	#endif
	
	return ret;

error:
	SMB_INFO("stop_charging_control fail, err = %d\n", ret);

	return ret;	
}

static int smb347_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb347_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == SMB347_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb347_update_reg(client, SMB347_CMD_REG,
						ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, SMB347_CMD_REG);
			return ret;
		}
	} else {
		ret = smb347_read(client, SMB347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb347_write(client, SMB347_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

static int smb347_configure_otg(struct i2c_client *client, int enable)
{
	int ret = 0;
	struct smb347_charger_platform_data *pdata;
	
	pdata = client->dev.platform_data;

	if(!pdata) {
		ret = -ENXIO;
		goto error;
	}

	/*Enable volatile writes to registers*/
	ret = smb347_volatile_writes(client, SMB347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring otg..\n",
								__func__);
		goto error;
	}

	if (enable) {

		SMB_INFO("Enable OTG\n");

		if(pcb_ver != 0)
		{
			SMB_INFO("set gpio high to enable otg\n");
			ret = gpio_direction_output(pdata->otg_enable_pin,high);

			if (ret) {
				dev_err(&client->dev, "Failed to set gpio as output high: %d\n", ret);
				goto error;
			}
		}

		//add for sa pcb
		if(pcb_ver == 0)
		{
			/* Enable OTG*/
			ret = smb347_update_reg(client, SMB347_CMD_REG, 0x10);
			if (ret < 0) {
				dev_err(&client->dev, "%s: Failed in writing register"
					"0x%02x\n", __func__, SMB347_CMD_REG);
				goto error;
			}			

		}
			/*Change INOK Polarity to Active High*/
			ret = smb347_read(client, SMB347_SYSOK_USB3);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}

			ret = smb347_write(client, SMB347_SYSOK_USB3,
							(ret | (1<<0)));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}		

		SMB_INFO("Change OTG Current Limit at USBIN from 250mA to 500mA\n");
		/*Change OTG Current Limit at USBIN from 250mA to 500mA*/
		ret = smb347_read(client, SMB347_OTG_TLIM_REG);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}

		ret = smb347_write(client, SMB347_OTG_TLIM_REG,
						((ret & (~(1<<2))) | (1<<3)));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}					
		
	} else {

		SMB_INFO("Disable OTG\n");

		if(pcb_ver != 0)
		{
			SMB_INFO("set gpio low to disable otg\n");
			ret = gpio_direction_output(pdata->otg_enable_pin,low);

			if (ret) {
			dev_err(&client->dev, "Failed to set gpio as output low: %d\n", ret);
				goto error;
			}
		}

		//add for sa pcb
		if(pcb_ver == 0)
		{		
			/* Disable OTG */			
			ret = smb347_read(client, SMB347_CMD_REG);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				goto error;
			}	

			ret = smb347_write(client, SMB347_CMD_REG,
							(ret & ~(1<<4)));

			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}	
		}	
		
			/*Change INOK Polarity to Active Low*/
			ret = smb347_read(client, SMB347_SYSOK_USB3);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}

			ret = smb347_write(client, SMB347_SYSOK_USB3,
							(ret & ~(1<<0)));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}		

		SMB_INFO("Change OTG Current Limit at USBIN from 500mA to 250mA\n");
		/*Change OTG Current Limit at USBIN from 500mA to 250mA*/
		ret = smb347_read(client, SMB347_OTG_TLIM_REG);
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}

		ret = smb347_write(client, SMB347_OTG_TLIM_REG,
						((ret & (~(1<<3))) | (1<<2)));
			if (ret < 0) {
				dev_err(&client->dev, "%s: err %d\n",
					__func__, ret);
				goto error;
			}			
		
	}

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s error in configuring OTG..\n",

								__func__);

	#if SMB_DEBUG
	show_smb347_regs();
	#endif
		
error:
	return ret;
}

static int smb347_suspend(struct i2c_client *client, int value)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		 /* Enable suspend */
		ret = smb347_update_reg(client, SMB347_CMD_REG, ENABLE_USBIN_SUSPEND);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x\n", __func__, SMB347_CMD_REG);
			goto error;
		}

	} else {
		ret = smb347_read(client, SMB347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb347_write(client, SMB347_CMD_REG, (ret & (~(1<<2))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
error:
	return ret;
}

static int smb347_configure_charger(struct i2c_client *client, int value)
{
	int ret = 0;

	/* Enable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

	if (value) {
		 /* Enable charging */
		ret = smb347_update_reg(client, SMB347_CMD_REG, ENABLE_CHARGE);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing register"
					"0x%02x\n", __func__, SMB347_CMD_REG);
			goto error;
		}

	} else {
		ret = smb347_read(client, SMB347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}

		ret = smb347_write(client, SMB347_CMD_REG, (ret & (~(1<<1))));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			goto error;
		}
	}
	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
error:
	return ret;
}

int update_charger_status(void)
{
	struct i2c_client *client;
	int val;

	if (!charger)
		return -ENODEV;
	else
		client = charger->client;

	val =  smb347_read(client, SMB347_STS_REG_D);
	if (val < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
			"0x%02x\n", __func__, SMB347_STS_REG_D);
		goto val_error;
	} else if (val != 0) {
		if (val & DEDICATED_CHARGER)
			charger->chrg_type = AC;
		else
			charger->chrg_type = USB;

		charger->state = progress;
	} else {
		charger->state = stopped;
	}

	if (charger->charger_cb)
		charger->charger_cb(charger->state, charger->chrg_type,
						charger->charger_cb_data);
	return 0;
val_error:
	return val;
}
EXPORT_SYMBOL_GPL(update_charger_status);

int register_callback(charging_callback_t cb, void *args)
{
	struct smb347_charger *charger_data = charger;
	if (!charger_data)
		return -ENODEV;

	charger_data->charger_cb = cb;
	charger_data->charger_cb_data = args;
	return 0;
}
EXPORT_SYMBOL_GPL(register_callback);

int smb347_battery_online(void)
{
	int val;
	struct i2c_client *client = charger->client;

	val = smb347_read(charger->client, SMB347_INTR_STS_B);
	if (val < 0) {
		dev_err(&client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, SMB347_INTR_STS_B);
		return val;
	}
	if (val & BATTERY_MISSING)
		return 0;
	else
		return 1;
}

static int smb347_enable_otg(struct regulator_dev *otg_rdev)
{
	struct i2c_client *client = charger->client;
	int ret;

	/* ENABLE OTG */
	ret = smb347_configure_otg(client, 1);
	if (ret < 0)
		goto error;

	charger->is_otg_enabled = 1;
	return 0;
error:
	dev_err(&client->dev, "%s() error in enabling"
			"otg..\n", __func__);
	return ret;
}

static int smb347_disable_otg(struct regulator_dev *otg_rdev)
{
	struct i2c_client *client = charger->client;
	int ret;

	/* Disable OTG */
	ret = smb347_configure_otg(client, 0);
	if (ret < 0)
		goto error;

	charger->is_otg_enabled = 0;
	return 0;
error:
	dev_err(&client->dev, "%s() error in disabling"
			"otg..\n", __func__);
	return ret;
}

static int smb347_is_otg_enabled(struct regulator_dev *otg_rdev)
{
	return charger->is_otg_enabled;
}

static int smb347_enable_charging(struct regulator_dev *rdev,
					int min_uA, int max_uA)
{
	struct i2c_client *client = charger->client;
	int ret;
		
	return 0;
		
	if (!max_uA) {
		/* Wait for SMB347 to debounce and get reset to POR when cable is unpluged */
		msleep(50);

		ret =  smb347_read(client, SMB347_STS_REG_C);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in reading register"
				"0x%02x\n", __func__, SMB347_STS_REG_C);
			return ret;
		}

		if (ret & CHARGING)
			return 0;

		charger->state = stopped;
		charger->chrg_type = NONE;
	} else {
		/* Wait for SMB347 to reload OTP setting and detect type*/
		msleep(500);

		ret =  smb347_read(client, SMB347_STS_REG_D);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in reading register"
					"0x%02x\n", __func__, SMB347_STS_REG_D);
			return ret;
		} else if (ret != 0) {
			if (ret & DEDICATED_CHARGER)
				charger->chrg_type = AC;
			else
				charger->chrg_type = USB;

			/* configure charger */
			ret = smb347_configure_charger(client, 1);
			if (ret < 0) {
				dev_err(&client->dev, "%s() error in"
					"configuring charger..\n", __func__);
				return ret;
			}
			charger->state = progress;
		}
	}
	if (charger->charger_cb)
		charger->charger_cb(charger->state, charger->chrg_type,
						charger->charger_cb_data);
	return 0;
}

static struct regulator_ops smb347_tegra_regulator_ops = {
	.set_current_limit = smb347_enable_charging,
};

static struct regulator_ops smb347_tegra_otg_regulator_ops = {
	.enable = smb347_enable_otg,
	.disable = smb347_disable_otg,
	.is_enabled = smb347_is_otg_enabled,
};

#if defined(CONFIG_DEBUG_FS)
static struct dentry *smb347_dentry_regs;

static int smb347_dump_regs(struct i2c_client *client, u8 *addrs, int num_addrs,
		char *buf, ssize_t *len)
{
	ssize_t count = *len;
	int ret = 0;
	int i;

	if (count >= PAGE_SIZE - 1)
		return -ERANGE;

	for (i = 0; i < num_addrs; i++) {
		count += sprintf(buf + count, "0x%02x: ", addrs[i]);
		if (count >= PAGE_SIZE - 1)
			return -ERANGE;

		ret = smb347_read(client, addrs[i]);
		if (ret < 0)
			count += sprintf(buf + count, "<read fail: %d\n", ret);
		else
			count += sprintf(buf + count, "0x%02x\n", ret);

		if (count >= PAGE_SIZE - 1)
			return -ERANGE;
	}
	*len = count;

	return 0;
}

static ssize_t smb347_debugfs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t smb347_debugfs_read(struct file *file, char __user *user_buf,
				size_t count, loff_t *ppos)
{
	ssize_t ret;
	struct i2c_client *client = file->private_data;
	char *buf;
	size_t len = 0;

	buf = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	len += sprintf(buf + len, "SMB347 Registers\n");
	smb347_dump_regs(client, regs, ARRAY_SIZE(regs), buf, &len);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
	kfree(buf);
	return ret;
}

static const struct file_operations smb347_debugfs_fops = {
	.open = smb347_debugfs_open,
	.read = smb347_debugfs_read,
};

static void smb347_debugfs_init(struct i2c_client *client)
{
	smb347_dentry_regs = debugfs_create_file(client->name,
						0444, 0, client,
						&smb347_debugfs_fops);
}

static void smb347_debugfs_exit(struct i2c_client *client)
{
	debugfs_remove(smb347_dentry_regs);
}
#else
static void smb347_debugfs_init(struct i2c_client *client){}
static void smb347_debugfs_exit(struct i2c_client *client){}
#endif /* CONFIG_DEBUG_FS */

static ssize_t charging_stop_flag_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value = 0;
	int ret;
	
	ret = strict_strtoul(buf, 0, &value);
	if (ret)
		printk(KERN_INFO "%s is not in hex or decimal form.\n", buf);
	else
		stop_charging_flag = value;

	stop_charging_control();
	printk(KERN_INFO "%s, value=0x%x, stop_charging_flag=0x%x \n", __func__, value, stop_charging_flag);
	
	return strnlen(buf, count);
}

static ssize_t charging_stop_flag_get(struct device *dev, struct kobj_attribute *attr, char *buf)
{
	printk("[%s] stop_charging_flag=%d \n", __func__, stop_charging_flag);	
	return sprintf(buf, "%d\n", stop_charging_flag);
}

static DEVICE_ATTR(charging_stop, S_IRWXO | S_IRWXG | S_IRWXU, charging_stop_flag_get, charging_stop_flag_store);

static int __devinit smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb347_charger_platform_data *pdata;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->client = client;
	charger->dev = &client->dev;
	pdata = client->dev.platform_data;
	if(!pdata) {
		ret = -ENXIO;
		goto error;
	}
	
	i2c_set_clientdata(client, charger);

	/* Check battery presence */
	if (!smb347_battery_online()) {
		dev_err(&client->dev, "%s() No Battery present, exiting..\n",
					__func__);
		ret = -ENODEV;
		goto regulator_error;
	}


	//modify Termination Current to 50mA
	/*Enable volatile writes to registers*/
	ret = smb347_volatile_writes(client, SMB347_ENABLE_WRITE);
	ret = smb347_write(client, SMB347_CHARGE, 0x91);
	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, SMB347_DISABLE_WRITE);

	charger->is_otg_enabled = 0;

	charger->reg_desc.name  = "vbus_charger";
	charger->reg_desc.ops   = &smb347_tegra_regulator_ops;
	charger->reg_desc.type  = REGULATOR_CURRENT;
	charger->reg_desc.id    = pdata->regulator_id;
	charger->reg_desc.owner = THIS_MODULE;

	charger->reg_init_data.supply_regulator         = NULL;
	charger->reg_init_data.num_consumer_supplies    =
				       pdata->num_consumer_supplies;
	charger->reg_init_data.regulator_init           = NULL;
	charger->reg_init_data.consumer_supplies        =
				       pdata->consumer_supplies;
	charger->reg_init_data.driver_data              = charger;
	charger->reg_init_data.constraints.name         = "vbus_charger";
	charger->reg_init_data.constraints.min_uA       = 0;
	charger->reg_init_data.constraints.max_uA       =
					pdata->max_charge_current_mA * 1000;

	charger->reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;

	charger->reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_CURRENT;

	charger->rdev = regulator_register(&charger->reg_desc, charger->dev,
					&charger->reg_init_data, charger);

	smb347_debugfs_init(client);

	if (IS_ERR(charger->rdev)) {
		dev_err(&client->dev, "failed to register %s\n",
				charger->reg_desc.name);
		ret = PTR_ERR(charger->rdev);
		goto regulator_error;
	}

	charger->otg_reg_desc.name  = "vbus_otg";
	charger->otg_reg_desc.ops   = &smb347_tegra_otg_regulator_ops;
	charger->otg_reg_desc.type  = REGULATOR_CURRENT;
	charger->otg_reg_desc.id    = pdata->otg_regulator_id;
	charger->otg_reg_desc.type  = REGULATOR_CURRENT;
	charger->otg_reg_desc.owner = THIS_MODULE;

	charger->otg_reg_init_data.supply_regulator         = NULL;
	charger->otg_reg_init_data.num_consumer_supplies    =
				       pdata->num_otg_consumer_supplies;
	charger->otg_reg_init_data.regulator_init           = NULL;
	charger->otg_reg_init_data.consumer_supplies        =
				       pdata->otg_consumer_supplies;
	charger->otg_reg_init_data.driver_data              = charger;
	charger->otg_reg_init_data.constraints.name         = "vbus_otg";
	charger->otg_reg_init_data.constraints.min_uA       = 0;
	charger->otg_reg_init_data.constraints.max_uA       = 500000;

	charger->otg_reg_init_data.constraints.valid_modes_mask =
					REGULATOR_MODE_NORMAL |
					REGULATOR_MODE_STANDBY;

	charger->otg_reg_init_data.constraints.valid_ops_mask =
					REGULATOR_CHANGE_MODE |
					REGULATOR_CHANGE_STATUS |
					REGULATOR_CHANGE_CURRENT;

	charger->otg_rdev = regulator_register(&charger->otg_reg_desc, charger->dev,
					&charger->otg_reg_init_data, charger);
	if (IS_ERR(charger->otg_rdev)) {
		dev_err(&client->dev, "failed to register %s\n",
				charger->otg_reg_desc.name);
		ret = PTR_ERR(charger->otg_rdev);
		goto otg_regulator_error;
	}
	
	SMB_INFO("Creating charging stop node\n");
	/*
	Creating charging stop node.
	/sys/bus/i2c/devices/i2c-4/4-006a/charging_stop
	*/
	if (device_create_file(&client->dev, &dev_attr_charging_stop)) {
		dev_err(&client->dev, "failed to create sysfs attributes");	
	}

	#if SMB_DEBUG
	show_smb347_regs();
	#endif

	//add for sa pcb
	if(pcb_ver != 0)
	{
		ret = gpio_request(pdata->otg_enable_pin, "OTG_ENABLE_PIN");
	
		if (ret) {
			dev_err(&client->dev, "Failed to request gpio: %d\n", ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->otg_enable_pin,low);

		if (ret) {
			dev_err(&client->dev, "Failed to set gpio as output low: %d\n", ret);
			return ret;
		}
	}
	
	return 0;
error:
	smb347_debugfs_exit(client);
	regulator_unregister(charger->otg_rdev);
otg_regulator_error:
	regulator_unregister(charger->rdev);
regulator_error:
	kfree(charger);
	charger = NULL;
	return ret;
}

static int __devexit smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *charger = i2c_get_clientdata(client);

	smb347_debugfs_exit(client);
	regulator_unregister(charger->rdev);
	regulator_unregister(charger->otg_rdev);
	kfree(charger);

	return 0;
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_i2c_driver = {
	.driver	= {
		.name	= "smb347",
	},
	.probe		= smb347_probe,
	.remove		= __devexit_p(smb347_remove),
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	return i2c_add_driver(&smb347_i2c_driver);
}
subsys_initcall(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_i2c_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Syed Rafiuddin <srafiuddin@nvidia.com>");
MODULE_DESCRIPTION("SMB347 Battery-Charger");
MODULE_LICENSE("GPL");
