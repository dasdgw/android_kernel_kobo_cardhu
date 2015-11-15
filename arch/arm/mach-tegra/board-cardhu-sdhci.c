/*
 * arch/arm/mach-tegra/board-harmony-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011-2012 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include "gpio-names.h"
#include "board.h"
#include "board-cardhu.h"

#define EXTERNAL_SD_ENABLE 0

#define CARDHU_WLAN_PWR	TEGRA_GPIO_PD4
#define CARDHU_WLAN_WOW	TEGRA_GPIO_PU5
#define CARDHU_SD_CD TEGRA_GPIO_PI5
#define CARDHU_SD_WP TEGRA_GPIO_PT3
#define PM269_SD_WP -1

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int cardhu_wifi_status_register(void (*callback)(int , void *), void *);

static int cardhu_wifi_reset(int on);
static int cardhu_wifi_power(int on);
static int cardhu_wifi_set_carddetect(int val);

static struct wifi_platform_data cardhu_wifi_control = {
	.set_power	= cardhu_wifi_power,
	.set_reset	= cardhu_wifi_reset,
	.set_carddetect	= cardhu_wifi_set_carddetect,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
		.end	= TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU5),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device broadcom_wifi_device = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &cardhu_wifi_control,
	},
};

static struct platform_device marvell_wifi_device = {
	.name		= "mrvl8797_wlan",
	.id		= 1,
	.num_resources	= 0,
	.dev		= {
		.platform_data = &cardhu_wifi_control,
	},
};

#if EXTERNAL_SD_ENABLE
static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};
#endif

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct embedded_sdio_data embedded_sdio_data2 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4329,
	},
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.register_status_notify	= cardhu_wifi_status_register,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio = &embedded_sdio_data2,
#endif
		.built_in = 0,
		.ocr_mask = MMC_OCR_1V8_MASK,
	},
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.pm_flags = MMC_PM_KEEP_POWER,
#endif
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
/*	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 0,
	.is_8bit_supported = false, */
};

#if EXTERNAL_SD_ENABLE
static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.cd_gpio = CARDHU_SD_CD,
	.wp_gpio = CARDHU_SD_WP,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
/*	.is_voltage_switch_supported = true,
	.vdd_rail_name = "vddio_sdmmc1",
	.slot_rail_name = "vddio_sd_slot",
	.vdd_max_uv = 3320000,
	.vdd_min_uv = 3280000,
	.max_clk = 208000000,
	.is_8bit_supported = false, */
};
#endif

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
	}
/*	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 48000000,
	.is_8bit_supported = true, */
};

#if EXTERNAL_SD_ENABLE
static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};
#endif

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static spinlock_t my_lock;
static unsigned long my_flags;
static enum wifibt_status wifi_state;
static enum wifibt_status bt_state;
static enum wifibt_status wifibt_reg_state;

#define WIFIBT_3v3	TEGRA_GPIO_PD0

void wifibt_setpower(int onoff,enum wifibt_status state )
{
	pr_debug("%s: onoff=[%d] stat=[%d] \n", __func__,onoff,state);
	pr_debug("%s: previous state :wifi_state=[%d] bt_state=[%d] wifibt_reg_state=[%d] \n", __func__,wifi_state,bt_state,wifibt_reg_state);
	spin_lock_irqsave(&my_lock,my_flags);

	switch(onoff)
	{
		case 0:
				if((state==BT_OFF)&&(wifi_state==WIFI_OFF))
				{
					gpio_set_value(WIFIBT_3v3,0); // disable RF_EN_WIFI_BT_REG 3v3
					wifibt_reg_state=POWER_OFF;
					pr_debug("%s: Turn OFF 3v3 by %s = %d \n", __func__,(state==BT_OFF) ? "BT_OFF":"WIFI_OFF",state);
				}
				else if((state==WIFI_OFF)&&(bt_state==BT_OFF))
				{
					gpio_set_value(WIFIBT_3v3,0); // disable RF_EN_WIFI_BT_REG 3v3
					wifibt_reg_state=POWER_OFF;
					pr_debug("%s: Turn OFF 3v3 by %s = %d \n", __func__,(state==WIFI_OFF) ? "WIFI_OFF":"BT_OFF",state);
				}
				if(state==WIFI_OFF)wifi_state=WIFI_OFF;
				if(state==BT_OFF)bt_state=BT_OFF;

				break;
		case 1:
				if(wifibt_reg_state==POWER_OFF)
				{
					gpio_set_value(WIFIBT_3v3,1); // enable RF_EN_WIFI_BT_REG 3v3
					wifibt_reg_state=POWER_ON;
					pr_debug("%s: Turn ON 3v3 by %s = %d wifibt_reg_state=%d \n", __func__,(state==BT_ON) ? "BT_ON":"WIFI_ON",state,wifibt_reg_state);

				}
				if((state==WIFI_ON) && (wifibt_reg_state==POWER_ON))
					wifi_state=WIFI_ON;
				else if ((state==BT_ON)&& (wifibt_reg_state==POWER_ON))
					bt_state=BT_ON;

				break;
		default:
		pr_debug("%s: unknow status onoff = %d state = %d  wifibt_reg = %d  \n", __func__,onoff,state,wifibt_reg_state);

	}

	spin_unlock_irqrestore(&my_lock,my_flags);

}

static void cardhu_wifibt_reg_init(void)
{
	int rc=0;
	pr_debug("%s: \n", __func__);
	wifi_state=WIFI_OFF;
	bt_state=BT_OFF;
	wifibt_reg_state=POWER_OFF;
	spin_lock_init(&my_lock);
	rc=gpio_request(WIFIBT_3v3,"wifibt_3v3");
	if (rc)
		pr_err("RF_EN_WIFI_BT_REG 3v3 gpio request failed:%d\n", rc);
	rc=gpio_direction_output(WIFIBT_3v3, 0);
	if (rc)
		pr_err("RF_EN_WIFI_BT_REG gpio direction configuration failed:%d\n", rc);

}

static int cardhu_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int cardhu_wifi_set_carddetect(int val)
{
	pr_debug("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

static int cardhu_wifi_power(int on)
{
	struct tegra_io_dpd *sd_dpd;

	pr_debug("%s: %d\n", __func__, on);

	/*
	 * FIXME : we need to revisit IO DPD code
	 * on how should multiple pins under DPD get controlled
	 *
	 * cardhu GPIO WLAN enable is part of SDMMC3 pin group
	 */
	sd_dpd = tegra_io_dpd_get(&tegra_sdhci_device2.dev);
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_disable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}

	if(on==0){
		gpio_set_value(CARDHU_WLAN_PWR, on);
		wifibt_setpower(on,WIFI_OFF);
	}
	else{
		wifibt_setpower(on,WIFI_ON);
		mdelay(10);
		gpio_set_value(CARDHU_WLAN_PWR,on);
	}

	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_enable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}

	return 0;
}

static int cardhu_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init cardhu_wifi_init(void)
{
	int rc;
	int commchip_id = tegra_get_commchip_id();

	cardhu_wifibt_reg_init();

	rc = gpio_request(CARDHU_WLAN_PWR, "wlan_power");
	if (rc)
		pr_err("WLAN_PWR gpio request failed:%d\n", rc);
	rc = gpio_request(CARDHU_WLAN_WOW, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);

	rc = gpio_direction_output(CARDHU_WLAN_PWR, 0);
	if (rc)
		pr_err("WLAN_PWR gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(CARDHU_WLAN_WOW);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	if (commchip_id == COMMCHIP_MARVELL_SD8797)
		platform_device_register(&marvell_wifi_device);
	else
		platform_device_register(&broadcom_wifi_device);

	return 0;
}

#ifdef CONFIG_TEGRA_PREPOWER_WIFI
static int __init cardhu_wifi_prepower(void)
{
	if (!machine_is_cardhu())
		return 0;

	cardhu_wifi_power(1);

	return 0;
}

subsys_initcall_sync(cardhu_wifi_prepower);
#endif

int __init cardhu_sdhci_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if ((board_info.board_id == BOARD_PM269) ||
		(board_info.board_id == BOARD_E1257) ||
		(board_info.board_id == BOARD_PM305) ||
		(board_info.board_id == BOARD_PM311)) {
#if EXTERNAL_SD_ENABLE
			tegra_sdhci_platform_data0.wp_gpio = PM269_SD_WP;
#endif
			tegra_sdhci_platform_data2.max_clk_limit = 12000000;
	}

	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
#if EXTERNAL_SD_ENABLE
	platform_device_register(&tegra_sdhci_device0);
#endif

	cardhu_wifi_init();
	return 0;
}
