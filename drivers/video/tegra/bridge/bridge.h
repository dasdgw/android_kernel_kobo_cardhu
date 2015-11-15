/*
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 */
#ifndef BRIDGE_H

#define BRIDGE_H
#include <linux/spi/spi.h>

#define STR_RGB_BRIDGE	"rgb_bridge_spi"

struct bridge_platform_data {
	u8 mode;
	u8 bits_per_word;
	u32 max_speed_hz;
};


extern int bridge_enable(void);
extern int bridge_disable(void);

#define FALSE 0
#define TRUE 1

#endif
