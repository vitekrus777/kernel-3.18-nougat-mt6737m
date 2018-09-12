/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
#ifndef _KD_CAMERA_HW_H_
#define _KD_CAMERA_HW_H_


#include <linux/types.h>
#include "kd_camera_typedef.h"

#define CAMERA_CMRST_PIN            0
#define CAMERA_CMRST_PIN_M_GPIO     0

#define CAMERA_CMPDN_PIN            0
#define CAMERA_CMPDN_PIN_M_GPIO     0

/* FRONT sensor */
#define CAMERA_CMRST1_PIN           0
#define CAMERA_CMRST1_PIN_M_GPIO    0

#define CAMERA_CMPDN1_PIN           0
#define CAMERA_CMPDN1_PIN_M_GPIO    0

#define GPIO_OUT_ONE 1
#define GPIO_OUT_ZERO 0


typedef enum KD_REGULATOR_TYPE_TAG {
	VCAMA,
	VCAMD,
	VCAMIO,
	VCAMAF,
	SUB_VCAMD
} KD_REGULATOR_TYPE_T;

typedef enum {
	CAMPDN,
	CAMRST,
	CAM1PDN,
	CAM1RST,
	CAMDLDO,
	CAMALDO
} CAMPowerType;

extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);


extern bool _hwPowerDown(KD_REGULATOR_TYPE_T type);
extern bool _hwPowerOn(KD_REGULATOR_TYPE_T type, int powerVolt);

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val);
int mtkcam_gpio_init(struct platform_device *pdev);

#endif

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1000 1000000

typedef enum{
    VDD_None = 0,
    SensorId = 0xf1,
    SensorMCLK = 0xf2,
    PDN = 0xf3,
    RST = 0xf4,
    AVDD = 0xf5,
    DVDD = 0xf6,
    DOVDD = 0xf7,
    AFVDD = 0xf8
}PowerType;

typedef enum{
	MAIN_SENSOR = 0,
	SUB_SENSOR = 1,
	MAIN_2_SENSOR = 2,
	Mclk1 = 0xf1,
	Mclk2 = 0xf2,
    Vol_Low = 0xf3,
    Vol_High = 0xf4,
    Vol_1000 = VOL_1000,
    Vol_1200 = VOL_1200,
    Vol_1500 = VOL_1500,
    Vol_1800 = VOL_1800,
    Vol_2800 = VOL_2800
}Voltage;

typedef struct{
    PowerType PowerType;
    Voltage Voltage;
    u32 Delay;
}PowerInformation;

typedef struct{
    char* SensorName;
    PowerInformation PowerInfo[15];
}PowerSequence;

typedef struct{
    PowerSequence PowerSeq[16];
}PowerUp;
