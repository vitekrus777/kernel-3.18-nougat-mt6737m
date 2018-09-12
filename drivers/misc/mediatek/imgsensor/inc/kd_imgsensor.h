/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _KD_IMGSENSOR_H
#define _KD_IMGSENSOR_H

#include <linux/ioctl.h>
/* #define CONFIG_COMPAT */
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#ifndef ASSERT
#define ASSERT(expr)        BUG_ON(!(expr))
#endif

#define IMGSENSORMAGIC 'i'
/* IOCTRL(inode * ,file * ,cmd ,arg ) */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */

/*******************************************************************************
*
********************************************************************************/
#define YUV_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : YUVSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : YUVSensorInfo < _id >  :  : getInstance, \
NSFeature :  : YUVSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : YUVSensorInfo < _id >  :  : getNullFlickerPara \
	}
#define RAW_INFO(_id, name, getCalData)\
	{ \
		_id, name, \
NSFeature :  : RAWSensorInfo < _id >  :  : createInstance(name, #name), \
		(NSFeature :  : SensorInfoBase*(*)()) \
NSFeature :  : RAWSensorInfo < _id >  :  : getInstance, \
NSFeature :  : RAWSensorInfo < _id >  :  : getDefaultData, \
		getCalData, \
NSFeature :  : RAWSensorInfo < _id >  :  : getFlickerPara \
	}
/*******************************************************************************
*
********************************************************************************/

/* sensorOpen */
#define KDIMGSENSORIOC_T_OPEN                       _IO(IMGSENSORMAGIC, 0)
/* sensorGetInfo */
#define KDIMGSENSORIOC_X_GETINFO                    _IOWR(IMGSENSORMAGIC, 5, ACDK_SENSOR_GETINFO_STRUCT)
/* sensorGetResolution */
#define KDIMGSENSORIOC_X_GETRESOLUTION              _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_RESOLUTION_INFO_STRUCT)
/* For kernel 64-bit */
#define KDIMGSENSORIOC_X_GETRESOLUTION2             _IOWR(IMGSENSORMAGIC, 10, ACDK_SENSOR_PRESOLUTION_STRUCT)
/* sensorFeatureControl */
#define KDIMGSENSORIOC_X_FEATURECONCTROL            _IOWR(IMGSENSORMAGIC, 15, ACDK_SENSOR_FEATURECONTROL_STRUCT)
/* sensorControl */
#define KDIMGSENSORIOC_X_CONTROL                    _IOWR(IMGSENSORMAGIC, 20, ACDK_SENSOR_CONTROL_STRUCT)
/* sensorClose */
#define KDIMGSENSORIOC_T_CLOSE                      _IO(IMGSENSORMAGIC, 25)
/* sensorSearch */
#define KDIMGSENSORIOC_T_CHECK_IS_ALIVE             _IO(IMGSENSORMAGIC, 30)
/* set sensor driver */
#define KDIMGSENSORIOC_X_SET_DRIVER                 _IOWR(IMGSENSORMAGIC, 35, SENSOR_DRIVER_INDEX_STRUCT)
/* get socket postion */
#define KDIMGSENSORIOC_X_GET_SOCKET_POS             _IOWR(IMGSENSORMAGIC, 40, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_SET_I2CBUS                 _IOWR(IMGSENSORMAGIC, 45, u32)
/* set I2C bus */
#define KDIMGSENSORIOC_X_RELEASE_I2C_TRIGGER_LOCK   _IO(IMGSENSORMAGIC, 50)
/* Set Shutter Gain Wait Done */
#define KDIMGSENSORIOC_X_SET_SHUTTER_GAIN_WAIT_DONE _IOWR(IMGSENSORMAGIC, 55, u32)
/* set mclk */
#define KDIMGSENSORIOC_X_SET_MCLK_PLL               _IOWR(IMGSENSORMAGIC, 60, ACDK_SENSOR_MCLK_STRUCT)
#define KDIMGSENSORIOC_X_GETINFO2                   _IOWR(IMGSENSORMAGIC, 65, IMAGESENSOR_GETINFO_STRUCT)
/* set open/close sensor index */
#define KDIMGSENSORIOC_X_SET_CURRENT_SENSOR         _IOWR(IMGSENSORMAGIC, 70, u32)
/* set GPIO */
#define KDIMGSENSORIOC_X_SET_GPIO                   _IOWR(IMGSENSORMAGIC, 75, IMGSENSOR_GPIO_STRUCT)
/* Get ISP CLK */
#define KDIMGSENSORIOC_X_GET_ISP_CLK                _IOWR(IMGSENSORMAGIC, 80, u32)
/* Get CSI CLK */
#define KDIMGSENSORIOC_X_GET_CSI_CLK                _IOWR(IMGSENSORMAGIC, 85, u32)

#ifdef CONFIG_COMPAT
#define COMPAT_KDIMGSENSORIOC_X_GETINFO            _IOWR(IMGSENSORMAGIC, 5, COMPAT_ACDK_SENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_FEATURECONCTROL    _IOWR(IMGSENSORMAGIC, 15, COMPAT_ACDK_SENSOR_FEATURECONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_CONTROL            _IOWR(IMGSENSORMAGIC, 20, COMPAT_ACDK_SENSOR_CONTROL_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETINFO2           _IOWR(IMGSENSORMAGIC, 65, COMPAT_IMAGESENSOR_GETINFO_STRUCT)
#define COMPAT_KDIMGSENSORIOC_X_GETRESOLUTION2     _IOWR(IMGSENSORMAGIC, 10, COMPAT_ACDK_SENSOR_PRESOLUTION_STRUCT)
#endif

/*******************************************************************************
*
********************************************************************************/
/* SENSOR CHIP VERSION */
/*IMX*/
#define IMX219_SENSOR_ID                        0x0219
/*HI*/
#define HI843B_SENSOR_ID                        0x0843
/*GC*/
#define GC2365_SENSOR_ID                        0x2365
/* CAMERA DRIVER NAME */
#define CAMERA_HW_DEVNAME                       "kd_camera_hw"
/* SENSOR DEVICE DRIVER NAME */
/*IMX*/
#define SENSOR_DRVNAME_IMX219_MIPI_RAW          "imx219mipiraw"
/*HI*/
#define SENSOR_DRVNAME_HI843B_MIPI_RAW          "hi843bmipiraw"
/*GC*/
#define SENSOR_DRVNAME_GC2365_MIPI_RAW          "gc2365mipiraw"
/*******************************************************************************
*
********************************************************************************/
void KD_IMGSENSOR_PROFILE_INIT(void);
void KD_IMGSENSOR_PROFILE(char *tag);
void KD_IMGSENSOR_PROFILE_INIT_I2C(void);
void KD_IMGSENSOR_PROFILE_I2C(char *tag, int trans_num);

#define mDELAY(ms)     mdelay(ms)
#define uDELAY(us)       udelay(us)
#endif              /* _KD_IMGSENSOR_H */
