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

//s_add new sensor driver here
//export funtions
/*IMX*/
UINT32 IMX219_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*HI*/
UINT32 HI843B_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
/*GC*/
UINT32 GC2365_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc);
//! Add Sensor Init function here
//! Note:
//! 1. Add by the resolution from ""large to small"", due to large sensor
//!    will be possible to be main sensor.
//!    This can avoid I2C error during searching sensor.
//! 2. This file should be the same as mediatek\custom\common\hal\imgsensor\src\sensorlist.cpp
ACDK_KD_SENSOR_INIT_FUNCTION_STRUCT kdSensorList[MAX_NUM_OF_SUPPORT_SENSOR+1] =
{
/*IMX*/
#if defined(IMX219_MIPI_RAW)
    {IMX219_SENSOR_ID, SENSOR_DRVNAME_IMX219_MIPI_RAW, IMX219_MIPI_RAW_SensorInit},
#endif
/*HI*/
#if defined(HI843B_MIPI_RAW)
    {HI843B_SENSOR_ID, SENSOR_DRVNAME_HI843B_MIPI_RAW,HI843B_MIPI_RAW_SensorInit},
#endif
/*GC*/
#if defined(GC2365_MIPI_RAW)
    {GC2365_SENSOR_ID, SENSOR_DRVNAME_GC2365_MIPI_RAW,GC2365_MIPI_RAW_SensorInit},
#endif
/*  ADD sensor driver before this line */
    {0,{0},NULL}, //end of list
};
//e_add new sensor driver here

