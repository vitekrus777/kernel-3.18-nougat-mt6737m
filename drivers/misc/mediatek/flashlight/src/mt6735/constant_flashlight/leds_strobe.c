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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/semaphore.h>
#include <linux/gpio.h>

#include <mt-plat/mt_pwm.h>
#include <mach/mt_clkmgr.h>
#include <mt-plat/upmu_common.h>
#include <linux/regulator/consumer.h>

/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)

/*#define DEBUG_LEDS_STROBE*/
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res;
static u32 strobe_Timeus;
static BOOL g_strobe_On;

static int g_duty = -1;
static int g_timeOutTimeMs;

static DEFINE_MUTEX(g_strobeSem);

static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);


/*****************************************************************************
Functions
*****************************************************************************/
struct pinctrl *flashctrl = NULL;
struct pinctrl_state *en_gpio_high = NULL;
struct pinctrl_state *en_gpio_low = NULL;

int flashlight_gpio_init(struct platform_device *pdev)
{
  int ret = 0;

  printk(KERN_ERR "%s line %d \n",__func__,__LINE__);
  flashctrl = devm_pinctrl_get(&pdev->dev);
  if (IS_ERR(flashctrl)) {
    dev_err(&pdev->dev, "Cannot find flash pinctrl!");
    printk(KERN_ERR"--------Cannot find flash pinctrl -----------\n");
    ret = PTR_ERR(flashctrl);
  }
  en_gpio_high = pinctrl_lookup_state(flashctrl, "state_en_output1");
  if (IS_ERR(en_gpio_high)) {
    ret = PTR_ERR(en_gpio_high);
    pr_err("%s : pinctrl err, en_gpio_high\n", __func__);
  }

  en_gpio_low = pinctrl_lookup_state(flashctrl, "state_en_output0");
  if (IS_ERR(en_gpio_low)) {
    ret = PTR_ERR(en_gpio_low);
    pr_err("%s : pinctrl err, en_gpio_low\n", __func__);
  }
  
  printk(KERN_ERR "%s line %d \n",__func__,__LINE__);
  return ret;
}

static void gpio_pwm_flash_50(void)
{
	struct pwm_spec_config pwm_setting;
	printk("%s Enter\n",__func__);
	pwm_setting.pwm_no  = 3;
	pwm_setting.mode    = PWM_MODE_FIFO;
	pwm_setting.clk_div = CLK_DIV8;
	pwm_setting.clk_src = PWM_CLK_NEW_MODE_BLOCK;
	pwm_setting.PWM_MODE_FIFO_REGS.IDLE_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.GUARD_VALUE = false;
	pwm_setting.PWM_MODE_FIFO_REGS.STOP_BITPOS_VALUE = 63;
	pwm_setting.PWM_MODE_FIFO_REGS.HDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.LDURATION = 1;
	pwm_setting.PWM_MODE_FIFO_REGS.GDURATION = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.WAVE_NUM  = 0;
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA0 = 0xffffffff;   //50%
	pwm_setting.PWM_MODE_FIFO_REGS.SEND_DATA1 = 0x00ffffff;
	pwm_set_spec_config(&pwm_setting);
}

static void gpio_flash_close(void)
{
	PK_DBG(" gpio_flash_close line=%d\n",__LINE__);
	pinctrl_select_state(flashctrl,en_gpio_high);
}

static int FL_Enable(void)
{
	PK_DBG(" FL_Enable g_duty = %d\n",g_duty);

	if(g_duty > 1)//flashlight
	{
		pinctrl_select_state(flashctrl,en_gpio_high);
		mdelay(2); 
		gpio_pwm_flash_50();
	}
	else//torch
	{
		pinctrl_select_state(flashctrl,en_gpio_low);
		mdelay(10); 
		gpio_pwm_flash_50();
	}
    return 0;
}

static int FL_Disable(void)
{
	struct pwm_spec_config pwm_setting ;
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
	pwm_setting.pwm_no  = 3;
	mt_pwm_disable(pwm_setting.pwm_no, false);
    gpio_flash_close();
    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    g_duty = duty;
    return 0;
}

static int FL_Init(void)
{
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    gpio_flash_close();
    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000; //1s
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}


static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",(int)arg);
			g_timeOutTimeMs=arg;
    		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",(int)arg);
    		FL_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("FLASH_IOC_SET_STEP: %d\n",(int)arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
			if (arg == 1) {
				int s;
				int ms;

				if (g_timeOutTimeMs > 1000) {
					s = g_timeOutTimeMs / 1000;
					ms = g_timeOutTimeMs - s * 1000;
				} else {
					s = 0;
					ms = g_timeOutTimeMs;
				}

				if (g_timeOutTimeMs != 0) {
					ktime_t ktime;

					ktime = ktime_set(s, ms * 1000000);
					hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
				}
				FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;

		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
        /* LED On Status */
        g_strobe_On = TRUE;
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		PK_DBG(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}
EXPORT_SYMBOL(constantFlashlightInit);


/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{
	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);

/*************opium add for factory mode flashlight test*********/
int Flashlight_Switch=0;//opium add for factory mode  flashlight test
static int flag = 1;

void Flashlight_ON(void)
{
	//hrtimer_cancel( &g_timeOutTimer );
	FL_dim_duty(1);
	if(0 == strobe_Res)
	{	
		FL_Init();
		Flashlight_Switch=0;
	}
	if(flag==1)
	{
		FL_Enable();
		Flashlight_Switch=1;
	}
}

void Flashlight_OFF(void)
{	
	FL_Uninit();
	Flashlight_Switch=0;
}

EXPORT_SYMBOL(Flashlight_ON);
EXPORT_SYMBOL(Flashlight_OFF);
EXPORT_SYMBOL(Flashlight_Switch);
/**************************end**********************/
