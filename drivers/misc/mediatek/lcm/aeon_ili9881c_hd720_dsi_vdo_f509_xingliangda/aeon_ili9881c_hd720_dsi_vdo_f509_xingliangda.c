#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mt-plat/mt_gpio.h>
#endif

#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif


#define _LCM_DEBUG_

//#include <cust_gpio_usage.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define LCM_ID       (0x9881)
#define REGFLAG_DELAY             							0xFEFF
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

//#define GPIO_LCM_RST_PIN         (GPIO141 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//#define  GPIO_LCD_ID_PIN     80
//#define  GPIO_LCM_ID2_PIN    64


//#define _SYNA_INFO_
//#define _SYNA_DEBUG_
//#define _LCM_DEBUG_
//#define _LCM_INFO_
/*
#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args) */

#ifdef _LCM_DEBUG_
  #ifdef BUILD_LK
  #define LCM_PRINT printf
  #else
  #define LCM_PRINT printk
  #endif
#else
	#define LCM_PRINT
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

extern unsigned int opium_gpio_get(const char *name);

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
{ 0xff, 3, {0x98, 0x81, 0x07}},
//GIP_1
{ 0x03, 1, {0x20}},
{ 0x04, 1, {0x0c}},
{ 0x05, 1, {0x00}},
{ 0x06, 1, {0x00}},
{ 0x07, 1, {0x00}},
{ 0x08, 1, {0x00}},
{ 0x09, 1, {0x00}},
{ 0x0a, 1, {0x01}},
{ 0x0b, 1, {0x01}},
{ 0x0c, 1, {0x01}},
{ 0x0d, 1, {0x01}},
{ 0x0e, 1, {0x01}},
{ 0x0f, 1, {0x01}},
{ 0x10, 1, {0x44}},
{ 0x11, 1, {0x0a}},
{ 0x12, 1, {0x03}},
{ 0x13, 1, {0x99}},
{ 0x14, 1, {0x0a}},
{ 0x15, 1, {0x03}},
{ 0x16, 1, {0x01}},
{ 0x17, 1, {0x01}},
{ 0x18, 1, {0x00}},
{ 0x19, 1, {0x00}},
{ 0x1a, 1, {0x00}},
{ 0x1b, 1, {0xc0}},
{ 0x1c, 1, {0xb8}},
{ 0x1d, 1, {0x0b}},
{ 0x1e, 1, {0x01}},
{ 0x1f, 1, {0x8c}},
{ 0x20, 1, {0x8c}},
{ 0x21, 1, {0x00}},
{ 0x22, 1, {0x00}},
{ 0x23, 1, {0xc0}},
{ 0x24, 1, {0x30}},
{ 0x25, 1, {0x00}},
{ 0x26, 1, {0x00}},
{ 0x27, 1, {0x23}},
{ 0x30, 1, {0x01}},
{ 0x31, 1, {0x23}},
{ 0x32, 1, {0x45}},
{ 0x33, 1, {0x67}},
{ 0x34, 1, {0x89}},
{ 0x35, 1, {0xab}},
{ 0x36, 1, {0x01}},
{ 0x37, 1, {0x23}},
{ 0x38, 1, {0x45}},
{ 0x39, 1, {0x67}},
{ 0x3a, 1, {0x89}},
{ 0x3b, 1, {0xab}},
{ 0x3c, 1, {0xcd}},
{ 0x3d, 1, {0xef}},
//GIP_2
{ 0x50, 1, {0x11}},
{ 0x51, 1, {0x0c}},
{ 0x52, 1, {0x0d}},
{ 0x53, 1, {0x0e}},
{ 0x54, 1, {0x0f}},
{ 0x55, 1, {0x06}},
{ 0x56, 1, {0x07}},
{ 0x57, 1, {0x02}},
{ 0x58, 1, {0x02}},
{ 0x59, 1, {0x02}},
{ 0x5a, 1, {0x02}},
{ 0x5b, 1, {0x02}},
{ 0x5c, 1, {0x02}},
{ 0x5d, 1, {0x02}},
//GIP_3
{ 0x5e, 1, {0x02}},
{ 0x5f, 1, {0x02}},
{ 0x60, 1, {0x02}},
{ 0x61, 1, {0x02}},
{ 0x62, 1, {0x02}},
{ 0x63, 1, {0x02}},
{ 0x64, 1, {0x02}},
{ 0x65, 1, {0x01}},
{ 0x66, 1, {0x00}},
{ 0x67, 1, {0x0c}},
{ 0x68, 1, {0x0d}},
{ 0x69, 1, {0x0e}},
{ 0x6a, 1, {0x0f}},
{ 0x6b, 1, {0x06}},
{ 0x6c, 1, {0x07}},
{ 0x6d, 1, {0x02}},
{ 0x6e, 1, {0x02}},
{ 0x6f, 1, {0x02}},
{ 0x70, 1, {0x02}},
{ 0x71, 1, {0x02}},
{ 0x72, 1, {0x02}},
{ 0x73, 1, {0x02}},
{ 0x74, 1, {0x02}},
{ 0x75, 1, {0x02}},
{ 0x76, 1, {0x02}},
{ 0x77, 1, {0x02}},
{ 0x78, 1, {0x02}},
{ 0x79, 1, {0x02}},
{ 0x7a, 1, {0x02}},
{ 0x7b, 1, {0x01}},
{ 0x7c, 1, {0x00}},
{ 0xff, 3, {0x98, 0x81, 0x08}},
{ 0x76, 1, {0xa4}},
{ 0x78, 1, {0x02}},
{ 0x74, 1, {0x1a}},
{ 0x8e, 1, {0x20}},
{ 0x40, 1, {0x01}},
{ 0x84, 1, {0x81}},
{ 0x72, 1, {0x25}},
{ 0xe3, 1, {0x45}},
{ 0x7d, 1, {0xcb}},
{ 0x7e, 1, {0x49}},
{ 0x49, 1, {0x10}},
{ 0x7d, 1, {0xc4}},
{ 0x7e, 1, {0x45}},
{ 0x80, 1, {0xc4}},
{ 0x81, 1, {0x04}},
//CMD_Page4
{ 0xff, 3, {0x98, 0x81, 0x01}},
{ 0x22, 1, {0x0a}},

{ 0x31, 1, {0x00}},
{ 0x53, 1, {0x58}},
{ 0x55, 1, {0x53}},
//CMD_Page1
{ 0x50, 1, {0x78}},
{ 0x51, 1, {0x78}},

{ 0xa0, 1, {0x00}},
{ 0xa1, 1, {0x0a}},
{ 0xa2, 1, {0x14}},
{ 0xa3, 1, {0x0e}},
{ 0xa4, 1, {0x0f}},
{ 0xa5, 1, {0x1f}},
{ 0xa6, 1, {0x14}},
{ 0xa7, 1, {0x19}},
{ 0xa8, 1, {0x5b}},
{ 0xa9, 1, {0x1c}},
{ 0xaa, 1, {0x29}},
{ 0xab, 1, {0x5b}},
{ 0xac, 1, {0x19}},
{ 0xad, 1, {0x15}},
{ 0xae, 1, {0x48}},
{ 0xaf, 1, {0x1e}},
{ 0xb0, 1, {0x26}},
{ 0xb1, 1, {0x54}},
{ 0xb2, 1, {0x67}},
{ 0xb3, 1, {0x39}},

{ 0xc0, 1, {0x00}},
{ 0xc1, 1, {0x09}},
{ 0xc2, 1, {0x13}},
{ 0xc3, 1, {0x0d}},
{ 0xc4, 1, {0x0e}},
{ 0xc5, 1, {0x1f}},
{ 0xc6, 1, {0x14}},
{ 0xc7, 1, {0x18}},
{ 0xc8, 1, {0x5a}},
{ 0xc9, 1, {0x1c}},
{ 0xca, 1, {0x29}},
{ 0xcb, 1, {0x5b}},
{ 0xcc, 1, {0x18}},
{ 0xcd, 1, {0x16}},
{ 0xce, 1, {0x49}},
{ 0xcf, 1, {0x1d}},
{ 0xd0, 1, {0x27}},
{ 0xd1, 1, {0x54}},
{ 0xd2, 1, {0x66}},
{ 0xd3, 1, {0x39}},

{ 0xff, 3, {0x98, 0x81, 0x00}},
{ 0x11, 1, {0x00}},
{ REGFLAG_DELAY, 120, {}},
{ 0x29, 1, {0x00}},
};
/*
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

/*static struct LCM_setting_table lcm_backlight_level_setting[] = {	
{0xFF, 3, {0x98, 0x81, 0x00}},	
{0x51, 2, {0x0F,0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	LCM_PRINT("%s %d\n", __func__,__LINE__);
    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	LCM_PRINT("%s %d\n", __func__,__LINE__);
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
  	LCM_PRINT("opium %s %d\n", __func__,__LINE__);
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	 //params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	 //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = BURST_VDO_MODE;
//	params->dsi.mode   = SYNC_PULSE_VDO_MODE;	
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 20;
	params->dsi.vertical_frontporch					= 20;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 10;
	params->dsi.horizontal_backporch				= 80;
	params->dsi.horizontal_frontporch				= 80;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
		
        params->dsi.PLL_CLOCK = 220;//156;
	
#if 1
    params->dsi.esd_check_enable =1;
	params->dsi.customization_esd_check_enable =1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
	params->dsi.lcm_esd_check_table[0].count =1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
#else
	params->dsi.cont_clock=1;
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;
#endif
	params->dsi.ssc_disable = 1;
//	params->dsi.clk_lp_per_line_enable=1;
	// Bit rate calculation
	// params->dsi.pll_div1=0;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
	// params->dsi.pll_div2=1; 		// div2=0~15: fout=fvo/(2*div2)
	// params->dsi.fbk_div =11;    //fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

}

static unsigned int lcm_compare_id(void)
{
    
    unsigned int lcd_id;
    unsigned char lcm_id, id_high, id_low;
    unsigned int array[16];	
    
        MDELAY(10);
        SET_RESET_PIN(0);
        MDELAY(20);
        SET_RESET_PIN(1);
        MDELAY(80);
        
        
        array[0] = 0x43902;
        array[1] = 0x18198FF;
        
        dsi_set_cmdq(array, 2, 1);
        MDELAY(10);

    
        array[0] = 0x00013700;	/* read id return one byte*/
        dsi_set_cmdq(array, 1, 1);
        MDELAY(10);
        
        read_reg_v2(0x0, &id_high, 1);
        read_reg_v2(0x1, &id_low, 1);
        read_reg_v2(0x2, &lcm_id, 1);

        lcd_id = id_low | (id_high << 8);         
        return (LCM_ID == lcd_id)?1 :0;	
}

static void lcm_init(void)
{
	//unsigned int data_array[16];
	LCM_PRINT(" %s %d\n", __func__,__LINE__);
	SET_RESET_PIN(1);
	MDELAY(10);
        SET_RESET_PIN(0);
	MDELAY(20);//50
        SET_RESET_PIN(1);
	MDELAY(100);//100
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
//	LCM_PRINT("%s %d\n", __func__,__LINE__);
//	SET_GPIO_OUT(GPIO_LCM_RST_PIN,1);
//	MDELAY(1);	
//	SET_GPIO_OUT(GPIO_LCM_RST_PIN,1);
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(20);//50

        SET_RESET_PIN(1);
	MDELAY(10);
        SET_RESET_PIN(0);
	MDELAY(20);//50
        SET_RESET_PIN(1);

//	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);

//	SET_GPIO_OUT(GPIO_LCM_PWR_EN,0);//Disable LCM Power
}

static void lcm_resume(void)
{
//	LCM_PRINT("%s %d\n", __func__,__LINE__);
	lcm_init();
}

#if 0
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	LCM_PRINT("%s %d\n", __func__,__LINE__);
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00000000;
	data_array[4]= 0x00053902;
	data_array[5]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[6]= (y1_LSB);
	data_array[7]= 0x00000000;
	data_array[8]= 0x002c3909;

	dsi_set_cmdq(&data_array, 9, 0);

}
#endif

LCM_DRIVER aeon_ili9881c_hd720_dsi_vdo_f509_xingliangda_lcm_drv = 
{
	.name           = "aeon_ili9881c_hd720_dsi_vdo_f509_xingliangda",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
