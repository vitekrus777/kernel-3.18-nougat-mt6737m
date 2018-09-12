#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/compat.h>
#include <linux/platform_device.h>



struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *fingerprint_power_on, *fingerprint_power_off,*fingerprint_reset_high,*fingerprint_reset_low,*fingerprint_spi0_mi_as_spi0_mi,*fingerprint_spi0_mi_as_gpio,
*fingerprint_spi0_mo_as_spi0_mo,*fingerprint_spi0_mo_as_gpio,*fingerprint_spi0_clk_as_spi0_clk,*fingerprint_spi0_clk_as_gpio,*fingerprint_spi0_cs_as_spi0_cs,*fingerprint_spi0_cs_as_gpio,
*fingerprint_int_as_int;

int fingerprint_get_gpio_info(struct platform_device *pdev)
{
	struct device_node *node;
	int ret;
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6737m-fingerprint");
	//printk("node.name %s full name %s",node->name,node->full_name);


		fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
		if (IS_ERR(fingerprint_pinctrl)) {
			ret = PTR_ERR(fingerprint_pinctrl);
			dev_err(&pdev->dev, "mt6737m-fingerprint cannot find pinctrl\n");
				return ret;
		}

   // printk("[%s] fingerprint_pinctrl+++++++++++++++++\n",pdev->name);

	fingerprint_power_on = pinctrl_lookup_state(fingerprint_pinctrl, "finger_power_en1");
	if (IS_ERR(fingerprint_power_on)) {
		ret = PTR_ERR(fingerprint_power_on);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_power_on!\n");
		return ret;
	}
	fingerprint_power_off = pinctrl_lookup_state(fingerprint_pinctrl, "finger_power_en0");
	if (IS_ERR(fingerprint_power_off)) {
		ret = PTR_ERR(fingerprint_power_off);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_power_off!\n");
		return ret;
	}
	fingerprint_reset_high = pinctrl_lookup_state(fingerprint_pinctrl, "finger_reset_en1");
	if (IS_ERR(fingerprint_reset_high)) {
		ret = PTR_ERR(fingerprint_reset_high);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_reset_high!\n");
		return ret;
	}
	fingerprint_reset_low = pinctrl_lookup_state(fingerprint_pinctrl, "finger_reset_en0");
	if (IS_ERR(fingerprint_reset_low)) {
		ret = PTR_ERR(fingerprint_reset_low);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_reset_low!\n");
		return ret;
	}
	fingerprint_spi0_mi_as_spi0_mi = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_mi_as_spi0_mi");
	if (IS_ERR(fingerprint_spi0_mi_as_spi0_mi)) {
		ret = PTR_ERR(fingerprint_spi0_mi_as_spi0_mi);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_mi_as_spi0_mi!\n");
		return ret;
	}
	fingerprint_spi0_mi_as_gpio = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_mi_as_gpio");
	if (IS_ERR(fingerprint_spi0_mi_as_gpio)) {
		ret = PTR_ERR(fingerprint_spi0_mi_as_gpio);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_mi_as_gpio!\n");
		return ret;
	}
	fingerprint_spi0_mo_as_spi0_mo = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_mo_as_spi0_mo");
	if (IS_ERR(fingerprint_spi0_mo_as_spi0_mo)) {
		ret = PTR_ERR(fingerprint_spi0_mo_as_spi0_mo);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_mo_as_spi0_mo!\n");
		return ret;
	}
	fingerprint_spi0_mo_as_gpio = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_mo_as_gpio");
	if (IS_ERR(fingerprint_spi0_mo_as_gpio)) {
		ret = PTR_ERR(fingerprint_spi0_mo_as_gpio);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_mo_as_gpio!\n");
		return ret;
	}
	fingerprint_spi0_clk_as_spi0_clk = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_clk_as_spi0_clk");
	if (IS_ERR(fingerprint_spi0_clk_as_spi0_clk)) {
		ret = PTR_ERR(fingerprint_spi0_clk_as_spi0_clk);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_clk_as_spi0_clk!\n");
		return ret;
	}
	fingerprint_spi0_clk_as_gpio = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_clk_as_gpio");
	if (IS_ERR(fingerprint_spi0_clk_as_gpio)) {
		ret = PTR_ERR(fingerprint_spi0_clk_as_gpio);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_clk_as_gpio!\n");
		return ret;
	}
	fingerprint_spi0_cs_as_spi0_cs = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_cs_as_spi0_cs");
	if (IS_ERR(fingerprint_spi0_cs_as_spi0_cs)) {
		ret = PTR_ERR(fingerprint_spi0_cs_as_spi0_cs);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_cs_as_spi0_cs!\n");
		return ret;
	}
	fingerprint_spi0_cs_as_gpio = pinctrl_lookup_state(fingerprint_pinctrl, "finger_spi0_cs_as_gpio");
	if (IS_ERR(fingerprint_spi0_cs_as_gpio)) {
		ret = PTR_ERR(fingerprint_spi0_cs_as_gpio);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_cs_as_gpio!\n");
		return ret;
	}
	fingerprint_int_as_int = pinctrl_lookup_state(fingerprint_pinctrl, "finger_int_as_int");
	if (IS_ERR(fingerprint_int_as_int)) {
		ret = PTR_ERR(fingerprint_int_as_int);
		dev_err(&pdev->dev, " Cannot find mt6737m-fingerprint pinctrl fingerprint_spi0_cs_as_gpio!\n");
		return ret;
	}
//	printk("mt6737m-fingerprint get gpio info ok--------");
	return 0;
}

int fingerprint_set_power(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_power_off);
		break;
		case 1 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_power_on);
		break;
		}
	return 0;
}

int fingerprint_set_reset(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_reset_low);
		break;
		case 1 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_reset_high);
		break;
		}
	return 0;
}

int fingerprint_set_eint(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			return -1;
		break;
		case 1 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_int_as_int);
		break;
		}
	return 0;
}

int fingerprint_set_spi_mode(int cmd)
{
	switch (cmd)
		{
		case 0 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_clk_as_gpio);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_cs_as_gpio);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_mi_as_gpio);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_mo_as_gpio);
		break;
		case 1 : 		
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_clk_as_spi0_clk);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_cs_as_spi0_cs);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_mi_as_spi0_mi);
			pinctrl_select_state(fingerprint_pinctrl, fingerprint_spi0_mo_as_spi0_mo);
		break;
		}
	return 0;
}



