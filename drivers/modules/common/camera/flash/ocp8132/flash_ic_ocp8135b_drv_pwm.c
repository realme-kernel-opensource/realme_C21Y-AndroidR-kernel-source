/*
 * Copyright (C) 2015-2016 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/vmalloc.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/clk.h>
#include <linux/types.h>

#include "sprd_img.h"
#include "flash_drv.h"

#ifdef pr_fmt
#undef pr_fmt
#endif
#define pr_fmt(fmt) "FLASH_OCP8135B: %d %d %s : " fmt, current->pid, __LINE__, __func__

#define FLASH_DRIVER_NAME "flash_ocp8135b"
#define FLASH_GPIO_ENF_NAME "flash-en-gpios"
#define FLASH_GPIO_ENM_NAME "flash-enm-gpios"
#define BOARDID_GPIO_MAX 3

#define CAMERA_PREFLASH_LIGHT_DUTY 62
#define CAMERA_TORCH_LIGHT_DUTY 37
#define FLASH_MAX_LEVEL 16

struct mutex flash_lock;
struct wakeup_source    wake_lock;

enum {
    GPIO_BOARDID1,
    GPIO_BOARDID2,
    GPIO_BOARDID3,
};
static const char *const boardId_gpio_names[BOARDID_GPIO_MAX] = {
    "board_id1",
    "board_id2",
    "board_id3",
};

struct flash_driver_data {
    int gpio_63_ENF;
    int gpio_123_ENM;
    struct pinctrl *pin_gpio_123;
    struct pinctrl_state *pinctrl_state_gpio;
    struct pinctrl_state *pinctrl_state_pwm;
    int torch_led_index;
    struct pwm_device *pwm_chip;
    int is_old_board;
	struct sprd_flash_capacity *flash_ic_info;
};
static struct sprd_flash_capacity flash_ic_info = {0};

struct sprd_flash_capacity sprd_flash_ic_get_flash_info(void *drvd, uint8_t idx,
					   struct sprd_flash_capacity *element)
{

	element = &flash_ic_info;
	pr_info("sprd_flash_ic_get_flash_info flash_ic_name %s %s\n", element->flash_ic_name,flash_ic_info.flash_ic_name);


	return flash_ic_info;
}

static int sprd_pin123_pwm_selected(void *drvd, bool is_pwm_selected)
{
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int ret;

    if (is_pwm_selected) {
        ret =  pinctrl_select_state(drv_data->pin_gpio_123, drv_data->pinctrl_state_pwm);
    } else {
        ret =  pinctrl_select_state(drv_data->pin_gpio_123, drv_data->pinctrl_state_gpio);
    }
    pr_info("is_pwm_selected:%d ret:%d\n", is_pwm_selected, ret);
    return ret;
}

void pwm_set_ocp8135b_config( struct pwm_device *pwm, int duty_cycle)
{
    int ret = 0;
    unsigned int duty_ns, period_ns;
    struct pwm_state state;

    pwm_get_state(pwm, &state);
    period_ns = state.period;
    duty_ns = duty_cycle * period_ns / 100;
    state.duty_cycle = duty_ns;

    if (duty_ns > 0) {
        state.enabled= true ;
    } else{
        state.enabled= false ;
    }
    ret = pwm_apply_state(pwm, &state);
    pr_info("pwm_apply_state duty_cycle: %d ret: %d",duty_cycle, ret);
}

static int sprd_flash_ocp8135b_deinit(void *drvd)
{
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    if (!drv_data)
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    pr_info("E\n");
    gpio_id = drv_data->gpio_63_ENF;
    if (gpio_is_valid(gpio_id)) {
        ret = gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
        if (ret)
            goto exit;
    }

    pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);

exit:
    pr_info("X\n");
    return ret;
}

static int sprd_flash_ocp8135b_open_torch(void *drvd, uint8_t idx)
{
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int gpio_id = 0;
    int ret = 0;

    if (IS_ERR(drv_data))
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    idx |= SPRD_FLASH_LED0;
    pr_info("E\n");
    __pm_stay_awake(&wake_lock);
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if(drv_data->is_old_board){
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
            if (gpio_is_valid(gpio_id)){
                gpio_direction_output(gpio_id, SPRD_FLASH_ON);
                udelay(5 * 1000);
            }
        }else{
            if (gpio_is_valid(gpio_id)){
                gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
            }
            if (gpio_is_valid(drv_data->gpio_123_ENM)){
                gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_ON);
                udelay(5 * 1000);
                gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_OFF);
            }
            sprd_pin123_pwm_selected(drv_data, true);
            #ifdef CAMERA_TORCH_LIGHT_DUTY
                pwm_set_ocp8135b_config(drv_data->pwm_chip, CAMERA_TORCH_LIGHT_DUTY);
            #else
                pwm_set_ocp8135b_config(drv_data->pwm_chip, drv_data->torch_led_index);
            #endif
        }
    }
    pr_info("X\n");
    return ret;
}

static int sprd_flash_ocp8135b_close_torch(void *drvd, uint8_t idx)
{
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

    if (!drv_data)
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    pr_info("E");
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if (gpio_is_valid(gpio_id)) {
            gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
        }
        pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
        sprd_pin123_pwm_selected(drv_data, false);
        if (gpio_is_valid(drv_data->gpio_123_ENM)){
            gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_OFF);
        }
        udelay(5 * 1000);
     }
     __pm_relax(&wake_lock);
    pr_info("X");
    return ret;
}

static int sprd_flash_ocp8135b_open_preflash(void *drvd, uint8_t idx)
{
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int gpio_id = 0;
    int ret = 0;

    if (IS_ERR(drv_data))
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    idx |= SPRD_FLASH_LED0;
    pr_info("E\n");
    pr_info("flash_lock begin!!!");
    mutex_lock(&flash_lock);
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if(drv_data->is_old_board){
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
            if (gpio_is_valid(gpio_id)){
                gpio_direction_output(gpio_id, SPRD_FLASH_ON);
                udelay(5 * 1000);
            }
        }else{
            if (gpio_is_valid(gpio_id)){
                gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
            }
            if (gpio_is_valid(drv_data->gpio_123_ENM)){
                gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_ON);
                udelay(5 * 1000);
                gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_OFF);
            }
            sprd_pin123_pwm_selected(drv_data, true);
            pwm_set_ocp8135b_config(drv_data->pwm_chip, drv_data->torch_led_index);
        }
    }
    mutex_unlock(&flash_lock);
    pr_info("flash_unlock success!!!");
    pr_info("X\n");
    return ret;
}

static int sprd_flash_ocp8135b_open_highlight(void *drvd, uint8_t idx)
{
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

    if (!drv_data)
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    pr_info("E");
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if(drv_data->is_old_board){
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
            if (gpio_is_valid(gpio_id)) {
                gpio_direction_output(gpio_id, SPRD_FLASH_ON);
            }
            udelay(1000);
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 100);
        }else{
            if (gpio_is_valid(gpio_id)) {
                gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
            }
            sprd_pin123_pwm_selected(drv_data, true);
            pwm_set_ocp8135b_config(drv_data->pwm_chip, drv_data->torch_led_index);
            udelay(1000);//<2.5ms

            if (gpio_is_valid(gpio_id)) {
                gpio_direction_output(gpio_id, SPRD_FLASH_ON);
            }
        }
    }
    pr_info("X");
    return ret;
}

static int sprd_flash_ocp8135b_close_preflash(void *drvd, uint8_t idx)
{
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;

    if (!drv_data)
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    pr_info("E");
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if (gpio_is_valid(gpio_id)) {
            pr_info("SPRD_FLASH_OFF");
            gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
        }
        pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
        sprd_pin123_pwm_selected(drv_data, false);
        if (gpio_is_valid(drv_data->gpio_123_ENM)){
            gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_OFF);
        }
        udelay(5 * 1000);
     }
    pr_info("X");
    return ret;
}

/**
ENF="0"
ENM="0"
Time(td)>= 5ms
*/

static int sprd_flash_ocp8135b_close_highlight(void *drvd, uint8_t idx)
{
    int ret = 0;
    int gpio_id = 0;
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    if (!drv_data)
        return -EFAULT;

    if (IS_ERR(drv_data->pwm_chip))
        return -EFAULT;

    pr_info("E");
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx) {
        gpio_id = drv_data->gpio_63_ENF;
        if(drv_data->is_old_board){
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
            udelay(100);
            if (gpio_is_valid(gpio_id)) {
                gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
            }
        }else {
            if (gpio_is_valid(gpio_id)) {
                pr_info("SPRD_FLASH_OFF");
                gpio_direction_output(gpio_id, SPRD_FLASH_OFF);
            }
            udelay(100);
            pwm_set_ocp8135b_config(drv_data->pwm_chip, 0);
            sprd_pin123_pwm_selected(drv_data, false);
        }
        udelay(5 * 1000);
    }
    pr_info("X");
    return ret;
}

static int sprd_flash_ocp8135b_cfg_value_torch(void *drvd, uint8_t idx,
    struct sprd_flash_element *element) {
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int level = 0;
    pr_info("idx:%d is_old_board:%d\n", idx, drv_data->is_old_board);
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx){
        level = element->index + 1;
        if (level>= FLASH_MAX_LEVEL){
            level = FLASH_MAX_LEVEL;
        }
        drv_data->torch_led_index = (int) (100 * level/FLASH_MAX_LEVEL);
        pr_info("level:%d,torch_led_index:%d", level, drv_data->torch_led_index);
    }
    return 0;
}

static int sprd_flash_ocp8135b_cfg_value_preflash(void *drvd, uint8_t idx,
                    struct sprd_flash_element *element) {
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int level = 0;
    pr_info("idx:%d is_old_board:%d\n", idx, drv_data->is_old_board);
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx){
        level = element->index + 1;
        if (level >= FLASH_MAX_LEVEL){
            level = FLASH_MAX_LEVEL;
        }
        drv_data->torch_led_index = (int) (100 * level/FLASH_MAX_LEVEL);
        if (drv_data->torch_led_index > CAMERA_PREFLASH_LIGHT_DUTY){
            drv_data->torch_led_index = CAMERA_PREFLASH_LIGHT_DUTY;
        }
        pr_info("level:%d,torch_led_index:%d", level, drv_data->torch_led_index);
    }
    return 0;
}

static int sprd_flash_ocp8135b_cfg_value_highlight(void *drvd, uint8_t idx,
                               struct sprd_flash_element *element) {
    struct flash_driver_data *drv_data = (struct flash_driver_data *)drvd;
    int level = 0;
    pr_info("idx:%d is_old_board:%d\n", idx, drv_data->is_old_board);
    idx |= SPRD_FLASH_LED0;
    if (SPRD_FLASH_LED0 & idx){
        level = element->index + 1;
        if (level >= FLASH_MAX_LEVEL){
            level = FLASH_MAX_LEVEL;
        }
        drv_data->torch_led_index = (int) (100 * level/FLASH_MAX_LEVEL);
        pr_info("level:%d,torch_led_index:%d", level, drv_data->torch_led_index);
    }

    return 0;
}


static const struct sprd_flash_driver_ops flash_gpio_ops = {
    .open_torch = sprd_flash_ocp8135b_open_torch,
    .close_torch = sprd_flash_ocp8135b_close_torch,
    .open_preflash = sprd_flash_ocp8135b_open_preflash,
    .close_preflash = sprd_flash_ocp8135b_close_preflash,
    .open_highlight = sprd_flash_ocp8135b_open_highlight,
    .close_highlight = sprd_flash_ocp8135b_close_highlight,
    .cfg_value_torch = sprd_flash_ocp8135b_cfg_value_torch,
    .cfg_value_preflash = sprd_flash_ocp8135b_cfg_value_preflash,
    .cfg_value_highlight = sprd_flash_ocp8135b_cfg_value_highlight,
	.get_flash_info = sprd_flash_ic_get_flash_info,
};

static const struct of_device_id ocp8135b_flash_of_match_table[] = {
    {.compatible = "sprd,flash-pwm-ocp8135b"},
};

static int sprd_flash_ocp8135b_probe(struct platform_device *pdev)
{
    int ret = 0;
    // int boardid_gpio = 0;
    // int is_old_board[BOARDID_GPIO_MAX];
    // int j;
    u32 gpio_node = 0;
    struct flash_driver_data *drv_data = NULL;
    mutex_init(&flash_lock);
    pr_err("flash_lock init\n");
    pr_err("E\n");
    if (IS_ERR_OR_NULL(pdev))
          return -EINVAL;

    if (!pdev->dev.of_node) {
        pr_err("no device node %s", __func__);
        return -ENODEV;
    }

    ret = of_property_read_u32(pdev->dev.of_node, "flash-ic", &gpio_node);
    if (ret || gpio_node != 8135) {
        pr_err("no ocp8135b flash\n");
        return -ENOMEM;
    }

    drv_data = devm_kzalloc(&pdev->dev, sizeof(*drv_data), GFP_KERNEL);
    if (!drv_data)
        return -ENOMEM;

    pdev->dev.platform_data = (void *)drv_data;

    drv_data->pwm_chip = devm_pwm_get(&pdev->dev,NULL);
    if (IS_ERR(drv_data->pwm_chip)) {
        dev_err(&pdev->dev, "get pwms device failed\n");
        goto exit;
    }
    pr_info("pwm_chip label : %s\n",drv_data->pwm_chip->label);

    // for (j = 0; j < BOARDID_GPIO_MAX; j++) {
    //     boardid_gpio = of_get_named_gpio(pdev->dev.of_node,
    //                             boardId_gpio_names[j], 0);
    //     if (gpio_is_valid(boardid_gpio)) {
    //         ret = devm_gpio_request(&pdev->dev,
    //                                 boardid_gpio, boardId_gpio_names[j]);
    //         if (ret) {
    //             pr_err("board_Id%d gpio err\n",j);
    //         }else{
    //             is_old_board[j] = gpio_get_value(boardid_gpio);
    //             pr_info("board_Id%d is_old_board[%d]:%d\n", j, j, is_old_board[j]);
    //         }
    //     }
    // }

    // if(is_old_board[2]){
    //     drv_data->is_old_board  = 0;
    // }else{
    //     drv_data->is_old_board  = 1;
    // }
    drv_data->is_old_board  = 0;
    pr_info("boardId is_old_board:%d\n",drv_data->is_old_board);
	flash_ic_info.flash_ic_name = "ocp8135b";
    wakeup_source_init(&wake_lock, "flash_pwm");

    drv_data->gpio_63_ENF = of_get_named_gpio(pdev->dev.of_node, FLASH_GPIO_ENF_NAME, 0);
    if (gpio_is_valid(drv_data->gpio_63_ENF)) {
        ret = devm_gpio_request(&pdev->dev,
                    drv_data->gpio_63_ENF, FLASH_GPIO_ENF_NAME);
        pr_err("gpio init gpio %d\n",drv_data->gpio_63_ENF);
        if (ret) {
            pr_err("flash gpio err\n");
            goto exit;
        }
     }

    ret = gpio_direction_output(drv_data->gpio_63_ENF, SPRD_FLASH_OFF);
    if (ret) {
        pr_err("flash gpio output err\n");
        goto exit;
    }

    drv_data->gpio_123_ENM = of_get_named_gpio(pdev->dev.of_node, FLASH_GPIO_ENM_NAME, 0);
    if (gpio_is_valid(drv_data->gpio_123_ENM)) {
        ret = devm_gpio_request(&pdev->dev,
                    drv_data->gpio_123_ENM, FLASH_GPIO_ENM_NAME);
        pr_err("gpio init gpio %d\n",drv_data->gpio_123_ENM);
        if (ret) {
            pr_err("flash gpio err\n");
            goto exit;
        }
    }

    ret = gpio_direction_output(drv_data->gpio_123_ENM, SPRD_FLASH_OFF);
    if (ret) {
        pr_err("flash gpio output err\n");
        goto exit;
    }

    drv_data->pin_gpio_123 = devm_pinctrl_get(&pdev->dev);
    drv_data->pinctrl_state_gpio = pinctrl_lookup_state(drv_data->pin_gpio_123, "gpio_123");
    drv_data->pinctrl_state_pwm = pinctrl_lookup_state(drv_data->pin_gpio_123, "keyout2_123_pwm");

	drv_data->flash_ic_info = &flash_ic_info;
	pr_info("flash_ic_info %s %s\n", flash_ic_info.flash_ic_name, drv_data->flash_ic_info->flash_ic_name);

    ret = sprd_flash_register(&flash_gpio_ops, drv_data, SPRD_FLASH_REAR);
    if (ret)
          goto exit;

exit:
    pr_err("x\n");
    return ret;
}

static int sprd_flash_ocp8135b_remove(struct platform_device *pdev)
{
    int ret = 0;

    ret = sprd_flash_ocp8135b_deinit(pdev->dev.platform_data);
    if (ret)
        pr_err("flash deinit err\n");
    mutex_destroy(&flash_lock);
    wakeup_source_trash(&wake_lock);
    return ret;
}

static const struct platform_device_id ocp8135b_flash_id[] = {
    {"flash_ocp8135b", 0},
    {},
};

static struct platform_driver sprd_flash_ocp8135b_driver = {
    .probe = sprd_flash_ocp8135b_probe,
    .remove = sprd_flash_ocp8135b_remove,
    .driver = {
                .name = FLASH_DRIVER_NAME,
                .owner = THIS_MODULE,
                .of_match_table = of_match_ptr(ocp8135b_flash_of_match_table),
             },
    .id_table = ocp8135b_flash_id,
};

module_platform_driver(sprd_flash_ocp8135b_driver);
MODULE_DESCRIPTION("Spreadtrum OCP8135 Camera Flash Driver");
MODULE_LICENSE("GPL");
