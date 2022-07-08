/*
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 */

#ifdef BSP_SIL_PLAT_SPRD

#ifndef __SILEAD_FP_SPRD__
#define __SILEAD_FP_SPRD__

#include <linux/of_irq.h>

static irqreturn_t silfp_irq_handler(int irq, void *dev_id);
static void silfp_work_func(struct work_struct *work);
static int silfp_input_init(struct silfp_data *fp_dev);

/* -------------------------------------------------------------------- */
/*                            power supply                              */
/* -------------------------------------------------------------------- */
static void silfp_hw_poweron(struct silfp_data *fp_dev)
{
    int err = 0;
    //LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    /* Power control by Regulators(LDO) */
    if ( fp_dev->avdd_ldo ) {
        err = regulator_set_voltage(fp_dev->avdd_ldo, AVDD_MIN, AVDD_MAX);	/*set 2.8v*/
        err = regulator_enable(fp_dev->avdd_ldo);	/*enable regulator*/
        //pmic_set_register_value(PMIC_RG_VCAMA_CAL,0x0A);
    }
    if ( fp_dev->vddio_ldo ) {
        err = regulator_set_voltage(fp_dev->vddio_ldo, VDDIO_MIN, VDDIO_MAX);	/*set 1.8v*/
        err = regulator_enable(fp_dev->vddio_ldo);	/*enable regulator*/
        //pmic_set_register_value(PMIC_RG_VCAMA_CAL,0x0A);
    }
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    /* Power control by GPIOs */
    if ( fp_dev->pin.pins_avdd_h ) {
        err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_avdd_h);
    }
    if ( fp_dev->pin.pins_vddio_h ) {
        err = pinctrl_select_state(fp_dev->pin.pinctrl, fp_dev->pin.pins_vddio_h);
    }
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    if ( fp_dev->avdd_port >= 0 ) {
        err = gpio_direction_output(fp_dev->avdd_port, 1);
    }
    if ( fp_dev->vddio_port >= 0 ) {
        err = gpio_direction_output(fp_dev->vddio_port, 1);
    }
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
    fp_dev->power_is_off = 0;
    LOG_MSG_DEBUG(INFO_LOG, "%s: power supply ret:%d \n", __func__, err);
}

static void silfp_hw_poweroff(struct silfp_data *fp_dev)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    /* Power control by Regulators(LDO) */
    if ( fp_dev->avdd_ldo && (regulator_is_enabled(fp_dev->avdd_ldo) > 0)) {
        regulator_disable(fp_dev->avdd_ldo);    /*disable regulator*/
    }
    if ( fp_dev->vddio_ldo && (regulator_is_enabled(fp_dev->vddio_ldo) > 0)) {
        regulator_disable(fp_dev->vddio_ldo);   /*disable regulator*/
    }
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    /* Power control by GPIOs */
    //fp_dev->pin.pins_avdd_h = NULL;
    //fp_dev->pin.pins_vddio_h = NULL;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    if ( fp_dev->avdd_port >= 0 ) {
        gpio_direction_output(fp_dev->avdd_port, 0);
    }
    if ( fp_dev->vddio_port >= 0 ) {
        gpio_direction_output(fp_dev->vddio_port, 0);
    }
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
    fp_dev->power_is_off = 1;
}

static void silfp_power_deinit(struct silfp_data *fp_dev)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter.\n", __func__);
#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    /* Power control by Regulators(LDO) */
    if ( fp_dev->avdd_ldo ) {
        regulator_disable(fp_dev->avdd_ldo);	/*disable regulator*/
        regulator_put(fp_dev->avdd_ldo);
        fp_dev->avdd_ldo = NULL;
    }
    if ( fp_dev->vddio_ldo ) {
        regulator_disable(fp_dev->vddio_ldo);	/*disable regulator*/
        regulator_put(fp_dev->vddio_ldo);
        fp_dev->vddio_ldo = NULL;
    }
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    /* Power control by GPIOs */
    fp_dev->pin.pins_avdd_h = NULL;
    fp_dev->pin.pins_vddio_h = NULL;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    if ( fp_dev->avdd_port >= 0 ) {
        gpio_direction_output(fp_dev->avdd_port, 0);
        gpio_free(fp_dev->avdd_port);
        fp_dev->avdd_port = -1;
    }
    if ( fp_dev->vddio_port >= 0 ) {
        gpio_direction_output(fp_dev->vddio_port, 0);
        gpio_free(fp_dev->vddio_port);
        fp_dev->vddio_port = -1;
    }
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
}

/* -------------------------------------------------------------------- */
/*                            hardware reset                            */
/* -------------------------------------------------------------------- */
static void silfp_hw_reset(struct silfp_data *fp_dev, u8 delay)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d\n", __func__, fp_dev->rst_port);

    if ( fp_dev->rst_port >= 0 ) {
        gpio_direction_output(fp_dev->rst_port, 0);
        if (fp_dev->irq_no_use && fp_dev->int_port >= 0) {
            gpio_direction_output(fp_dev->int_port, 1);
        }
        mdelay((delay?delay:5)*RESET_TIME_MULTIPLE);
        gpio_direction_output(fp_dev->rst_port, 1);
        if (fp_dev->irq_no_use && fp_dev->int_port >= 0) {
            gpio_direction_output(fp_dev->int_port, 0);
        }

        mdelay((delay?delay:3)*RESET_TIME_MULTIPLE);
    }
}

/* -------------------------------------------------------------------- */
/*                            power  down                               */
/* -------------------------------------------------------------------- */
static void silfp_pwdn(struct silfp_data *fp_dev, u8 flag_avdd)
{
    LOG_MSG_DEBUG(INFO_LOG, "[%s] enter, port=%d\n", __func__, fp_dev->rst_port);

    if (SIFP_PWDN_FLASH == flag_avdd) {
        silfp_hw_poweroff(fp_dev);
        msleep(200*RESET_TIME_MULTIPLE);
        silfp_hw_poweron(fp_dev);
    }

    if ( fp_dev->rst_port >= 0 ) {
        gpio_direction_output(fp_dev->rst_port, 0);
        if (fp_dev->irq_no_use && fp_dev->int_port >= 0) {
            gpio_direction_output(fp_dev->int_port, 1);
        }
    }

    if (SIFP_PWDN_POWEROFF == flag_avdd) {
        silfp_hw_poweroff(fp_dev);
    }
}

/* -------------------------------------------------------------------- */
/*                         init/deinit functions                        */
/* -------------------------------------------------------------------- */
static int silfp_parse_dts(struct silfp_data* fp_dev)
{
    int ret;

    /* Get power settings */
#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    fp_dev->pin.pins_avdd_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "avdd-enable");
    if (IS_ERR_OR_NULL(fp_dev->pin.pins_avdd_h)) {
        fp_dev->pin.pins_avdd_h = NULL;
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp avdd-enable\n", __func__);
        // Ignore error
    }

    fp_dev->pin.pins_vddio_h = pinctrl_lookup_state(fp_dev->pin.pinctrl, "vddio-enable");
    if (IS_ERR_OR_NULL(fp_dev->pin.pins_vddio_h)) {
        fp_dev->pin.pins_vddio_h = NULL;
        LOG_MSG_DEBUG(ERR_LOG, "%s can't find silfp vddio-enable\n", __func__);
        // Ignore error
    }
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */

#ifdef BSP_SIL_POWER_SUPPLY_REGULATOR
    // Todo: use correct settings.
    fp_dev->avdd_ldo = regulator_get(&fp_dev->spi->dev, "avdd");
    fp_dev->vddio_ldo= regulator_get(&fp_dev->spi->dev, "vddio");
#endif /* BSP_SIL_POWER_SUPPLY_REGULATOR */

    /* Get the SPI Max speed */
    ret = of_property_read_u32(fp_dev->spi->dev.of_node,"spi-max-frequency", &fp_dev->pin.max_speed_hz);
    if (ret) {
        fp_dev->pin.max_speed_hz = 0;
        LOG_MSG_DEBUG(ERR_LOG, "Error getting spi max speed\n");
    }

    /* Get the QUP ID (#1-12) */
    ret = of_property_read_u32(fp_dev->spi->dev.of_node,"sprd,spi-id", &fp_dev->pin.qup_id);
    if (ret) {
        fp_dev->pin.spi_id = 0;
        LOG_MSG_DEBUG(ERR_LOG, "Error getting qup_id\n");
    }

    LOG_MSG_DEBUG(INFO_LOG, "[%s] done (%d).\n",__func__,ret);
    return ret;
}


static int silfp_set_spi(struct silfp_data *fp_dev, bool enable)
{
    int ret = -ENOENT;
    LOG_MSG_DEBUG(ERR_LOG, "%s: sprd no needed!\n", __func__);
    return ret;
}

static int silfp_irq_to_reset_init(struct silfp_data *fp_dev)
{
    int ret = 0;

    silfp_irq_disable(fp_dev);
    free_irq(fp_dev->irq, fp_dev);

    if (fp_dev->int_port >= 0) {
        gpio_free(fp_dev->int_port);
        ret = gpio_request(fp_dev->int_port, "SILFP_IRQ_TO_RST_PIN");
        if (ret < 0) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->int_port, ret);
            return -ENODEV;
        } else {
            gpio_direction_output(fp_dev->int_port, 0);
        }
    }

    fp_dev->irq_no_use = 1;

    return ret;
}

static int silfp_set_feature(struct silfp_data *fp_dev, u8 feature)
{
    int ret = 0;

    switch (feature) {
    case FEATURE_FLASH_CS:
        LOG_MSG_DEBUG(INFO_LOG, "%s set feature flash cs\n", __func__);
        ret = silfp_irq_to_reset_init(fp_dev);
        break;

    default:
        break;
    }
    return ret;
}

static int silfp_resource_init(struct silfp_data *fp_dev, struct fp_dev_init_t *dev_info)
{
    int status = 0;
    int ret;

    if (atomic_read(&fp_dev->init)) {
        atomic_inc(&fp_dev->init);
        LOG_MSG_DEBUG(DBG_LOG, "[%s] dev already init(%d).\n",__func__,atomic_read(&fp_dev->init));
        return status;
    }

    fp_dev->int_port = -1;
    fp_dev->rst_port = -1;
    silfp_parse_dts(fp_dev);
#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    fp_dev->avdd_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "avdd-gpios", 0);
    fp_dev->vddio_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "vddio-gpios", 0);
    if (fp_dev->avdd_port >= 0 ) {
        gpio_free(fp_dev->avdd_port);
        ret = gpio_request(fp_dev->avdd_port, "SILFP_AVDD_PIN");
        if (ret < 0) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->avdd_port, ret);
            status = -ENODEV;
            goto err_avdd;
        }
    }

    if (fp_dev->vddio_port >= 0 ) {
        gpio_free(fp_dev->vddio_port);
        ret = gpio_request(fp_dev->vddio_port, "SILFP_VDDIO_PIN");
        if (ret < 0) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->vddio_port, ret);
            status = -ENODEV;
            goto err_vddio;
        }
    }
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */
    silfp_hw_poweron(fp_dev);
    fp_dev->int_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "irq-gpios", 0);
    fp_dev->rst_port = of_get_named_gpio(fp_dev->spi->dev.of_node, "rst-gpios", 0);
    LOG_MSG_DEBUG(INFO_LOG, "[%s] int_port %d, rst_port %d.\n",__func__,fp_dev->int_port,fp_dev->rst_port);
    if (fp_dev->int_port >= 0 ) {
        gpio_free(fp_dev->int_port);
    }

    if (fp_dev->rst_port >= 0 ) {
        gpio_free(fp_dev->rst_port);
    }

    ret = gpio_request(fp_dev->int_port, "SILFP_INT_IRQ");
    if (ret < 0) {
        LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->int_port, ret);
        status = -ENODEV;
        goto err;
    } else {
        gpio_direction_input(fp_dev->int_port);
        fp_dev->irq = gpio_to_irq(fp_dev->int_port);
        fp_dev->irq_is_disable = 0;

        ret  = request_irq(fp_dev->irq,
                           silfp_irq_handler,
                           IRQ_TYPE_EDGE_RISING, //IRQ_TYPE_LEVEL_HIGH, //irq_table[ts->int_trigger_type],
                           "silfp",
                           fp_dev);
        if ( ret < 0 ) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Filed to request_irq (%d), ert=%d",__func__,fp_dev->irq, ret);
            status = -ENODEV;
            goto err_irq;
        } else {
            LOG_MSG_DEBUG(INFO_LOG,"[%s] Enable_irq_wake.\n",__func__);
            enable_irq_wake(fp_dev->irq);
            silfp_irq_disable(fp_dev);
        }
    }

    if (fp_dev->rst_port >= 0 ) {
        ret = gpio_request(fp_dev->rst_port, "SILFP_RST_PIN");
        if (ret < 0) {
            LOG_MSG_DEBUG(ERR_LOG, "[%s] Failed to request GPIO=%d, ret=%d",__func__,(s32)fp_dev->rst_port, ret);
            status = -ENODEV;
            goto err_rst;
        } else {
            gpio_direction_output(fp_dev->rst_port, 1);
        }
    }

    if (!ret) {
        if (silfp_input_init(fp_dev)) {
            goto err_input;
        }
        atomic_set(&fp_dev->init,1);
    }
    fp_dev->irq_no_use = 0;
    dev_info->reserve = PKG_SIZE;
    dev_info->reserve <<= 12;
    if (fp_dev->pin.max_speed_hz) {
        dev_info->speed = fp_dev->pin.max_speed_hz;
    }

    if (dev_info && fp_dev->pin.spi_id && (fp_dev->pin.spi_id <= 32)) {
        dev_info->dev_id = fp_dev->pin.spi_id;
        strncpy(dev_info->ta,TANAME,sizeof(dev_info->ta));
    }

    return status;

err_input:
    if (fp_dev->rst_port >= 0 ) {
        gpio_free(fp_dev->rst_port);
    }

err_rst:
    free_irq(fp_dev->irq, fp_dev);
    gpio_direction_input(fp_dev->int_port);

err_irq:
    gpio_free(fp_dev->int_port);

#ifdef BSP_SIL_POWER_SUPPLY_GPIO
    gpio_free(fp_dev->vddio_port);
err_vddio:
    gpio_free(fp_dev->avdd_port);

err_avdd:
    fp_dev->avdd_port  = -1;
    fp_dev->vddio_port = -1;
#endif /* BSP_SIL_POWER_SUPPLY_GPIO */

err:
    fp_dev->int_port = -1;
    fp_dev->rst_port = -1;

    return status;
}

#endif /* __SILEAD_FP_SPRD__ */

#endif /* BSP_SIL_PLAT_SPRD */

/* End of file spilead_fp_sprd.c */
