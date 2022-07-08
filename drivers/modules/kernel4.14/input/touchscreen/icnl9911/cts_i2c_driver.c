#define LOG_TAG         "I2CDrv"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_sysfs.h"
#include <linux/power_supply.h>


bool cts_show_debug_log = false;
module_param_named(debug_log, cts_show_debug_log, bool, 0660);
MODULE_PARM_DESC(debug_log, "Show debug log control");

int cts_suspend(struct chipone_ts_data *cts_data)
{
    int ret;
    cts_info("Suspend");
	cts_data->suspend=true;

    cts_lock_device(&cts_data->cts_dev);
    ret = cts_suspend_device(&cts_data->cts_dev);
    cts_unlock_device(&cts_data->cts_dev);

    if (ret) {
        cts_err("Suspend device failed %d", ret);
        // TODO:
        //return ret;
    }

    ret = cts_stop_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Stop device failed %d", ret);
        return ret;
    }

#ifdef CFG_CTS_GESTURE
    /* Enable IRQ wake if gesture wakeup enabled */
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_enable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_err("Enable IRQ wake failed %d", ret);
            return ret;
        }
        ret = cts_plat_enable_irq(cts_data->pdata);
        if (ret){
            cts_err("Enable IRQ failed %d",ret);
            return ret;
        }
    }
#endif /* CFG_CTS_GESTURE */

    /** - To avoid waking up while not sleeping,
            delay 20ms to ensure reliability */
    msleep(20);

    return 0;
}

int cts_resume(struct chipone_ts_data *cts_data)
{
    int ret;
    cts_info("Resume");
	cts_data->suspend=false;
#ifdef CFG_CTS_GESTURE
    if (cts_is_gesture_wakeup_enabled(&cts_data->cts_dev)) {
        ret = cts_plat_disable_irq_wake(cts_data->pdata);
        if (ret) {
            cts_warn("Disable IRQ wake failed %d", ret);
            //return ret;
        }
        if ((ret = cts_plat_disable_irq(cts_data->pdata)) < 0) {
            cts_err("Disable IRQ failed %d", ret);
            //return ret;
        }
    }
#endif /* CFG_CTS_GESTURE */

    ret = cts_resume_device(&cts_data->cts_dev);
    if(ret) {
        cts_warn("Resume device failed %d", ret);
        return ret;
    }

    ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        return ret;
    }

    return 0;
}

#ifdef CONFIG_CTS_PM_FB_NOTIFIER
static int fb_notifier_callback(struct notifier_block *nb,
				      unsigned long action, void *data)
{
	const struct cts_platform_data *pdata = container_of(nb, struct cts_platform_data, fb_notifier);
	struct chipone_ts_data *cts_data =container_of(pdata->cts_dev, struct chipone_ts_data, cts_dev);

	struct adf_notifier_event *event = data;
	int adf_event_data;

	if (action != ADF_EVENT_BLANK)
		return NOTIFY_DONE;

	adf_event_data = *(int *)event->data;
	cts_info("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		cts_resume(cts_data);
		break;
	case DRM_MODE_DPMS_OFF:
		cts_suspend(cts_data);
		break;
	default:
		cts_info("receive adf event with error data, adf_event_data=%d",
			adf_event_data);
		break;
	}

	return NOTIFY_OK;
}

static int cts_init_pm_fb_notifier(struct chipone_ts_data * cts_data)
{
    cts_info("Init FB notifier");


    cts_data->pdata->fb_notifier.notifier_call = fb_notifier_callback;
    cts_data->pdata->fb_notifier.priority = 1000;

    return adf_register_client(&cts_data->pdata->fb_notifier);}	

static int cts_deinit_pm_fb_notifier(struct chipone_ts_data * cts_data)
{
    cts_info("Deinit FB notifier");

    return adf_unregister_client(&cts_data->pdata->fb_notifier); 
}	
#endif /* CONFIG_CTS_PM_FB_NOTIFIER */
/*
extern int tp_device_id(int id);
extern void ctp_lock_mutex(void);
extern void ctp_unlock_mutex(void);

static int check_ctp_chip(void)
{
	ctp_lock_mutex();
	tp_device_id(0x9911);
	ctp_unlock_mutex();
	return 0;
}

static int remove_ctp_chip(void)
{
	ctp_lock_mutex();
	tp_device_id(0xFFFF);
	ctp_unlock_mutex();
	return 0;
}
*/

#ifdef CONFIG_CTS_I2C_HOST
static int cts_driver_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
#else        
static int cts_driver_probe(struct spi_device *client)
#endif
{
    struct chipone_ts_data *cts_data = NULL;
    int ret = 0;

/*	if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}
*/
#ifdef CONFIG_CTS_I2C_HOST
    cts_info("Probe i2c client: name='%s' addr=0x%02x flags=0x%02x irq=%d",
        client->name, client->addr, client->flags, client->irq);

#if !defined(CONFIG_MTK_PLATFORM)
    if (client->addr != CTS_DEV_NORMAL_MODE_I2CADDR) {
        cts_err("Probe i2c addr 0x%02x != driver config addr 0x%02x",
            client->addr, CTS_DEV_NORMAL_MODE_I2CADDR);
        return -ENODEV;
    };
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        cts_err("Check functionality failed");
        return -ENODEV;
    }
#endif

    cts_data = (struct chipone_ts_data *)kzalloc(sizeof(*cts_data), GFP_KERNEL);
    if (cts_data == NULL) {
        cts_err("Allocate chipone_ts_data failed");
        return -ENOMEM;
    }

    cts_data->pdata = (struct cts_platform_data *)kzalloc(
            sizeof(struct cts_platform_data), GFP_KERNEL);
    if (cts_data->pdata == NULL) {
        cts_err("Allocate cts_platform_data failed");
        ret = -ENOMEM;
        goto err_free_cts_data;
    }

#ifdef CONFIG_CTS_I2C_HOST
    i2c_set_clientdata(client, cts_data);
    cts_data->i2c_client = client;
#else
	spi_set_drvdata(client, cts_data);
	cts_data->spi_client = client;
#endif

    cts_init_platform_data(cts_data->pdata, client);

    cts_data->cts_dev.pdata = cts_data->pdata;
    cts_data->pdata->cts_dev = &cts_data->cts_dev;

    cts_data->workqueue = create_singlethread_workqueue(CFG_CTS_DEVICE_NAME "-workqueue");
    if (cts_data->workqueue == NULL) {
        cts_err("Create workqueue failed");
        ret = -ENOMEM;
        goto err_deinit_platform_data;
    }

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_data->esd_workqueue = create_singlethread_workqueue(CFG_CTS_DEVICE_NAME "-esd_workqueue");
    if (cts_data->esd_workqueue == NULL) {
        cts_err("Create esd workqueue failed");
        ret = -ENOMEM;
        goto err_destroy_workqueue;
    }
#endif
    ret = cts_plat_request_resource(cts_data->pdata);
    if (ret < 0) {
        cts_err("Request resource failed %d", ret);
        goto err_destroy_esd_workqueue;
    }

    ret = cts_plat_reset_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Reset device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_probe_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Probe device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_plat_init_touch_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init touch device failed %d", ret);
        goto err_free_resource;
    }

    ret = cts_plat_init_vkey_device(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init vkey device failed %d", ret);
        goto err_deinit_touch_device;
    }
    
    ret = cts_plat_init_gesture(cts_data->pdata);
    if (ret < 0) {
        cts_err("Init gesture failed %d", ret);
        goto err_deinit_vkey_device;
    }

    cts_init_esd_protection(cts_data);

    ret = cts_tool_init(cts_data);
    if (ret < 0) {
        cts_warn("Init tool node failed %d", ret);
    }

    ret = cts_sysfs_add_device(&client->dev);
    if (ret < 0) {
        cts_warn("Add sysfs entry for device failed %d", ret);
    }
	ret = sysfs_create_link(NULL, &client->dev.kobj, "touchscreen");
	if (ret < 0) {
		cts_warn("Failed to create link! %d",ret);
		return -ENOMEM;
	}

#ifdef CONFIG_CTS_PM_FB_NOTIFIER
    ret = cts_init_pm_fb_notifier(cts_data);
    if (ret) {
        cts_err("Init FB notifier failed %d", ret);
        goto err_deinit_sysfs;
    }   
#endif /* CONFIG_CTS_PM_FB_NOTIFIER */

    ret = cts_plat_request_irq(cts_data->pdata);
    if (ret < 0) {
        cts_err("Request IRQ failed %d", ret);
        goto err_register_fb;
    }

    ret = cts_start_device(&cts_data->cts_dev);
    if (ret) {
        cts_err("Start device failed %d", ret);
        goto err_free_irq;
    }
	//check_ctp_chip();
	{
	//extern void zyt_info_s2(char* s1,char* s2);
	//zyt_info_s2("[TP] : ","ICNL9911");
    }
	cts_info("cts_driver_probe success");
	
    return 0;
    
err_free_irq:
    cts_plat_free_irq(cts_data->pdata);

err_register_fb:
#ifdef CONFIG_CTS_PM_FB_NOTIFIER
    cts_deinit_pm_fb_notifier(cts_data);
err_deinit_sysfs:
#endif /* CONFIG_CTS_PM_FB_NOTIFIER */
    cts_sysfs_remove_device(&client->dev);
#ifdef CONFIG_CTS_LEGACY_TOOL
    cts_tool_deinit(cts_data);
#endif /* CONFIG_CTS_LEGACY_TOOL */

#ifdef CONFIG_CTS_ESD_PROTECTION
    cts_deinit_esd_protection(cts_data);
#endif /* CONFIG_CTS_ESD_PROTECTION */

#ifdef CFG_CTS_GESTURE
    cts_plat_deinit_gesture(cts_data->pdata);
#endif /* CFG_CTS_GESTURE */

err_deinit_vkey_device:
#ifdef CONFIG_CTS_VIRTUALKEY
    cts_plat_deinit_vkey_device(cts_data->pdata);
#endif /* CONFIG_CTS_VIRTUALKEY */

err_deinit_touch_device:
    cts_plat_deinit_touch_device(cts_data->pdata);

err_free_resource:
    cts_plat_free_resource(cts_data->pdata);
err_destroy_esd_workqueue:
#ifdef CONFIG_CTS_ESD_PROTECTION
    destroy_workqueue(cts_data->esd_workqueue);
err_destroy_workqueue:
#endif
    destroy_workqueue(cts_data->workqueue);
err_deinit_platform_data:
    cts_deinit_platform_data(cts_data->pdata);   
//err_free_pdata:
    kfree(cts_data->pdata);
err_free_cts_data:
    kfree(cts_data);
	//remove_ctp_chip();
	
    cts_err("Probe failed %d", ret);

    return ret;
}

#ifdef CONFIG_CTS_I2C_HOST
static int cts_driver_remove(struct i2c_client *client)
#else
static int cts_driver_remove(struct spi_device *client)
#endif
{
    struct chipone_ts_data *cts_data;
    int ret = 0;

    cts_info("Remove");

	//remove_ctp_chip();
	
#ifdef CONFIG_CTS_I2C_HOST
    cts_data = (struct chipone_ts_data *)i2c_get_clientdata(client);
#else
	cts_data = (struct chipone_ts_data *)spi_get_drvdata(client);
#endif
    if (cts_data) {
        ret = cts_stop_device(&cts_data->cts_dev);
        if (ret) {
            cts_warn("Stop device failed %d", ret);
        }

        cts_plat_free_irq(cts_data->pdata);

#ifdef CONFIG_CTS_PM_FB_NOTIFIER       
        cts_deinit_pm_fb_notifier(cts_data);
#endif /* CONFIG_CTS_PM_FB_NOTIFIER */

        cts_tool_deinit(cts_data);

        cts_sysfs_remove_device(&client->dev);

        cts_deinit_esd_protection(cts_data);

        cts_plat_deinit_touch_device(cts_data->pdata);

        cts_plat_deinit_vkey_device(cts_data->pdata);

        cts_plat_deinit_gesture(cts_data->pdata);

        cts_plat_free_resource(cts_data->pdata);

#ifdef CONFIG_CTS_ESD_PROTECTION
        if (cts_data->esd_workqueue) {
            destroy_workqueue(cts_data->esd_workqueue);
        }
#endif

        if (cts_data->workqueue) {
            destroy_workqueue(cts_data->workqueue);
        }

        cts_deinit_platform_data(cts_data->pdata);

        if (cts_data->pdata) {
            kfree(cts_data->pdata);
        }else {
		pr_info("Failed in cts_driver_remove.\n");
		kfree(cts_data);
		return -ENOMEM;
	}
        kfree(cts_data);
    }else {
        cts_warn("Chipone i2c driver remove while NULL chipone_ts_data");
        return -EINVAL;
    }

    return ret;
}

#ifdef CONFIG_CTS_PM_LEGACY
static int cts_i2c_driver_suspend(struct device *dev, pm_message_t state)
{
	cts_info("Suspend by legacy power management");
	return cts_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_resume(struct device *dev)
{
	cts_info("Resume by legacy power management");
	return cts_resume(dev_get_drvdata(dev));
}
#endif /* CONFIG_CTS_PM_LEGACY */

#ifdef CONFIG_CTS_PM_GENERIC
static int cts_i2c_driver_pm_suspend(struct device *dev)
{
	cts_info("Suspend by bus power management");
	return cts_suspend(dev_get_drvdata(dev));
}

static int cts_i2c_driver_pm_resume(struct device *dev)
{
	cts_info("Resume by bus power management");
	return cts_resume(dev_get_drvdata(dev));
}

/* bus control the suspend/resume procedure */
static const struct dev_pm_ops cts_i2c_driver_pm_ops = {
	.suspend = cts_i2c_driver_pm_suspend,
	.resume = cts_i2c_driver_pm_resume,
};
#endif /* CONFIG_CTS_PM_GENERIC */

#ifdef CONFIG_CTS_SYSFS


	

static ssize_t reset_pin_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_HAS_RESET_PIN
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_reset_pin = {
		.attr = {
			.name = "reset_pin",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = reset_pin_show,
		.store = NULL,
};

static ssize_t swap_xy_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_SWAP_XY: %c\n",
#ifdef CFG_CTS_SWAP_XY
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_swap_xy = {
		.attr = {
			.name = "swap_xy",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = swap_xy_show,
		.store = NULL,
};


static ssize_t wrap_x_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_WRAP_X: %c\n",
#ifdef CFG_CTS_WRAP_X
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_wrap_x = {
		.attr = {
			.name = "wrap_x",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = wrap_x_show,
		.store = NULL,
};


static ssize_t wrap_y_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_WRAP_Y: %c\n",
#ifdef CFG_CTS_WRAP_Y
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_wrap_y = {
		.attr = {
			.name = "wrap_y",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = wrap_y_show,
		.store = NULL,
};


static ssize_t force_update_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_HAS_RESET_PIN: %c\n",
#ifdef CFG_CTS_FIRMWARE_FORCE_UPDATE
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_force_update = {
		.attr = {
			.name = "force_update",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = force_update_show,
		.store = NULL,
};


static ssize_t max_touch_num_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_MAX_TOUCH_NUM: %d\n",
        CFG_CTS_MAX_TOUCH_NUM);
}

static struct device_attribute driver_attr_max_touch_num = {
		.attr = {
			.name = "max_touch_num",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = max_touch_num_show,
		.store = NULL,
};


static ssize_t vkey_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_VIRTUALKEY: %c\n",
#ifdef CONFIG_CTS_VIRTUALKEY
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_vkey = {
		.attr = {
			.name = "vkey",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = vkey_show,
		.store = NULL,
};

static ssize_t gesture_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CFG_CTS_GESTURE: %c\n",
#ifdef CFG_CTS_GESTURE
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_gesture = {
		.attr = {
			.name = "gesture",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = gesture_show,
		.store = NULL,
};



static ssize_t esd_protection_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_ESD_PROTECTION: %c\n",
#ifdef CONFIG_CTS_ESD_PROTECTION
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_esd_protection = {
		.attr = {
			.name = "esd_protection",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = esd_protection_show,
		.store = NULL,
};

static ssize_t slot_protocol_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "CONFIG_CTS_SLOTPROTOCOL: %c\n",
#ifdef CONFIG_CTS_SLOTPROTOCOL
        'Y'
#else
        'N'
#endif
	    );
}

static struct device_attribute driver_attr_slot_protocol = {
		.attr = {
			.name = "slot_protocol",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = slot_protocol_show,
		.store = NULL,
};

static ssize_t max_xfer_size_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_CTS_I2C_HOST	
	return sprintf(buf, "CFG_CTS_MAX_I2C_XFER_SIZE: %d\n",
        CFG_CTS_MAX_I2C_XFER_SIZE);
#else
	return sprintf(buf, "CFG_CTS_MAX_SPI_XFER_SIZE: %d\n",
        CFG_CTS_MAX_SPI_XFER_SIZE);
#endif        
}

static struct device_attribute driver_attr_max_xfer_size = {
		.attr = {
			.name = "max_xfer_size",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = max_xfer_size_show,
		.store = NULL,
};



static ssize_t driver_info_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Driver version: %s\n", CFG_CTS_DRIVER_VERSION);
}

static struct device_attribute driver_attr_driver_info = {
		.attr = {
			.name = "driver_info",
			.mode = S_IWUSR | S_IRUGO,
		},
		.show = driver_info_show,
		.store = NULL,
};


static struct attribute *cts_i2c_driver_config_attrs[] = {
    &driver_attr_reset_pin.attr,
    &driver_attr_swap_xy.attr,
    &driver_attr_wrap_x.attr,
    &driver_attr_wrap_y.attr,
    &driver_attr_force_update.attr,
	&driver_attr_max_touch_num.attr,
	&driver_attr_vkey.attr,
    &driver_attr_gesture.attr,
	&driver_attr_esd_protection.attr,
    &driver_attr_slot_protocol.attr,
    &driver_attr_max_xfer_size.attr,
    &driver_attr_driver_info.attr,
	NULL
};

static const struct attribute_group cts_i2c_driver_config_group = {
    .name = "config",
	.attrs = cts_i2c_driver_config_attrs,
};

static const struct attribute_group *cts_i2c_driver_config_groups[] = {
    &cts_i2c_driver_config_group,
    NULL,
};
#endif /* CONFIG_CTS_SYSFS */

#ifdef CONFIG_CTS_OF
static const struct of_device_id cts_i2c_of_match_table[] = {
    {.compatible = CFG_CTS_OF_DEVICE_ID_NAME,},
    { },
};
MODULE_DEVICE_TABLE(of, cts_i2c_of_match_table);
#endif /* CONFIG_CTS_OF */

#ifdef CONFIG_CTS_I2C_HOST
static const struct i2c_device_id cts_device_id_table[] = {
    {CFG_CTS_DEVICE_NAME, 0},
    {}
};
#else
static const struct spi_device_id cts_device_id_table[] = {
    {CFG_CTS_DEVICE_NAME, 0},
    {}
};
#endif

#ifdef CONFIG_CTS_I2C_HOST
static struct i2c_driver cts_i2c_driver = {
#else
static struct spi_driver cts_spi_driver = {
#endif
    .probe = cts_driver_probe,
    .remove = cts_driver_remove,
    .driver = {
        .name = CFG_CTS_DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_CTS_OF
        .of_match_table = of_match_ptr(cts_i2c_of_match_table),
#endif /* CONFIG_CTS_OF */
#ifdef CONFIG_CTS_SYSFS
        .groups = cts_i2c_driver_config_groups,
#endif /* CONFIG_CTS_SYSFS */
#ifdef CONFIG_CTS_PM_LEGACY
        .suspend = cts_i2c_driver_suspend,
        .resume  = cts_i2c_driver_resume,
#endif /* CONFIG_CTS_PM_LEGACY */
#ifdef CONFIG_CTS_PM_GENERIC
        .pm = &cts_i2c_driver_pm_ops,
#endif /* CONFIG_CTS_PM_GENERIC */

    },
    .id_table = cts_device_id_table,
};

static int __init cts_driver_init(void)
{
    cts_info("Init");
/*	if(tp_device_id(0)!=0)
	{
		printk("CTP(0x%x)Exist!", tp_device_id(0));
		return -ENODEV;
	}
*/
#ifdef CONFIG_CTS_I2C_HOST
    return i2c_add_driver(&cts_i2c_driver);
#else
	return spi_register_driver(&cts_spi_driver);
#endif
}

static void __exit cts_driver_exit(void)
{
    cts_info("Exit");

#ifdef CONFIG_CTS_I2C_HOST
    i2c_del_driver(&cts_i2c_driver);
#else
	spi_unregister_driver(&cts_spi_driver);
#endif
}

module_init(cts_driver_init);
module_exit(cts_driver_exit);

MODULE_DESCRIPTION("Chipone TDDI touchscreen Driver for QualComm platform");
MODULE_VERSION(CFG_CTS_DRIVER_VERSION);
MODULE_AUTHOR("Miao Defang <dfmiao@chiponeic.com>");
MODULE_LICENSE("GPL");

