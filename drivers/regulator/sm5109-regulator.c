/*
 * Regulator driver for SM5109 Bias chip
 *
 */

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define SM5109_REG_VPOS			0x00
#define SM5109_REG_VNEG			0x01
#define SM5109_REG_APPS_DISP_DISN	0x03

#define SM5109_VOUT_MASK		0x1F
#define SM5109_VOUT_N_VOLTAGE		0x15
#define SM5109_VOUT_VMIN		4000
#define SM5109_VOUT_VMAX		6000
#define SM5109_VOUT_STEP		100

struct sm5109_data {
	struct device *dev;
	struct i2c_client *i2c;
};

static struct sm5109_data *sm5109;

/* set voltage in mV */
void sm5109_set_voltage(unsigned int vol)
{
	int ret;
	u8 val;

	if (vol > SM5109_VOUT_VMAX) {
		dev_err(sm5109->dev, "voltage out of range\n");
		return;
	}
	val = ((vol - SM5109_VOUT_VMIN) / SM5109_VOUT_STEP) & SM5109_VOUT_MASK;

	ret = i2c_smbus_write_byte_data(sm5109->i2c, SM5109_REG_VPOS, val);
	if (ret < 0)
		dev_err(sm5109->dev, "write reg [0x%02x, %u] failed\n", SM5109_REG_VPOS, val);

	ret = i2c_smbus_write_byte_data(sm5109->i2c, SM5109_REG_VNEG, val);
	if (ret < 0)
		dev_err(sm5109->dev, "write reg [0x%02x, %u] failed\n", SM5109_REG_VNEG, val);

}
EXPORT_SYMBOL_GPL(sm5109_set_voltage);

static int sm5109_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	sm5109 = devm_kzalloc(&i2c->dev, sizeof(*sm5109), GFP_KERNEL);
	if (!sm5109)
		return -ENOMEM;

	dev_err(&i2c->dev, "%s: start\n", __func__);

	sm5109->i2c = i2c;
	sm5109->dev = &i2c->dev;

	i2c_set_clientdata(i2c, sm5109);

	return 0;
}

static const struct i2c_device_id sm5109_i2c_id[] = {
	{ "sm5109", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5109_i2c_id);

static struct i2c_driver sm5109_i2c_driver = {
	.driver = {
		.name = "SM5109",
	},
	.probe    = sm5109_i2c_probe,
	.id_table = sm5109_i2c_id,
};

module_i2c_driver(sm5109_i2c_driver);

MODULE_LICENSE("GPL");
