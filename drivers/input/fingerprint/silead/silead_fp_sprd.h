/*
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 *
 */

#ifndef __SILEAD_FP_SPRD_H__
#define __SILEAD_FP_SPRD_H__

#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>

struct fp_plat_t {
    u32 spi_id;
    u32 max_speed_hz;
    u32 qup_id;
#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    struct pinctrl_state *pins_avdd_h, *pins_vddio_h;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */
};

#endif /* __SILEAD_FP_SPRD_H__ */

/* End of file silead_fp_sprd.h */
