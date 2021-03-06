#ifndef _DT_BINDINGS_PINCTRL_SPRD_H
#define _DT_BINDINGS_PINCTRL_SPRD_H

#define AP_SLEEP		(1 << 0)
#define PUBCP_SLEEP		(1 << 1)
#define TGLDSP_SLEEP		(1 << 2)
#define AGDSP_SLEEP		(1 << 3)
#define CM4_SLEEP		(1 << 4)
#define SPRD_PIN_NO(x) 		((x) << 8)
#define PINMUX_GPIO61__FUNC_GPIO61 (SPRD_PIN_NO(61) | 0)
#endif
