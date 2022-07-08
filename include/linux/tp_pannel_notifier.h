/*******************************************************************************
** FILE: - Tp_suspend_notifier.h
** Description : This program is for Tp_suspend_notifier
*******************************************************************************/

#include <linux/notifier.h>
#include <linux/export.h>

enum TOUCHSCREEN_SLEEP_STATUS{
    TOUCHSCREEN_SUSPEND = 0,
    TOUCHSCREEN_RESUME = 1
};

extern struct atomic_notifier_head tp_pannel_notifier_list;

extern int tp_pannel_register_client(struct notifier_block *nb);
extern int tp_pannel_unregister_client(struct notifier_block *nb);
extern int tp_pannel_notifier_call_chain(unsigned long val, void *v);

