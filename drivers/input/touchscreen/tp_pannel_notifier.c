/*******************************************************************************
** Copyright (C), 2008-2016, OPLUS Mobile Comm Corp., Ltd.
** Description : This program is for iliteck tp ic suspend & resume by lcd notifier
** Version: 1.0
*******************************************************************************/

#include <linux/notifier.h>
#include <linux/tp_pannel_notifier.h>

ATOMIC_NOTIFIER_HEAD(tp_pannel_notifier_list);
//EXPORT_SYMBOL_GPL(tp_suspend_notifier_list);

int tp_pannel_register_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_register(&tp_pannel_notifier_list, nb);

}
EXPORT_SYMBOL(tp_pannel_register_client);

int tp_pannel_unregister_client(struct notifier_block *nb)
{
	return atomic_notifier_chain_unregister(&tp_pannel_notifier_list, nb);
}
EXPORT_SYMBOL(tp_pannel_unregister_client);


int tp_pannel_notifier_call_chain(unsigned long val, void *v)
{
	return atomic_notifier_call_chain(&tp_pannel_notifier_list, val, v);

}
EXPORT_SYMBOL_GPL(tp_pannel_notifier_call_chain);


