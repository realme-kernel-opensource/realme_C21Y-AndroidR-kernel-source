/*******************************************************************************
** FILE: - Headset_notifier.h
** Description : This program is for Headset_notifier
*******************************************************************************/

#include <linux/notifier.h>
#include <linux/export.h>




extern struct atomic_notifier_head headset_notifier_list;

extern int headset_register_client(struct notifier_block *nb);
extern int headset_unregister_client(struct notifier_block *nb);
extern int headset_notifier_call_chain(unsigned long val, void *v);

