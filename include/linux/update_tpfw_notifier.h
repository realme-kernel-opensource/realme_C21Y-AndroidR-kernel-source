/*******************************************************************************
** FILE: - update_tpfw_notifier.h
** Description : This program is for touch firmware update notifier
*******************************************************************************/
 
#include <linux/notifier.h>
#include <linux/export.h>




extern struct atomic_notifier_head tpfw_notifier_list;

extern int update_tpfw_register_client(struct notifier_block *nb);
extern int update_tpfw_unregister_client(struct notifier_block *nb);
extern int update_tpfw_notifier_call_chain(unsigned long val, void *v);

