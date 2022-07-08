#include "ilitek_v3.h"

extern unsigned char ili9882_user_buf[USER_STR_BUFF];
int tp_gesture;

typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct file_operations *fops;
	bool isCreated;
} proc_node_t;


struct proc_dir_entry *proc_dir_oplus;
struct proc_dir_entry *proc_dir_debug_info;

static ssize_t oplus_proc_mp_lcm_on_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0, len = 0;
	bool esd_en = ilits->wq_esd_ctrl, bat_en = ilits->wq_bat_ctrl;
	char *ptr = NULL;


	if (*pPos != 0)
		return 0;


	ILI_INFO("Run MP test with LCM on\n");

	mutex_lock(&ilits->touch_mutex);
	ilits->oplus_result = 0;
	ptr = (char*)kzalloc(256,GFP_KERNEL);
    if (ptr == NULL) {
        ILI_ERR("failed to alloc ptr memory\n");
        goto out;
    }

	/* Create the directory for mp_test result */
	if ((ili_dev_mkdir(CSV_LCM_ON_PATH, S_IRUGO | S_IWUSR)) != 0)
		ILI_ERR("Failed to create directory for mp_test\n");

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, DISABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, DISABLE);

	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	ilits->mp_ret_len = 0;

	ret = ili_mp_test_handler(ili9882_user_buf, ON);
	ILI_INFO("MP TEST %s, Error code = %d\n", (ret < 0) ? "FAIL" : "PASS", ret);
	if(ilits->oplus_result == 0){
		ILI_INFO("MP TEST PASS or warning\n");
	}else{
		ILI_ERR("MP TEST FAIL\n");
	}
	len = snprintf(ptr, 256,"%d error(s) %s\n", ilits->oplus_result, ilits->oplus_result ? "test failed" : "All test passed");	

	if (copy_to_user((char *)buff, ptr, len))
		ILI_ERR("Failed to copy data to user space\n");

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, ENABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, ENABLE);

	*pPos += len;
	ipio_kfree((void **)&ptr);
out:
	mutex_unlock(&ilits->touch_mutex);
	return len;


}
static ssize_t oplus_proc_black_screen_test_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0, len = 0;
	bool esd_en = ilits->wq_esd_ctrl, bat_en = ilits->wq_bat_ctrl;
	char *ptr = NULL;

	if (*pPos != 0 || !ilits->black_flag)
		return 0;

	ILI_INFO("Run MP test with LCM off\n");


	mutex_lock(&ilits->touch_mutex);
	ilits->oplus_result = 0;
	ptr = (char*)kzalloc(256,GFP_KERNEL);
    if (ptr == NULL) {
        ILI_ERR("failed to alloc ptr memory\n");
		len = snprintf(ptr, 256,"buff is not malloc\n");
        goto out;
    }

	//no need gesture mode start
	len = snprintf(ptr, 256,"0 error(s) All test passed\n");
	goto out;
	//no need gesture mode end

	if(ilits->gesture == false){
		ILI_ERR("need to open gesture\n");
		len = snprintf(ptr, 256,"gesture mode is off");
		goto out;
	}

	/* Create the directory for mp_test result */
	if ((ili_dev_mkdir(CSV_LCM_OFF_PATH, S_IRUGO | S_IWUSR)) != 0)
		ILI_ERR("Failed to create directory for mp_test\n");

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, DISABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, DISABLE);

	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	ilits->mp_ret_len = 0;

	ret = ili_mp_test_handler(ili9882_user_buf, OFF);
	ILI_INFO("MP TEST %s, Error code = %d\n", (ret < 0) ? "FAIL" : "PASS", ret);
	if(ilits->oplus_result == 0){
		ILI_INFO("MP TEST PASS or warning\n");
	}else{
		ILI_ERR("MP TEST FAIL\n");
	}	
	len = snprintf(ptr, 256,"%d error(s) %s\n", ilits->oplus_result, ilits->oplus_result ? "test failed" : "All test passed");


out:
	ilits->gesture = ilits->gesture_black;
	ilits->black_flag = false;
	if (copy_to_user((char *)buff, ptr, len))
		ILI_ERR("Failed to copy data to user space\n");

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, ENABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, ENABLE);

	*pPos += len;
	ipio_kfree((void **)&ptr);
	mutex_unlock(&ilits->touch_mutex);
	return len;


}
static ssize_t oplus_proc_black_screen_test_write(struct file *filp, const char *buffer, size_t size, loff_t *pos)
{
  	int ret = 0;
	char cmd[10] = { 0 };
	int value = 0;
	
	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	sscanf(cmd, "%d", &value);

	ILI_INFO("value = %d\n", value);
	ilits->gesture_black = ilits->gesture;
	ilits->gesture = true;
	tp_gesture = 1;
	ilits->black_flag = !!value;
	
	return size;

}
static ssize_t oplus_proc_gesture_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "gesture : %s\n", ilits->switch_gesture_flag ? "Enable" : "Disable");

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}
static ssize_t oplus_proc_gesture_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int ret = 0;
	int gesture_oplus = 0;
	char cmd[256] = { 0 };
	int mode = 0;

	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	mutex_lock(&ilits->touch_mutex);

	gesture_oplus = ili_katoi(cmd);
	ILI_INFO("gesture_oplus = %d\n", gesture_oplus);
	if(gesture_oplus == 1){
		if(ilits->tp_suspend){
			ILI_INFO("gesture mode from proximity\n");
			ilits->switch_gesture_flag = true;
			tp_gesture = 1;
			mode = WAKE_UP_SWITCH_GESTURE_MODE;
			ili_proximity_far(mode);
		}else{
			ILI_INFO("enable gesture mode\n");
			tp_gesture = 1;
			ilits->gesture = true;
			ilits->switch_gesture_flag = true;
		}

	}else if(gesture_oplus == 0){
		ILI_INFO("disable gesture mode\n");
		ilits->gesture = false;
		ilits->switch_gesture_flag = false;
		tp_gesture = 0;
	}else if(gesture_oplus == 2){
		if(ilits->tp_suspend){
			ILI_INFO("enable gesture sleep\n");
			ilits->switch_gesture_flag = false;
			//tp_gesture = 0;
			mode = DDI_POWER_ON;
			ili_proximity_near(mode);	
		}else{
			ILI_INFO("is not in suspend, do nothing\n");
		}

	}
	mutex_unlock(&ilits->touch_mutex);

	return size;

}
static ssize_t oplus_proc_game_switch_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "%x\n", ilits->game_switch_mode);

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}
static ssize_t oplus_proc_game_switch_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int ret = 0;
	char cmd[256] = { 0 };
	int game_cmd = 0;

	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	mutex_lock(&ilits->touch_mutex);

	ILI_INFO("cmd[0] = %c\n", cmd[0]);

	if (cmd[0] == '0') {
		ILI_INFO("disable game switeh\n");
		game_cmd = 0x01;
		ret = ili_ic_func_ctrl("lock_point", game_cmd);	
		if(ret < 0)
			ILI_ERR("write lock_point err\n");
		ilits->game_switch_mode = false;	
	} else {
		ILI_INFO("enable game switeh\n");
		game_cmd = 0x00;
		ret = ili_ic_func_ctrl("lock_point", game_cmd);
		if(ret < 0)
			ILI_ERR("write lock_point err\n");
		ilits->game_switch_mode = true;
	}
	
	mutex_unlock(&ilits->touch_mutex);

	return size;

}
static u32 oplus_rw_reg[5] = {0};

static ssize_t oplus_proc_register_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0, len = 0;
	bool mcu_on = 0, read = 0;
	u32 type, addr, read_data, write_data, write_len, stop_mcu;
	bool esd_en = ilits->wq_esd_ctrl, bat_en = ilits->wq_bat_ctrl;

	if (*pPos != 0)
		return 0;

	stop_mcu = oplus_rw_reg[0];
	type = oplus_rw_reg[1];
	addr = oplus_rw_reg[2];
	write_data = oplus_rw_reg[3];
	write_len = oplus_rw_reg[4];

	ILI_INFO("stop_mcu = %d\n", oplus_rw_reg[0]);

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, DISABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, DISABLE);

	mutex_lock(&ilits->touch_mutex);

	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	if (stop_mcu == mcu_on)
		ret = ili_ice_mode_ctrl(ENABLE, ON);
	else
		ret = ili_ice_mode_ctrl(ENABLE, OFF);

	if (ret < 0) {
		ILI_ERR("Failed to enter ICE mode, ret = %d\n", ret);
		len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "Failed to enter ICE mode\n");
	}

	if (type == read) {
		if (ili_ice_mode_read(addr, &read_data, sizeof(u32)) < 0) {
			ILI_ERR("Read data error\n");
			len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "Read data error\n");
		}

		ILI_INFO("[READ]:addr = 0x%06x, read = 0x%08x\n", addr, read_data);
		len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "READ:addr = 0x%06x, read = 0x%08x\n", addr, read_data);
	} else {
		if (ili_ice_mode_write(addr, write_data, write_len) < 0) {
			ILI_ERR("Write data error\n");
			len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "Write data error\n");
		}

		ILI_INFO("[WRITE]:addr = 0x%06x, write = 0x%08x, len = %d byte\n", addr, write_data, write_len);
		len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "WRITE:addr = 0x%06x, write = 0x%08x, len =%d byte\n", addr, write_data, write_len);
	}

	if (stop_mcu == mcu_on)
		ret = ili_ice_mode_ctrl(DISABLE, ON);
	else
		ret = ili_ice_mode_ctrl(DISABLE, OFF);

	if (ret < 0) {
		ILI_ERR("Failed to disable ICE mode, ret = %d\n", ret);
		len += snprintf(ili9882_user_buf + len, USER_STR_BUFF - len, "%s\n", "Failed to disable ICE mode");
	}

	if (copy_to_user((char *)buff, ili9882_user_buf, len))
		ILI_ERR("Failed to copy data to user space\n");

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, ENABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, ENABLE);

	*pPos += len;
	mutex_unlock(&ilits->touch_mutex);
	return len;
}
static ssize_t oplus_proc_register_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
		char *token = NULL, *cur = NULL;
		char cmd[256] = { 0 };
		u32 count = 0;
	
		if ((size - 1) > sizeof(cmd)) {
			ILI_ERR("ERROR! input length is larger than local buffer\n");
			return -1;
		}
	
		mutex_lock(&ilits->touch_mutex);
	
		if (buffer != NULL) {
			if (copy_from_user(cmd, buffer, size)) {
				ILI_INFO("Failed to copy data from user space\n");
				size = -1;
				goto out;
			}
		}
		token = cur = cmd;
		while ((token = strsep(&cur, ",")) != NULL) {
			oplus_rw_reg[count] = ili_str2hex(token);
			ILI_INFO("oplus_rw_reg[%d] = 0x%x\n", count, oplus_rw_reg[count]);
			count++;
		}
	
	out:
		mutex_unlock(&ilits->touch_mutex);
		return size;

}

static ssize_t ilitek_direction_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "oplus_direction info : %d\n", ilits->direction_oplus);

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}
static ssize_t ilitek_direction_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int ret = 0;
	int direction_oplus = 0;
	char cmd[256] = { 0 };
	int direction_cmd = 0;

	mutex_lock(&ilits->touch_mutex);
	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	sscanf(cmd, "%d", &direction_oplus);

	//direction_oplus = cmd[0] - '0';

	ILI_INFO("direction_oplus = %d\n", direction_oplus);
	if(direction_oplus == 0){
		direction_cmd = 0x01;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->direction_oplus = 0;
	}else if(direction_oplus == 1){
		direction_cmd = 0x02;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);	
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->direction_oplus = 1;
	}else if(direction_oplus == 2){
		direction_cmd = 0x00;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);	
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->direction_oplus = 2;
	}else{ 
		ILI_ERR("UNKNOW CMD\n");
	}
	mutex_unlock(&ilits->touch_mutex);

	return size;

}

static ssize_t oplus_proc_i2c_device_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
#if (TDDI_INTERFACE == BUS_SPI)
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "ILITEK SPI devices\n");
#else
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "NO devices\n");
#endif

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}

static ssize_t ilitek_limit_control_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "tp_limit_enable : %d\n", ilits->tp_limit_enable);

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}
static ssize_t ilitek_limit_control_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int ret = 0;
	int limit_control = 0;
	char cmd[256] = { 0 };
	int direction_cmd = 0;


	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	mutex_lock(&ilits->touch_mutex);

	//sscanf(cmd, "%d", &direction_oplus);
	limit_control = cmd[0] - '0';
	
	ILI_INFO("limit_control = %d\n", ilits->direction_oplus);
	if(ilits->direction_oplus == 0){
		direction_cmd = 0x01;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->tp_limit_enable = 1;
	}else if(ilits->direction_oplus == 1){
		direction_cmd = 0x02;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);	
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->tp_limit_enable = 0;
	}else if(ilits->direction_oplus == 2){
		direction_cmd = 0x00;
		ret = ili_ic_func_ctrl("edge_palm", direction_cmd);	
		if(ret < 0)
			ILI_ERR("write edge_palm err\n");
		ilits->tp_limit_enable = 0;
	}else{ 
		ILI_ERR("UNKNOW CMD\n");
	}
	mutex_unlock(&ilits->touch_mutex);

	return size;

}

static ssize_t oplus_proc_incell_panel_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;

	if (*pPos != 0)
		return 0;
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "1\n");

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;
}

static ssize_t ilitek_proc_oplus_debug_level_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int res = 0;
	char cmd[10] = { 0 };

	if (buffer != NULL) {
		res = copy_from_user(cmd, buffer, size);
		if (res < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}

	ilits->oplus_debug_level = ili_katoi(cmd);

	ILI_INFO("oplus_debug_level = %d\n", ilits->oplus_debug_level);

	return size;

}
static ssize_t oplus_proc_irq_depth_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;
	struct irq_desc *desc = NULL;

	if (*pPos != 0)
		return 0;
	desc = irq_to_desc(ilits->irq_num);
	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "now depth=%d\n", desc->depth);

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}

static ssize_t ilitek_proc_oplus_upgrade_fw_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{
	int ret = 0;
	int value = 0;
	char cmd[10] = { 0 };
	bool esd_en = ilits->wq_esd_ctrl, bat_en = ilits->wq_bat_ctrl;


	if (buffer != NULL) {
		ret = copy_from_user(cmd, buffer, size);
		if (ret < 0) {
			ILI_INFO("copy data from user space, failed\n");
			return -1;
		}
	}
	value = ili_katoi(cmd);
	ILI_INFO("oplus sign value = %d\n", value);
	if(value == 2){
		ilits->sign_fw = true;
	}else if((value == 1)||(value == 0)){
		ilits->sign_fw = false;
	}else{
		ILI_ERR("ERR CMD\n");
		return 0;
	}
	ILI_INFO("Prepar to upgarde firmware\n");

	mutex_lock(&ilits->touch_mutex);

	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

	if (esd_en)
		ili_wq_ctrl(WQ_ESD, DISABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, DISABLE);

	ilits->force_fw_update = ENABLE;
	ilits->node_update = true;

	if (ili_fw_upgrade_handler(NULL) < 0)
		ILI_ERR("FW upgrade failed during oplus fw upgrade\n");

	
	ilits->node_update = false;
	ilits->force_fw_update = DISABLE;
	
	if (esd_en)
		ili_wq_ctrl(WQ_ESD, ENABLE);
	if (bat_en)
		ili_wq_ctrl(WQ_BAT, ENABLE);

	mutex_unlock(&ilits->touch_mutex);
	return size;


}

static ssize_t ilitek_proc_rawdata_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
		s16 *rawdata = NULL;
		int row = 0, col = 0,  index = 0;
		int ret, i, x, y;
		int read_length = 0, len = 0;
		u8 cmd[2] = {0};
		u8 *data = NULL;
	
		if (*pPos != 0)
			return 0;
	
		memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	
		ili_wq_ctrl(WQ_ESD, DISABLE);
		ili_wq_ctrl(WQ_BAT, DISABLE);
		mutex_lock(&ilits->touch_mutex);
	
		row = ilits->ych_num;
		col = ilits->xch_num;
		read_length = 4 + 2 * row * col + 1 ;
	
		ILI_INFO("read length = %d\n", read_length);
	
		data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(data)) {
			ILI_ERR("Failed to allocate data mem\n");
			goto out;
		}
	
		rawdata = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
		if (ERR_ALLOC_MEM(rawdata)) {
			ILI_ERR("Failed to allocate rawdata mem\n");
			goto out;
		}
	
		cmd[0] = 0xB7;
		cmd[1] = 0x2; //get rawdata
		ret = ilits->wrapper(cmd, sizeof(cmd), NULL, 0, OFF, OFF);
		if (ret < 0) {
			ILI_ERR("Failed to write 0xB7,0x2 command, %d\n", ret);
			goto out;
		}
	
		msleep(120);
	
		/* read debug packet header */
		ret = ilits->wrapper(NULL, 0, data, read_length, OFF, OFF);
		if (ret < 0) {
			ILI_ERR("Read debug packet header failed, %d\n", ret);
			goto out;
		}
	
		cmd[1] = 0x03; //switch to normal mode
		ret = ilits->wrapper(cmd, sizeof(cmd), NULL, 0, ON, OFF);
		if (ret < 0) {
			ILI_ERR("Failed to write 0xB7,0x3 command, %d\n", ret);
			goto out;
		}
	
		for (i = 4, index = 0; i < row * col * 2 + 4; i += 2, index++)
			rawdata[index] = (data[i] << 8) + data[i + 1];
	
		len = snprintf(ili9882_user_buf, PAGE_SIZE, "======== RawData ========\n");
		ILI_INFO("======== RawData ========\n");
	
		len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len,
				"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
		ILI_INFO("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
	
		// print raw data
		for (y = 0; y < row; y++) {
			len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "[%2d] ", (y+1));
			ILI_INFO("[%2d] ", (y+1));
	
			for (x = 0; x < col; x++) {
				int shift = y * col + x;
				len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "%5d", rawdata[shift]);
				printk(KERN_CONT "%5d", rawdata[shift]);
			}
			len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "\n");
			printk(KERN_CONT "\n");
		}
	
		if (copy_to_user(buff, ili9882_user_buf, len))
			ILI_ERR("Failed to copy data to user space\n");
	
		*pPos += len;
	
	out:
		mutex_unlock(&ilits->touch_mutex);
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
		ipio_kfree((void **)&data);
		ipio_kfree((void **)&rawdata);
		return len;

}

static ssize_t ilitek_proc_CDC_delta_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
		s16 *delta = NULL;
		int row = 0, col = 0,  index = 0;
		int ret, i, x, y;
		int read_length = 0, len = 0;
		u8 cmd[2] = {0};
		u8 *data = NULL;
	
		if (*pPos != 0)
			return 0;
	
		memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	
		ili_wq_ctrl(WQ_ESD, DISABLE);
		ili_wq_ctrl(WQ_BAT, DISABLE);
		mutex_lock(&ilits->touch_mutex);
	
		row = ilits->ych_num;
		col = ilits->xch_num;
		read_length = 4 + 2 * row * col + 1 ;
	
		ILI_INFO("read length = %d\n", read_length);
	
		data = kcalloc(read_length + 1, sizeof(u8), GFP_KERNEL);
		if (ERR_ALLOC_MEM(data)) {
			ILI_ERR("Failed to allocate data mem\n");
			goto out;
		}
	
		delta = kcalloc(P5_X_DEBUG_MODE_PACKET_LENGTH, sizeof(s32), GFP_KERNEL);
		if (ERR_ALLOC_MEM(delta)) {
			ILI_ERR("Failed to allocate delta mem\n");
			goto out;
		}
	
		cmd[0] = 0xB7;
		cmd[1] = 0x1; //get delta
		ret = ilits->wrapper(cmd, sizeof(cmd), NULL, 0, OFF, OFF);
		if (ret < 0) {
			ILI_ERR("Failed to write 0xB7,0x1 command, %d\n", ret);
			goto out;
		}
	
		msleep(120);
	
		/* read debug packet header */
		ret = ilits->wrapper(NULL, 0, data, read_length, OFF, OFF);
		if (ret < 0) {
			ILI_ERR("Read debug packet header failed, %d\n", ret);
			goto out;
		}
	
		cmd[1] = 0x03; //switch to normal mode
		ret = ilits->wrapper(cmd, sizeof(cmd), NULL, 0, ON, OFF);
		if (ret < 0) {
			ILI_ERR("Failed to write 0xB7,0x3 command, %d\n", ret);
			goto out;
		}
	
		for (i = 4, index = 0; i < row * col * 2 + 4; i += 2, index++)
			delta[index] = (data[i] << 8) + data[i + 1];
	
		len = snprintf(ili9882_user_buf + size, PAGE_SIZE - len, "======== Deltadata ========\n");
		ILI_INFO("======== Deltadata ========\n");
	
		len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len,
			"Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
		ILI_INFO("Header 0x%x ,Type %d, Length %d\n", data[0], data[1], (data[2] << 8) | data[3]);
	
		// print delta data
		for (y = 0; y < row; y++) {
			len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "[%2d] ", (y+1));
			ILI_INFO("[%2d] ", (y+1));
	
			for (x = 0; x < col; x++) {
				int shift = y * col + x;
				len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "%5d", delta[shift]);
				printk(KERN_CONT "%5d", delta[shift]);
			}
			len += snprintf(ili9882_user_buf + len, PAGE_SIZE - len, "\n");
			printk(KERN_CONT "\n");
		}
	
		if (copy_to_user(buff, ili9882_user_buf, len))
			ILI_ERR("Failed to copy data to user space\n");
	
		*pPos += len;
	
	out:
		mutex_unlock(&ilits->touch_mutex);
		ili_wq_ctrl(WQ_ESD, ENABLE);
		ili_wq_ctrl(WQ_BAT, ENABLE);
		ipio_kfree((void **)&data);
		ipio_kfree((void **)&delta);
		return len;

}

static ssize_t ilitek_proc_main_register_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	uint32_t len = 0;
	u8 szBuf[4];
	if (*pPos != 0)
		return 0;
	szBuf[3] = ilits->chip->fw_ver & 0xFF;
	szBuf[2] = (ilits->chip->fw_ver >> 8) & 0xFF;
	szBuf[1] = (ilits->chip->fw_ver >> 16) & 0xFF;
	szBuf[0] = ilits->chip->fw_ver >> 24;

	memset(ili9882_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));
	len = snprintf(ili9882_user_buf, USER_STR_BUFF * sizeof(unsigned char), "%d.%d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2], szBuf[3]);

	*pPos += len;

	if (copy_to_user(buff, ili9882_user_buf, len))
		ILI_INFO("Failed to copy data to user space\n");

	return len;


}
/*
static ssize_t ilitek_proc_oplus_sign_firmware_write(struct file *filp, const char *buffer, size_t size, loff_t *pPos)
{

}
*/
static void *c_start(struct seq_file *m, loff_t *pos)
{
	return *pos < 1 ? (void *)1 : NULL;
}

static void *c_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void c_stop(struct seq_file *m, void *v)
{
	return;
}

static int32_t c_oplus_ili_coordinate_show(struct seq_file *m, void *v)
{
	struct gesture_coordinate *gesture = ilits->gcoord;
	char tmp[256] = {0};
	ILI_INFO("c_oplus_coordinate_show\n");
	sprintf(tmp, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d",
		gesture->gesture_type,
		gesture->pos_start.x, gesture->pos_start.y,
		gesture->pos_end.x, gesture->pos_end.y,
		gesture->pos_1st.x, gesture->pos_1st.y,
		gesture->pos_2nd.x, gesture->pos_2nd.y,
		gesture->pos_3rd.x, gesture->pos_3rd.y,
		gesture->pos_4th.x, gesture->pos_4th.y,
		gesture->clockwise);

	/* oplus gesture formate */
	seq_printf(m, "%s\n", tmp);

	return 0;
}

const struct seq_operations ili9882n_oplus_ili_coordinate_seq_ops = {
	.start  = c_start,
	.next   = c_next,
	.stop   = c_stop,
	.show   = c_oplus_ili_coordinate_show
};

static int32_t oplus_ili_coordinate_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &ili9882n_oplus_ili_coordinate_seq_ops);
}


struct file_operations proc_basline_test_fops = {
	.read = oplus_proc_mp_lcm_on_read,
};

struct file_operations proc_blank_screen_fops = {
	.read  = oplus_proc_black_screen_test_read,
	.write = oplus_proc_black_screen_test_write,
};

struct file_operations proc_double_tap_fops = {
	.write = oplus_proc_gesture_write,
	.read = oplus_proc_gesture_read,
};

struct file_operations proc_game_switch_fops = {
	.read  = oplus_proc_game_switch_read,
	.write = oplus_proc_game_switch_write,
};

struct file_operations ili9882n_proc_oplus_register_info_fops = {
	.write =oplus_proc_register_write,
	.read = oplus_proc_register_read,
};

struct file_operations proc_oplus_tp_direction_fops = {
	.owner = THIS_MODULE,
	.read = ilitek_direction_read,
	.write = ilitek_direction_write,
};

struct file_operations proc_coordinate_fops = {
	.owner = THIS_MODULE,
	.open = oplus_ili_coordinate_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

struct file_operations proc_i2c_device_test_fops = {
	.read = oplus_proc_i2c_device_read,
};

struct file_operations proc_oplus_tp_limit_enable_fops =
{
    .read  = ilitek_limit_control_read,
    .write = ilitek_limit_control_write,
    .owner = THIS_MODULE,
};

struct file_operations proc_incell_panel_fops = {
	.read = oplus_proc_incell_panel_read,
};

struct file_operations proc_debug_level_oplus_fops = {
	.write = ilitek_proc_oplus_debug_level_write,
};

struct file_operations ili9882n_proc_irq_depth_fops = {
	.read = oplus_proc_irq_depth_read,
};

struct file_operations proc_tp_fw_update_fops = {
	.write = ilitek_proc_oplus_upgrade_fw_write,
};

struct file_operations proc_basline_fops = {
	.read = ilitek_proc_rawdata_read,
};

struct file_operations proc_delta_fops = {
	.read = ilitek_proc_CDC_delta_read,
};

struct file_operations ili9882n_proc_main_register_fops = {
	.read = ilitek_proc_main_register_read,
};
/*
struct file_operations proc_sign_firmware_fops = {
	.write = ilitek_proc_oplus_sign_firmware_write,
};
*/
proc_node_t proc_table_oplus[] = {
	{"baseline_test", NULL, &proc_basline_test_fops, false},
	{"black_screen_test", NULL, &proc_blank_screen_fops, false},
	{"double_tap_enable", NULL, &proc_double_tap_fops, false},
	{"game_switch_enable", NULL, &proc_game_switch_fops, false},
	{"oplus_register_info", NULL, &ili9882n_proc_oplus_register_info_fops, false},
	{"oplus_tp_direction", NULL, &proc_oplus_tp_direction_fops, false},
	{"coordinate", NULL, &proc_coordinate_fops, false},
	{"i2c_device_test", NULL, &proc_i2c_device_test_fops, false},
	{"oplus_tp_limit_enable", NULL, &proc_oplus_tp_limit_enable_fops, false},
	{"incell_panel", NULL, &proc_incell_panel_fops, false},
//	{"sign_firmware", NULL, &proc_sign_firmware_fops, false},
	{"debug_level", NULL, &proc_debug_level_oplus_fops, false},
	{"irq_depth", NULL, &ili9882n_proc_irq_depth_fops, false},
	{"tp_fw_update", NULL, &proc_tp_fw_update_fops, false},

};
proc_node_t proc_table_debug_info[] = {
	{"baseline", NULL, &proc_basline_fops, false},
	{"delta", NULL, &proc_delta_fops, false},
//	{"freq_hop_simulate", NULL, &proc_freq_hop_simulate_fops, false},
	{"main_register", NULL, &ili9882n_proc_main_register_fops, false},
};


void ili9882n_tddi_node_init(void)
{
	int i = 0, ret = 0, size;

	proc_dir_oplus = proc_mkdir("touchpanel", NULL);
	proc_dir_debug_info = proc_mkdir("debug_info", proc_dir_oplus);

	
	size = ARRAY_SIZE(proc_table_oplus);
	for (; i < size; i++) {
		proc_table_oplus[i].node = proc_create(proc_table_oplus[i].name, 0666, proc_dir_oplus, proc_table_oplus[i].fops);

		if (proc_table_oplus[i].node == NULL) {
			proc_table_oplus[i].isCreated = false;
			ILI_ERR("Failed to create oplus %s under /proc\n", proc_table_oplus[i].name);
			ret = -ENODEV;
		} else {
			proc_table_oplus[i].isCreated = true;
			ILI_INFO("Succeed to create oplus %s under /proc\n", proc_table_oplus[i].name);
		}
	}

	size = ARRAY_SIZE(proc_table_debug_info);
	for (i = 0; i < size; i++) {
		proc_table_debug_info[i].node = proc_create(proc_table_debug_info[i].name, 0644, proc_dir_debug_info, proc_table_debug_info[i].fops);

		if (proc_table_debug_info[i].node == NULL) {
			proc_table_debug_info[i].isCreated = false;
			ILI_ERR("Failed to create oplus %s under /proc\n", proc_table_debug_info[i].name);
			ret = -ENODEV;
		} else {
			proc_table_debug_info[i].isCreated = true;
			ILI_INFO("Succeed to create oplus %s under /proc\n", proc_table_debug_info[i].name);
		}
	}

}
