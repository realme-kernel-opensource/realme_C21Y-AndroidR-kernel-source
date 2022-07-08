#define LOG_TAG         "Vendor"

#include "cts_config.h"
#include "cts_platform.h"
#include "cts_core.h"
#include "cts_test.h"
#include "cts_firmware.h"
#include "cts_strerror.h"

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/tp_usb_notifier.h>
#include <linux/headset_notifier.h>

/* /proc/touchpanel */
#define PROC_TOUCHPANEL_DIR_NAME    "touchpanel"
#define PROC_TOUCHPANEL_DIR_PATH    "/proc/"PROC_TOUCHPANEL_DIR_NAME

#define PROC_BASELINE_TEST_FILENAME "baseline_test"
#define PROC_BASELINE_TEST_FILEPATH \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_BASELINE_TEST_FILENAME

#define PROC_BLACK_SCREEN_TEST_FILENAME "black_screen_test"
#define PROC_BLACK_SCREEN_TEST_FILEPATH \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_BLACK_SCREEN_TEST_FILENAME

#define PROC_COORDINATE_FILENAME "coordinate"
#define PROC_COORDINATE_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_COORDINATE_FILENAME

#define PROC_DEBUG_LEVEL_FILENAME "debug_level"
#define PROC_DEBUG_LEVEL_FILEPATH   \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_DEBUG_LEVEL_FILENAME

#define PROC_DOUBLE_TAP_ENABLE_FILENAME "double_tap_enable"
#define PROC_DOUBLE_TAP_ENABLE_FILEPATH \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_DOUBLE_TAP_ENABLE_FILENAME

#define PROC_GAME_SWITCH_ENABLE_FILENAME "game_switch_enable"
#define PROC_GAME_SWITCH_ENABLE_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_GAME_SWITCH_ENABLE_FILENAME

#define PROC_INCELL_PANEL_FILENAME "incell_panel"
#define PROC_INCELL_PANEL_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_INCELL_PANEL_FILENAME

#define PROC_IRQ_DEPTH_FILENAME "irq_depth"
#define PROC_IRQ_DEPTH_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_IRQ_DEPTH_FILENAME

#define PROC_OPLUS_REGISTER_INFO_FILENAME "oplus_register_info"
#define PROC_OPLUS_REGISTER_INFO_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_OPLUS_REGISTER_INFO_FILENAME

#define PROC_OPLUS_TP_DIRECTION_FILENAME "oplus_tp_direction"
#define PROC_OPLUS_TP_DIRECTION_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_OPLUS_TP_DIRECTION_FILENAME

#define PROC_OPLUS_TP_LIMIT_ENABLE_FILENAME "oplus_tp_limit_enable"
#define PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_OPLUS_TP_LIMIT_ENABLE_FILENAME

#define PROC_TP_FW_UPDATE_FILENAME "tp_fw_update"
#define PROC_TP_FW_UPDATE_FILEPATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_TP_FW_UPDATE_FILENAME

/* /proc/debug_info */
#define PROC_DEBUG_INFO_DIR_NAME    "debug_info"
#define PROC_DEBUG_INFO_DIR_PATH    \
    PROC_TOUCHPANEL_DIR_PATH"/"PROC_DEBUG_INFO_DIR_NAME

#define PROC_BASELINE_FILENAME  "baseline"
#define PROC_BASELINE_FILEPATH    \
    PROC_DEBUG_INFO_DIR_PATH"/"PROC_BASELINE_FILENAME

#define PROC_DATA_LIMIT_FILENAME  "data_limit"
#define PROC_DATA_LIMIT_FILEPATH    \
    PROC_DEBUG_INFO_DIR_PATH"/"PROC_DATA_LIMIT_FILENAME

#define PROC_DELTA_FILENAME  "delta"
#define PROC_DELTA_FILEPATH    \
    PROC_DEBUG_INFO_DIR_PATH"/"PROC_DELTA_FILENAME

#define PROC_MAIN_REGISTER_FILENAME  "main_register"
#define PROC_MAIN_REGISTER_FILEPATH    \
    PROC_DEBUG_INFO_DIR_PATH"/"PROC_MAIN_REGISTER_FILENAME

#define TEST_DATA_DIR                   "/data/vendor/fac_sources"
#define RAWDATA_TEST_DATA_FILEPATH      TEST_DATA_DIR"/rawdata.csv"
#define NOISE_TEST_DATA_FILEPATH        TEST_DATA_DIR"/noise.csv"
#define OPEN_TEST_DATA_FILEPATH         TEST_DATA_DIR"/open.csv"
#define SHORT_TEST_DATA_FILEPATH        TEST_DATA_DIR"/short.csv"
#define COMP_CAP_TEST_DATA_FILEPATH     TEST_DATA_DIR"/compensate_cap.csv"

struct cts_vendor_data {
#ifdef CONFIG_PROC_FS
    struct proc_dir_entry *proc_dir_touchpanel_entry;
    struct proc_dir_entry *proc_dir_debuginfo_entry;

    void *rawdata;

    void *delta;

    int   double_tap_enable;
    char  coordinate[256];
    int   coordinate_len;
    int   debug_level;

    bool game_switch_enable;

    u32 oplus_tp_direction;
    u32 oplus_tp_limit_enable;

    u32  register_addr;
    u32  register_len;

    /* Baseline test(Screen ON) parameter */
    bool test_reset_pin;
    int  reset_pin_test_result;

    bool test_int_pin;
    int  int_pin_test_result;

    bool test_rawdata;
    u32  rawdata_test_frames;
    int  rawdata_test_result;
    int  rawdata_min;
    int  rawdata_max;
    void *rawdata_test_data;
    int  rawdata_test_data_buf_size;
    int  rawdata_test_data_size;

    bool test_noise;
    u32  noise_test_frames;
    int  noise_test_result;
    int  noise_max;
    void *noise_test_data;
    int  noise_test_data_buf_size;
    int  noise_test_data_size;

    bool test_open;
    int  open_test_result;
    int  open_min;
    void *open_test_data;
    int  open_test_data_buf_size;
    int  open_test_data_size;

    bool test_short;
    int  short_test_result;
    int  short_min;
    void *short_test_data;
    int  short_test_data_buf_size;
    int  short_test_data_size;

    bool test_comp_cap;
    int  comp_cap_test_result;
    int  comp_cap_min;
    int  comp_cap_max;
    void *comp_cap_test_data;
    int  comp_cap_test_data_buf_size;
    int  comp_cap_test_data_size;

    /* Black screen test parameter */
    bool test_lpwg_rawdata;
    u32  lpwg_rawdata_test_frames;
    int  lpwg_rawdata_test_result;
    int  lpwg_rawdata_min;
    int  lpwg_rawdata_max;
    void *lpwg_rawdata_test_data;
    int  lpwg_rawdata_test_data_buf_size;
    int  lpwg_rawdata_test_data_size;
#endif /* CONFIG_PROC_FS */

    /* Headset */
    bool headset_notifier_registered;
    struct notifier_block headset_notifier;
    struct work_struct set_headset_state_work;
    bool headset_state;

    /* USB */
    bool usb_notifier_registered;
    struct notifier_block usb_notifier;
    struct work_struct set_usb_state_work;
    bool usb_state;

    struct chipone_ts_data *cts_data;
};

#ifdef CONFIG_PROC_FS
static void *seq_start(struct seq_file *m, loff_t *pos)
{
    return *pos < 1 ? (void *)1 : NULL;
}

static void *seq_next(struct seq_file *m, void *v, loff_t *pos)
{
    ++*pos;
    return NULL;
}

static void seq_stop(struct seq_file *m, void *v)
{
    return;
}

/* Use for NOT use seq file to set private data with chipone_ts_data */
static int proc_simple_open(struct inode *inode, struct file *file)
{
    file->private_data = PDE_DATA(inode);
    return 0;
}

#define ALLOC_TEST_DATA_MEM(type, size) \
    do { \
        if (vdata->test_##type) { \
            if (vdata->type##_test_data == NULL) { \
                cts_info("- Alloc " #type " test data mem size %d", size); \
                vdata->type##_test_data = vmalloc(size); \
                if (vdata->type##_test_data == NULL) { \
                    cts_err("Alloc " #type " test data mem failed"); \
                    return -ENOMEM; \
                } \
                vdata->type##_test_data_size = size; \
            } \
            memset(vdata->type##_test_data, 0, size); \
        } \
    } while (0)

#define FREE_TEST_DATA_MEM(type) \
    do { \
        if (vdata->type##_test_data) { \
            cts_info("- Free " #type " test data mem"); \
            vfree(vdata->type##_test_data); \
            vdata->type##_test_data = NULL; \
        } \
    } while(0)

static int alloc_baseline_test_data_mem(struct cts_vendor_data *vdata, int nodes)
{
    cts_info("Alloc baseline test data mem");

    ALLOC_TEST_DATA_MEM(rawdata,
        nodes * 2 * vdata->rawdata_test_frames);
    ALLOC_TEST_DATA_MEM(noise,
        nodes * 2 * vdata->noise_test_frames);
    ALLOC_TEST_DATA_MEM(open, nodes * 2);
    ALLOC_TEST_DATA_MEM(short, nodes * 2 * 7);
    ALLOC_TEST_DATA_MEM(comp_cap, nodes);

    return 0;
}

static void free_baseline_test_data_mem(struct cts_vendor_data *vdata)
{
    cts_info("Free baseline test data mem");

    FREE_TEST_DATA_MEM(rawdata);
    FREE_TEST_DATA_MEM(noise);
    FREE_TEST_DATA_MEM(open);
    FREE_TEST_DATA_MEM(short);
    FREE_TEST_DATA_MEM(comp_cap);
}
#if 0
static int alloc_black_screen_test_data_mem(struct cts_vendor_data *vdata, int nodes)
{
    cts_info("Alloc black screen test data mem");

    ALLOC_TEST_DATA_MEM(lpwg_rawdata,
        nodes * 2 * vdata->lpwg_rawdata_test_frames);

    return 0;
}

static void free_black_screen_test_data_mem(struct cts_vendor_data *vdata)
{
    cts_info("Free black screen test data mem");

    FREE_TEST_DATA_MEM(lpwg_rawdata);
}
#endif
#undef ALLOC_TEST_DATA_MEM
#undef FREE_TEST_DATA_MEM

static int dump_touch_data_row_to_buffer(char *buf, size_t size, const void *data,
    int cols, const char *prefix, const char *suffix, char seperator)
{
    int c, count = 0;

    if (prefix) {
        count += scnprintf(buf, size, "%s", prefix);
    }

    for (c = 0; c < cols; c++) {
        count += scnprintf(buf + count, size - count,
            "%4d%c ", ((s16 *)data)[c], seperator);
    }

    if (suffix) {
        count += scnprintf(buf + count, size - count, "%s", suffix);
    }

    return count;
}

static int dump_touch_data_to_csv_file(const char *filepath,
    const void *data, int frames, int rows, int cols)
{
    struct file *file;
    int r, ret = 0;
    loff_t pos = 0;

    cts_info("Dump touch data to csv file: '%s' frames: %u row: %d col: %d",
        filepath, frames, rows, cols);

    file = filp_open(filepath, O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (IS_ERR(file)) {
        cts_err("Open file '%s' failed %ld(%s)", filepath,
            PTR_ERR(file), cts_strerror((int)PTR_ERR(file)));
        return PTR_ERR(file);
    }

    while (frames--) {
        for (r = 0; r < rows; r++) {
            char linebuf[256];
            int len;

            len = dump_touch_data_row_to_buffer(linebuf,
                sizeof(linebuf), data, cols, NULL, "\n", ',');
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
            ret = kernel_write(file, linebuf, len, &pos);
#else
            ret = kernel_write(file, linebuf, len, pos);
            pos += len;
#endif
            if (ret != len) {
                cts_err("Write to file '%s' failed %d(%s)",
                    filepath, ret, cts_strerror(ret));
                ret = -EIO;
                goto close_file;
            }

            data += cols * 2;
        }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        ret = kernel_write(file, "\n", 1, &pos);
#else
        ret = kernel_write(file, "\n", 1, pos);
        pos ++;
#endif
        if (ret != 1) {
            cts_err("Write newline to file '%s' failed %d(%s)",
                filepath, ret, cts_strerror(ret));
            ret = -EIO;
            goto close_file;
        }
    }

close_file: {
        int r = filp_close(file, NULL);
        if (r) {
            cts_err("Close file '%s' failed %d(%s)",
                filepath, ret, cts_strerror(ret));
        }
    }

    return ret;
}

static void dump_touch_data_to_seq_file(struct seq_file *m,
    const void *data, int rows, int cols)
{
    int r;

    for (r = 0; r < rows; r++) {
        char linebuf[256];
        int len;

        len = dump_touch_data_row_to_buffer(linebuf, sizeof(linebuf),
            data, cols, NULL, "\n", ',');
        seq_puts(m, linebuf);

        data += cols * 2;
    }
}

static int dump_comp_cap_row_to_buffer(char *buf, size_t size, const u8 *cap,
    int cols, const char *prefix, const char *suffix, char seperator)
{
    int c, count = 0;

    if (prefix) {
        count += scnprintf(buf, size, "%s", prefix);
    }

    for (c = 0; c < cols; c++) {
        count += scnprintf(buf + count, size - count,
            "%3u%c ", cap[c], seperator);
    }

    if (suffix) {
        count += scnprintf(buf + count, size - count, "%s", suffix);
    }

    return count;
}

static int dump_comp_cap_to_csv_file(const char *filepath,
    const u8 *cap, int rows, int cols)
{
    struct file *file;
    int r, ret = 0;
    loff_t pos = 0;

    cts_info("Dump compensate cap to csv file: '%s' row: %d col: %d",
        filepath, rows, cols);

    file = filp_open(filepath, O_RDWR | O_CREAT | O_TRUNC, 0666);
    if (IS_ERR(file)) {
        cts_err("Open file '%s' failed %ld(%s)", filepath,
            PTR_ERR(file), cts_strerror((int)PTR_ERR(file)));
        return PTR_ERR(file);
    }

    for (r = 0; r < rows; r++) {
        char linebuf[256];
        int len;

        len = dump_comp_cap_row_to_buffer(linebuf, sizeof(linebuf),
            cap, cols, NULL, "\n", ',');
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,14,0)
        ret = kernel_write(file, linebuf, len, &pos);
#else
        ret = kernel_write(file, linebuf, len, pos);
        pos += len;
#endif
        if (ret != len) {
            cts_err("Write to file '%s' failed %d(%s)",
                filepath, ret, cts_strerror(ret));
            goto close_file;
        }

        cap += cols;
    }

close_file: {
        int r = filp_close(file, NULL);
        if (r) {
            cts_err("Close file '%s' failed %d(%s)",
                filepath, ret, cts_strerror(ret));
        }
    }

    return ret;
}

#if 0
static void dump_comp_cap_to_seq_file(struct seq_file *m,
    const u8 *data, int rows, int cols)
{
    int r;

    for (r = 0; r < rows; r++) {
        char linebuf[256];
        int len;

        len = dump_comp_cap_row_to_buffer(linebuf, sizeof(linebuf),
            data, cols, NULL, "\n", ',');
        seq_puts(m, linebuf);

        data += cols;
    }
}
#endif

static int save_baseline_test_data_to_file(struct cts_vendor_data *vdata)
{
    int rows, cols;
    int ret;

    cts_info("Save baseline test data to file");

    rows  = vdata->cts_data->cts_dev.fwdata.rows;
    cols  = vdata->cts_data->cts_dev.fwdata.cols;

    if (vdata->test_rawdata) {
        cts_info(" - Save rawdata test data to file");
        ret = dump_touch_data_to_csv_file(
            RAWDATA_TEST_DATA_FILEPATH, vdata->rawdata_test_data,
            vdata->rawdata_test_frames, rows, cols);
        if (ret < 0) {
            cts_err("Dump rawdata test data to file failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    }

    if (vdata->test_noise) {
        cts_info(" - Save noise test data to file");
        ret = dump_touch_data_to_csv_file(
            NOISE_TEST_DATA_FILEPATH, vdata->noise_test_data,
            vdata->noise_test_frames, rows, cols);
        if (ret < 0) {
            cts_err("Dump noise test data to file failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    }

    if (vdata->test_open) {
        cts_info(" - Save open test data to file");
        ret = dump_touch_data_to_csv_file(
            OPEN_TEST_DATA_FILEPATH, vdata->open_test_data,
            1, rows, cols);
        if (ret < 0) {
            cts_err("Dump open test data to file failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    }

    if (vdata->test_short) {
        cts_info(" - Save short test data to file");
        ret = dump_touch_data_to_csv_file(
            SHORT_TEST_DATA_FILEPATH, vdata->short_test_data,
            7, rows, cols);
        if (ret < 0) {
            cts_err("Dump short test data to file failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    }

    if (vdata->test_comp_cap) {
        cts_info(" - Save compensate-cap test data to file");
        ret = dump_comp_cap_to_csv_file(
            COMP_CAP_TEST_DATA_FILEPATH, vdata->comp_cap_test_data,
            rows, cols);
        if (ret < 0) {
            cts_err("Dump compensate cap test data to file failed %d(%s)",
                ret, cts_strerror(ret));
            return ret;
        }
    }

    return 0;
}

static int baseline_test_seq_show(struct seq_file *m, void *v)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int rows, cols;
    int errors = 0;

    cts_data = (struct chipone_ts_data *)m->private;
    if (cts_data == NULL) {
        cts_err("Baseline test seq file private data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    rows  = cts_data->cts_dev.fwdata.rows;
    cols  = cts_data->cts_dev.fwdata.cols;

    if (vdata->test_reset_pin && vdata->reset_pin_test_result != 0) {
        errors++;
    }
    if (vdata->test_int_pin && vdata->int_pin_test_result != 0) {
        errors++;
    }
    if (vdata->test_rawdata && vdata->rawdata_test_result != 0) {
        errors++;
    }
    if (vdata->test_noise && vdata->noise_test_result != 0) {
        errors++;
    }
    if (vdata->test_open && vdata->open_test_result != 0) {
        errors++;
    }
    if (vdata->test_short && vdata->short_test_result != 0) {
        errors++;
    }
    if (vdata->test_comp_cap && vdata->comp_cap_test_result != 0) {
        errors++;
    }

    if (errors) {
        seq_printf(m, "MP test fail.\n");
    } else {
        seq_puts(m, "0 errors. All test passed.\n");
    }

    cts_info("FW Version %04x\n", cts_data->cts_dev.fwdata.version);

    if (vdata->test_reset_pin) {
        cts_info("Reset-Pin Test %s!\n\n",
            vdata->reset_pin_test_result == 0 ? "PASS" : "FAIL");
    }
    if (vdata->test_int_pin) {
        cts_info("Int-Pin Test %s!\n\n",
            vdata->int_pin_test_result == 0 ? "PASS" : "FAIL");
    }
    if (vdata->test_rawdata) {
        cts_info("FW Rawdata Test");
        if (vdata->rawdata_test_result == 0) {
            cts_info(" PASS!\n");
        } else if (vdata->rawdata_test_result > 0) {
            cts_err(" FAIL!\n");
#if 0
            for(i = 0; i < vdata->rawdata_test_frames; i++) {
                dump_touch_data_to_seq_file(m,
                    vdata->rawdata_test_data + i * rows * cols, rows, cols);
                seq_putc(m, '\n');
            }
#endif 
        } else {
            cts_err(" ERROR(%d)!\n\n", vdata->rawdata_test_result);
        }
    }
    if (vdata->test_noise) {
        cts_info("Noise Test");
        if (vdata->noise_test_result == 0) {
            cts_info(" PASS!\n");
        } else if (vdata->noise_test_result > 0) {
            cts_err(" FAIL!\n");
#if 0
            for(i = 0; i < vdata->noise_test_frames; i++) {
                dump_touch_data_to_seq_file(m,
                    vdata->noise_test_data + i * rows * cols , rows, cols);
                seq_putc(m, '\n');
            }
#endif
        } else {
            cts_err(" ERROR(%d)!\n\n", vdata->noise_test_result);
        }
    }
    if (vdata->test_open) {
        cts_info("Open Test");
        if (vdata->open_test_result == 0) {
            cts_info(" PASS!\n");
        } else if (vdata->open_test_result > 0) {
            cts_err(" FAIL!\n");
            //dump_touch_data_to_seq_file(m,
                //vdata->open_test_data, rows, cols);
        } else {
            cts_err(" ERROR(%d)!\n\n", vdata->open_test_result);
        }
    }
    if (vdata->test_short) {
        cts_info("Short Test");
        if (vdata->short_test_result == 0) {
            cts_info(" PASS!\n");
        } else if (vdata->short_test_result > 0) {
            cts_err(" FAIL!\n");
#if 0
            for (i = 0; i < 7; i++) {
                dump_touch_data_to_seq_file(m,
                    vdata->short_test_data + i * rows * cols, rows, cols);
                seq_putc(m, '\n');
            }
#endif
        } else {
            cts_err(" ERROR(%d)!\n\n", vdata->short_test_result);
        }
    }
    if (vdata->test_comp_cap) {
        cts_info("Compensate-Cap Test");
        if (vdata->comp_cap_test_result == 0) {
            cts_info(" PASS!\n");
        } else if (vdata->comp_cap_test_result > 0) {
            cts_err(" FAIL!\n");
            //dump_comp_cap_to_seq_file(m,
                //vdata->comp_cap_test_data, rows, cols);
        } else {
            cts_err(" ERROR(%d)!\n\n", vdata->comp_cap_test_result);
        }
    }

    return 0;
}

const struct seq_operations baseline_test_seq_ops = {
    .start  = seq_start,
    .next   = seq_next,
    .stop   = seq_stop,
    .show   = baseline_test_seq_show,
};

static int init_baseline_test_param_ls(struct cts_vendor_data *vdata)
{
    cts_info("Init baseline test param");

    vdata->test_reset_pin = true;

    vdata->test_int_pin = true;

    vdata->test_rawdata = true;
    vdata->rawdata_test_frames = 1;
    vdata->rawdata_min = 1400;
    vdata->rawdata_max = 2600;

    vdata->test_noise = true;
    vdata->noise_test_frames = 50;
    vdata->noise_max = 80;

    vdata->test_open = true;
    vdata->open_min = 2770;

    vdata->test_short = true;
    vdata->short_min = 400;

    vdata->test_comp_cap = true;
    vdata->comp_cap_min = 1;
    vdata->comp_cap_max = 126;

    vdata->test_lpwg_rawdata = true;
    vdata->lpwg_rawdata_test_frames = 50;
    vdata->lpwg_rawdata_min = 10000;
    vdata->lpwg_rawdata_max = 20000;

    return 0;
}

static int init_baseline_test_param_xl(struct cts_vendor_data *vdata)
{
    cts_info("Init baseline test param");

    vdata->test_reset_pin = true;

    vdata->test_int_pin = true;

    vdata->test_rawdata = true;
    vdata->rawdata_test_frames = 1;
    vdata->rawdata_min = 1400;
    vdata->rawdata_max = 2600;

    vdata->test_noise = true;
    vdata->noise_test_frames = 50;
    vdata->noise_max = 80;

    vdata->test_open = true;
    vdata->open_min = 2770;

    vdata->test_short = true;
    vdata->short_min = 400;

    vdata->test_comp_cap = true;
    vdata->comp_cap_min = 1;
    vdata->comp_cap_max = 126;

    vdata->test_lpwg_rawdata = true;
    vdata->lpwg_rawdata_test_frames = 50;
    vdata->lpwg_rawdata_min = 10000;
    vdata->lpwg_rawdata_max = 20000;

    return 0;
}

static void do_baseline_test(struct cts_vendor_data *vdata)
{
    struct cts_device *cts_dev = &vdata->cts_data->cts_dev;
    struct cts_test_param reset_pin_test_param = {
        .test_item = CTS_TEST_RESET_PIN,
        .flags = 0,
    };
    struct cts_test_param int_pin_test_param = {
        .test_item = CTS_TEST_INT_PIN,
        .flags = 0,
    };
    struct cts_rawdata_test_priv_param rawdata_test_priv_param = {0};
    struct cts_test_param rawdata_test_param = {
        .test_item = CTS_TEST_RAWDATA,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE,
        .test_data_filepath = "/sdcard/chipone-tddi/test/rawdata.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .priv_param = &rawdata_test_priv_param,
        .priv_param_size = sizeof(rawdata_test_priv_param),
    };
    struct cts_noise_test_priv_param noise_test_priv_param = {0};
    struct cts_test_param noise_test_param = {
        .test_item = CTS_TEST_NOISE,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE,
        .test_data_filepath = "/sdcard/chipone-tddi/test/noise.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
        .priv_param = &noise_test_priv_param,
        .priv_param_size = sizeof(noise_test_priv_param),
    };
    struct cts_test_param open_test_param = {
        .test_item = CTS_TEST_OPEN,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE,
        .test_data_filepath = "/sdcard/chipone-tddi/test/open.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
    struct cts_test_param short_test_param = {
        .test_item = CTS_TEST_SHORT,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE,
        .test_data_filepath = "/sdcard/chipone-tddi/test/short.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };
    struct cts_test_param comp_cap_test_param = {
        .test_item = CTS_TEST_COMPENSATE_CAP,
        .flags = CTS_TEST_FLAG_VALIDATE_DATA |
                 CTS_TEST_FLAG_VALIDATE_MIN |
                 CTS_TEST_FLAG_VALIDATE_MAX |
                 CTS_TEST_FLAG_STOP_TEST_IF_VALIDATE_FAILED |
                 CTS_TEST_FLAG_DUMP_TEST_DATA_TO_USERSPACE,
        .test_data_filepath = "/sdcard/chipone-tddi/test/compensate-cap.txt",
        .num_invalid_node = 0,
        .invalid_nodes = NULL,
    };

    rawdata_test_priv_param.frames = vdata->rawdata_test_frames;
    rawdata_test_param.min = &vdata->rawdata_min;
    rawdata_test_param.max = &vdata->rawdata_max;
    rawdata_test_param.test_data_buf = vdata->rawdata_test_data;
    rawdata_test_param.test_data_buf_size = vdata->rawdata_test_data_buf_size;
    vdata->rawdata_test_data_size = 0;
    rawdata_test_param.test_data_wr_size = &vdata->rawdata_test_data_size;

    noise_test_priv_param.frames = vdata->noise_test_frames;
    noise_test_param.max = &vdata->noise_max;
    noise_test_param.test_data_buf = vdata->noise_test_data;
    noise_test_param.test_data_buf_size = vdata->noise_test_data_buf_size;
    vdata->noise_test_data_size = 0;
    noise_test_param.test_data_wr_size = &vdata->noise_test_data_size;

    open_test_param.min = &vdata->open_min;
    open_test_param.test_data_buf = vdata->open_test_data;
    open_test_param.test_data_buf_size = vdata->open_test_data_buf_size;
    vdata->open_test_data_size = 0;
    open_test_param.test_data_wr_size = &vdata->open_test_data_size;

    short_test_param.min = &vdata->short_min;
    short_test_param.test_data_buf = vdata->short_test_data;
    short_test_param.test_data_buf_size = vdata->short_test_data_buf_size;
    vdata->short_test_data_size = 0;
    short_test_param.test_data_wr_size = &vdata->short_test_data_size;

    comp_cap_test_param.min = &vdata->comp_cap_min;
    comp_cap_test_param.max = &vdata->comp_cap_max;
    comp_cap_test_param.test_data_buf = vdata->comp_cap_test_data;
    comp_cap_test_param.test_data_buf_size = vdata->comp_cap_test_data_buf_size;
    vdata->short_test_data_size = 0;
    comp_cap_test_param.test_data_wr_size = &vdata->short_test_data_size;

    if (vdata->test_reset_pin) {
        vdata->reset_pin_test_result =
            cts_test_reset_pin(cts_dev, &reset_pin_test_param);
    }
    if (vdata->test_int_pin) {
        vdata->int_pin_test_result =
            cts_test_int_pin(cts_dev, &int_pin_test_param);
    }
    if (vdata->test_rawdata) {
        vdata->rawdata_test_result =
            cts_test_rawdata(cts_dev, &rawdata_test_param);
    }
    if (vdata->test_noise) {
        vdata->noise_test_result =
            cts_test_noise(cts_dev, &noise_test_param);
    }
    if (vdata->test_open) {
        vdata->open_test_result =
            cts_test_open(cts_dev, &open_test_param);
    }
    if (vdata->test_short) {
        vdata->short_test_result =
            cts_test_short(cts_dev, &short_test_param);
    }
    if (vdata->test_comp_cap) {
        vdata->comp_cap_test_result =
            cts_test_compensate_cap(cts_dev, &comp_cap_test_param);
    }
}

static int proc_baseline_test_open(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Open proc file '"PROC_BASELINE_TEST_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Open proc file '"PROC_BASELINE_TEST_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    cts_info("Open '"PROC_BASELINE_TEST_FILEPATH"'");

	if(cts_tpmodule == 0)
	{
		ret = init_baseline_test_param_ls(vdata);
	    if (ret) {
	        cts_err("Init baseline test param failed %d(%s)",
	            ret, cts_strerror(ret));
	        return ret;
	    }
	} else if(cts_tpmodule == 1) {
		ret = init_baseline_test_param_xl(vdata);
	    if (ret) {
	        cts_err("Init baseline test param failed %d(%s)",
	            ret, cts_strerror(ret));
	        return ret;
	    }
	} else {
		cts_err("Init baseline test param failed: cts_tpmodule:%d \n", cts_tpmodule);
		return -1;
	}
    ret = alloc_baseline_test_data_mem(vdata,
        cts_data->cts_dev.fwdata.rows * cts_data->cts_dev.fwdata.cols);
    if (ret) {
        cts_err("Alloc baseline test data mem failed");
        return ret;
    }

    do_baseline_test(vdata);

    ret = save_baseline_test_data_to_file(vdata);
    if (ret) {
        cts_err("Save baseline test data to file failed %d(%s)",
            ret, cts_strerror(ret));
    }

    ret = seq_open(file, &baseline_test_seq_ops);
    if (ret) {
        cts_err("Open baseline_test seq file failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ((struct seq_file *)file->private_data)->private = cts_data;

    return 0;
}

static int proc_baseline_test_release(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Release proc file '"PROC_BASELINE_TEST_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Release proc file '"PROC_BASELINE_TEST_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    free_baseline_test_data_mem(vdata);

	return 0;
}

static const struct file_operations proc_baseline_test_fops = {
    .owner   = THIS_MODULE,
    .open    = proc_baseline_test_open,
    .release = proc_baseline_test_release,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};

static int black_screen_test_seq_show(struct seq_file *m, void *v)
{
#if 0
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int rows, cols;

    cts_data = (struct chipone_ts_data *)m->private;
    if (cts_data == NULL) {
        cts_err("Black screen test seq file private data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    rows  = cts_data->cts_dev.fwdata.rows;
    cols  = cts_data->cts_dev.fwdata.cols;

	msleep(500);

    cts_info("FW Version %04x\n\n", cts_data->cts_dev.fwdata.version);
#endif
	msleep(500);
	cts_info("black_screen_test_seq_show\n");
	seq_puts(m, "0 errors. All test passed.\n");

    return 0;
}

const struct seq_operations black_screen_test_seq_ops = {
    .start  = seq_start,
    .next   = seq_next,
    .stop   = seq_stop,
    .show   = black_screen_test_seq_show,
};
#if 0
static int init_blackscreen_test_param(struct cts_vendor_data *vdata)
{
    vdata->test_lpwg_rawdata = false;
    vdata->lpwg_rawdata_test_frames = 50;
    vdata->lpwg_rawdata_min = 10000;
    vdata->lpwg_rawdata_max = 20000;

    return 0;
}

static void do_black_screen_test(struct cts_vendor_data *vdata)
{
    cts_info("Do black screen test, TBD.");
}
#endif
static int32_t proc_black_screen_test_open(struct inode *inode, struct file *file)
{
#if 0
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Open proc file '"PROC_BLACK_SCREEN_TEST_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Open proc file '"PROC_BLACK_SCREEN_TEST_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    cts_info("Open '"PROC_BLACK_SCREEN_TEST_FILEPATH"'");

    ret = init_blackscreen_test_param(vdata);
    if (ret) {
        cts_err("Init black screen test param failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    if (!vdata->test_lpwg_rawdata) {
        cts_info("Open '"PROC_BLACK_SCREEN_TEST_FILEPATH
            "' with test_lpwg_rawdata = false");
        //return -EINVAL;
    }

    ret = alloc_black_screen_test_data_mem(vdata,
        cts_data->cts_dev.fwdata.rows * cts_data->cts_dev.fwdata.cols);
    if (ret) {
        cts_err("Alloc black screen test data mem failed");
        return ret;
    }

    do_black_screen_test(vdata);

    ret = seq_open(file, &black_screen_test_seq_ops);
    if (ret) {
        cts_err("Open black screen test seq file failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ((struct seq_file *)file->private_data)->private = cts_data;
#endif
	int ret;

	cts_info("Open '"PROC_BLACK_SCREEN_TEST_FILEPATH"'");

	ret = seq_open(file, &black_screen_test_seq_ops);
    if (ret) {
        cts_err("Open black screen test seq file failed %d",
            ret, cts_strerror(ret));
        return ret;
    }

    return 0;
}

static int proc_black_screen_test_release(struct inode *inode, struct file *file)
{
#if 0
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Release proc file '"PROC_BLACK_SCREEN_TEST_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Release proc file '"PROC_BLACK_SCREEN_TEST_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    free_black_screen_test_data_mem(vdata);
#endif
	cts_info("proc_black_screen_test_release\n");

	return 0;
}

static ssize_t proc_black_screen_test_write(struct file *filp, const char __user *buff,
			size_t len, loff_t *data)
{
	int value = 0;
	char buf[4] = {0};
 
	if ( copy_from_user(buf, buff, len) ) {
		cts_err("%s: copy from user error.", __func__);
		return -1;
	}
	sscanf(buf, "%d", &value);

	cts_info("proc_black_screen_test_write %d\n", value);

	return len;
}

static const struct file_operations proc_black_screen_test_fops = {
    .owner   = THIS_MODULE,
    .open    = proc_black_screen_test_open,
    .release = proc_black_screen_test_release,
    .read    = seq_read,
    .write   = proc_black_screen_test_write,
    .llseek  = seq_lseek,
    .release = seq_release,
};

static ssize_t proc_coordinate_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_COORDINATE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_COORDINATE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_COORDINATE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_COORDINATE_FILEPATH"'");

    ret = copy_to_user(buffer, vdata->coordinate, vdata->coordinate_len);
    if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
        return 0;
    }

    return vdata->coordinate_len;
}
static const struct file_operations proc_coordinate_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read  = proc_coordinate_read,
};

static ssize_t proc_debug_level_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char debug_level_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_DEBUG_LEVEL_FILEPATH"'");

	len = scnprintf(debug_level_str, sizeof(debug_level_str),
	    "DEBUG_LEVEL = %d\n", vdata->debug_level);
	*ppos += len;

    ret = copy_to_user(buffer, debug_level_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

	return len;
}

static ssize_t proc_debug_level_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int debug_level, ret;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Write proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Write proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Write proc file '"PROC_DEBUG_LEVEL_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

    cts_info("Write '"PROC_DEBUG_LEVEL_FILEPATH"' size: %zu", size);

	ret = kstrtoint_from_user(buffer, size, 0, &debug_level);
	if (ret) {
        cts_err("Write invalid DEBUG_LEVEL %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
	}

    vdata->debug_level = debug_level;
    cts_info("Set DEBUG_LEVEL = %d", debug_level);

	return size;
}
static const struct file_operations proc_debug_level_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read  = proc_debug_level_read,
    .write = proc_debug_level_write,
};

static ssize_t proc_double_tap_enable_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char double_tap_enable_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_DOUBLE_TAP_ENABLE_FILEPATH"'");

	len = scnprintf(double_tap_enable_str, sizeof(double_tap_enable_str),
	    "DOUBLE_TAP_ENABLE = %d\n", vdata->double_tap_enable);
	*ppos += len;

    ret = copy_to_user(buffer, double_tap_enable_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

	return len;
}

static ssize_t proc_double_tap_enable_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int double_tap_enable, ret;

#if !defined(CFG_CTS_GESTURE)
    cts_err("Set DOUBLE_TAP_ENABLE while CFG_CTS_GESTURE not defined");
    return -EFAULT;
#endif /* CFG_CTS_GESTURE */

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Write proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Write proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Write proc file '"PROC_DOUBLE_TAP_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

    cts_info("Write '"PROC_DOUBLE_TAP_ENABLE_FILEPATH"' size: %zu", size);

    ret = kstrtoint_from_user(buffer, size, 0, &double_tap_enable);
    if (ret) {
        cts_err("Write invalid DOUBLE_TAP_ENABLE %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
    }

    vdata->double_tap_enable = double_tap_enable;
    cts_info("Set DOUBLE_TAP_ENABLE = %d", double_tap_enable);

#ifdef CFG_CTS_GESTURE
    if (double_tap_enable) {
        cts_enable_gesture_wakeup(&cts_data->cts_dev);
    } else {
        cts_disable_gesture_wakeup(&cts_data->cts_dev);
    }
#endif /* CFG_CTS_GESTURE */

    return size;
}

static const struct file_operations proc_double_tap_enable_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read  = proc_double_tap_enable_read,
	.write = proc_double_tap_enable_write,
};

static ssize_t proc_game_switch_enable_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char game_switch_enable_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_GAME_SWITCH_ENABLE_FILEPATH"'");

	len = scnprintf(game_switch_enable_str, sizeof(game_switch_enable_str),
	    "GAME_SWITCH_ENABLE = %d\n", vdata->game_switch_enable);
	*ppos += len;

    ret = copy_to_user(buffer, game_switch_enable_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

    return len;
}

static ssize_t proc_game_switch_enable_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int game_switch_enable, ret;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Write proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Write proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Write proc file '"PROC_GAME_SWITCH_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

    cts_info("Write '"PROC_GAME_SWITCH_ENABLE_FILEPATH"' size: %zu", size);

    ret = kstrtoint_from_user(buffer, size, 0, &game_switch_enable);
    if (ret) {
        cts_err("Write invalid GAME_SWITCH_ENABLE %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
    }

    vdata->game_switch_enable = game_switch_enable;
    cts_info("Set GAME_SWITCH_ENABLE = %d", game_switch_enable);

    ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x086E,
        game_switch_enable ? 1 : 0);
    if (ret) {
        cts_err("Set dev game mode failed %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
    }

    return size;

}
static const struct file_operations proc_game_switch_enable_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
	.read  = proc_game_switch_enable_read,
	.write = proc_game_switch_enable_write,
};

static ssize_t proc_incell_panel_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char incell_panel_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_INCELL_PANEL_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_INCELL_PANEL_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_INCELL_PANEL_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_INCELL_PANEL_FILEPATH"'");

	len = scnprintf(incell_panel_str, sizeof(incell_panel_str),
	    "INCELL_PANEL = 1\n");
	*ppos += len;

    ret = copy_to_user(buffer, incell_panel_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

    return len;
}
static const struct file_operations proc_incell_panel_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read = proc_incell_panel_read,
};

static ssize_t proc_irq_depth_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct irq_desc *desc;
    char irq_depth_str[128];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_IRQ_DEPTH_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_IRQ_DEPTH_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_IRQ_DEPTH_FILEPATH"'");

    desc = irq_to_desc(cts_data->pdata->irq);
    if (desc == NULL) {
        len = scnprintf(irq_depth_str, sizeof(irq_depth_str),
            "IRQ: %d descriptor not found\n",
            cts_data->pdata->irq);
    } else {
        len = scnprintf(irq_depth_str, sizeof(irq_depth_str),
            "IRQ num: %d, depth: %u, "
            "count: %u, unhandled: %u, last unhandled eslape: %lu\n",
            cts_data->pdata->irq, desc->depth,
            desc->irq_count, desc->irqs_unhandled,
            desc->last_unhandled);
    }
	*ppos += len;

    ret = copy_to_user(buffer, irq_depth_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

    return len;
}
static const struct file_operations proc_irq_depth_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read = proc_irq_depth_read,
};

static ssize_t proc_oplus_register_info_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    return 0;
}

/* echo r/w fw/hw addr len/val0 val1 val2 ... */
static ssize_t proc_oplus_register_info_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    int parse_arg(const char *buf, size_t count);

    parse_arg(buffer, size);

    return size;
}
static const struct file_operations proc_oplus_register_info_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read  = proc_oplus_register_info_read,
	.write = proc_oplus_register_info_write,
};

static ssize_t proc_oplus_tp_direction_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char oplus_tp_direction_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_OPLUS_TP_DIRECTION_FILEPATH"'");

	len = scnprintf(oplus_tp_direction_str, sizeof(oplus_tp_direction_str),
	    "OPLUS_TP_DIRECTION = %d\n", vdata->oplus_tp_direction);
	*ppos += len;

    ret = copy_to_user(buffer, oplus_tp_direction_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

	return len;
}

static ssize_t proc_oplus_tp_direction_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret = 0;
    u32 oplus_tp_direction = 0;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_DIRECTION_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

    cts_info("Write '"PROC_OPLUS_TP_DIRECTION_FILEPATH"' size: %zu", size);

	ret = kstrtoint_from_user(buffer, size, 0, &oplus_tp_direction);
	if (ret) {
        cts_err("Write invalid OPLUS_TP_DIRECTION %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
	}

    switch (oplus_tp_direction) {
        case 0:
        case 1:
        case 2:
            ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x0801,
                (u8)oplus_tp_direction);
            if (ret) {
                cts_err("Write tp direction firmware failed %d(%d)",
                    ret, cts_strerror(ret));
                return ret;
            }
            break;
        default:
            cts_err("Write invalid OPLUS_TP_DIRECTION %d",
                oplus_tp_direction);
            return -EINVAL;
    }


    vdata->oplus_tp_direction = oplus_tp_direction;
    cts_info("Set OPLUS_TP_DIRECTION = %d", oplus_tp_direction);

	return size;
}
static const struct file_operations proc_oplus_tp_direction_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
	.read  = proc_oplus_tp_direction_read,
	.write = proc_oplus_tp_direction_write,
};

static ssize_t proc_oplus_tp_limit_enable_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    char oplus_tp_limit_enable_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH"'");

	len = scnprintf(oplus_tp_limit_enable_str, sizeof(oplus_tp_limit_enable_str),
	    "OPLUS_TP_LIMIT_ENABLE = %d\n", vdata->oplus_tp_limit_enable);
	*ppos += len;

    ret = copy_to_user(buffer, oplus_tp_limit_enable_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

	return len;
}

static ssize_t proc_oplus_tp_limit_enable_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;
    int oplus_tp_limit_enable, ret;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Write proc file '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

    cts_info("Write '"PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH"' size: %zu", size);

	ret = kstrtoint_from_user(buffer, size, 0, &oplus_tp_limit_enable);
	if (ret) {
        cts_err("Write invalid OPLUS_TP_LIMIT_ENABLE %d(%s)",
            ret, cts_strerror(ret));
        return -EINVAL;
	}

    switch (oplus_tp_limit_enable) {
        case 0:
        case 1:
        case 2:
            ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x0801,
                (u8)oplus_tp_limit_enable);
            if (ret) {
                cts_err("Write tp limit to firmware failed %d(%d)",
                    ret, cts_strerror(ret));
                return ret;
            }
            break;
        default:
            cts_err("Write invalid OPLUS_TP_LIMIT_ENABLE %d",
                oplus_tp_limit_enable);
            return -EINVAL;
    }

    vdata->oplus_tp_limit_enable = oplus_tp_limit_enable;
    cts_info("Set OPLUS_TP_LIMIT_ENABLE = %d", oplus_tp_limit_enable);

	return size;
}

static const struct file_operations proc_oplus_tp_limit_enable_fops =
{
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
    .read  = proc_oplus_tp_limit_enable_read,
    .write = proc_oplus_tp_limit_enable_write,
};

static ssize_t proc_tp_fw_update_write(struct file *filp,
    const char __user *buffer, size_t size, loff_t *ppos)
{
    //TODO:
    return size;
}
static const struct file_operations proc_tp_fw_update_fops = {
    .owner = THIS_MODULE,
    .open  = proc_simple_open,
	.write = proc_tp_fw_update_write,
};

static int baseline_seq_show(struct seq_file *m, void *v)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = (struct chipone_ts_data *)m->private;
    if (cts_data == NULL) {
        cts_err("Baseline seq file private data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Baseline seq file with vdata = NULL");
        return -EFAULT;
    }

    if (vdata->rawdata == NULL) {
        cts_err("Baseline seq file with vdata->rawdata = NULL");
        return -EFAULT;
    }

    dump_touch_data_to_seq_file(m, vdata->rawdata,
        cts_data->cts_dev.fwdata.rows, cts_data->cts_dev.fwdata.cols);

    return 0;
}

const struct seq_operations baseline_seq_ops = {
    .start  = seq_start,
    .next   = seq_next,
    .stop   = seq_stop,
    .show   = baseline_seq_show,
};

static int proc_baseline_open(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_device *cts_dev = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Open proc file '"PROC_BASELINE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    cts_dev = &cts_data->cts_dev;

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Open proc file '"PROC_BASELINE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    cts_info("Open '"PROC_BASELINE_FILEPATH"'");

    vdata->rawdata = kzalloc(2 * cts_dev->fwdata.rows * cts_dev->fwdata.cols,
        GFP_KERNEL);
    if (vdata->rawdata == NULL) {
        cts_err("Alloc mem for rawdata failed");
        return -ENOMEM;
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read touch data failed %d(%s)",
            ret, cts_strerror(ret));
        goto unlock_device_and_return;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            ret, cts_strerror(ret));
        goto disable_get_touch_data_and_return;
    }
    msleep(50);

    ret = cts_get_rawdata(cts_dev, vdata->rawdata);
    if(ret) {
        cts_err("Get rawdata failed %d(%s)", ret, cts_strerror(ret));
        goto disable_get_touch_data_and_return;
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read touch data failed %d(%s)",
            ret, cts_strerror(ret));
        // TODO: Try to recovery
    }
    cts_unlock_device(cts_dev);

    ret = seq_open(file, &baseline_seq_ops);
    if (ret) {
        cts_err("Open baseline seq file failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ((struct seq_file *)file->private_data)->private = cts_data;

    return 0;

disable_get_touch_data_and_return: {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable read touch data failed %d(%s)",
                ret, cts_strerror(ret));
        }
    }
unlock_device_and_return:
    cts_unlock_device(cts_dev);
    if (vdata->rawdata) {
        kfree(vdata->rawdata);
        vdata->rawdata = NULL;
    }

    return ret;
}

static int proc_baseline_release(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Release proc file '"PROC_BASELINE_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Release proc file '"PROC_BASELINE_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if(vdata->rawdata) {
        kfree(vdata->rawdata);
        vdata->rawdata = NULL;
    }

	return 0;
}

static const struct file_operations proc_baseline_fops = {
    .owner   = THIS_MODULE,
    .open    = proc_baseline_open,
    .release = proc_baseline_release,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};

static ssize_t proc_data_limit_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    return 0;
}
static const struct file_operations proc_data_limit_fops = {
	.owner = THIS_MODULE,
    .open  = proc_simple_open,
	.read = proc_data_limit_read,
};

static int delta_seq_show(struct seq_file *m, void *v)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = (struct chipone_ts_data *)m->private;
    if (cts_data == NULL) {
        cts_err("Delta seq file private data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Delta seq file with vdata = NULL");
        return -EFAULT;
    }

    if (vdata->delta == NULL) {
        cts_err("Delta seq file with vdata->delta = NULL");
        return -EFAULT;
    }

    dump_touch_data_to_seq_file(m, vdata->delta,
        cts_data->cts_dev.fwdata.rows, cts_data->cts_dev.fwdata.cols);

    return 0;
}

const struct seq_operations delta_seq_ops = {
    .start  = seq_start,
    .next   = seq_next,
    .stop   = seq_stop,
    .show   = delta_seq_show,
};

static int proc_delta_open(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_device *cts_dev = NULL;
    struct cts_vendor_data *vdata = NULL;
    int ret;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Open proc file '"PROC_DELTA_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    cts_dev = &cts_data->cts_dev;

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Open proc file '"PROC_DELTA_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    cts_info("Open '"PROC_DELTA_FILEPATH"'");

    vdata->delta = kzalloc(2 * cts_dev->fwdata.rows * cts_dev->fwdata.cols,
        GFP_KERNEL);
    if (vdata->delta == NULL) {
        cts_err("Alloc mem for delta failed");
        return -ENOMEM;
    }

    cts_lock_device(cts_dev);
    ret = cts_enable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Enable read touch data failed %d(%s)",
            ret, cts_strerror(ret));
        goto unlock_device_and_return;
    }

    ret = cts_send_command(cts_dev, CTS_CMD_QUIT_GESTURE_MONITOR);
    if (ret) {
        cts_err("Send cmd QUIT_GESTURE_MONITOR failed %d(%s)",
            ret, cts_strerror(ret));
        goto disable_get_touch_data_and_return;
    }
    msleep(50);

    ret = cts_get_diffdata(cts_dev, vdata->delta);
    if(ret) {
        cts_err("Get diffdata failed %d(%s)", ret, cts_strerror(ret));
        goto disable_get_touch_data_and_return;
    }
    ret = cts_disable_get_rawdata(cts_dev);
    if (ret) {
        cts_err("Disable read touch data failed %d(%s)",
            ret, cts_strerror(ret));
        // TODO: Try to recovery
    }
    cts_unlock_device(cts_dev);

    ret = seq_open(file, &delta_seq_ops);
    if (ret) {
        cts_err("Open delta seq file failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ((struct seq_file *)file->private_data)->private = cts_data;

    return 0;

disable_get_touch_data_and_return: {
        int r = cts_disable_get_rawdata(cts_dev);
        if (r) {
            cts_err("Disable read touch data failed %d(%s)",
                ret, cts_strerror(ret));
        }
    }
unlock_device_and_return:
    cts_unlock_device(cts_dev);
    if (vdata->delta) {
        kfree(vdata->delta);
        vdata->delta = NULL;
    }

    return ret;
}

static int proc_delta_release(struct inode *inode, struct file *file)
{
    struct chipone_ts_data *cts_data = NULL;
    struct cts_vendor_data *vdata = NULL;

    cts_data = PDE_DATA(inode);
    if (cts_data == NULL) {
        cts_err("Release proc file '"PROC_DELTA_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    vdata = cts_data->vendor_data;
    if (vdata == NULL) {
        cts_err("Release proc file '"PROC_DELTA_FILEPATH
                "' with vdata = NULL");
        return -EFAULT;
    }

    if (vdata->delta) {
        kfree(vdata->delta);
        vdata->delta = NULL;
    }

	return 0;
}

static const struct file_operations proc_delta_fops = {
    .owner   = THIS_MODULE,
    .open    = proc_delta_open,
    .release = proc_delta_release,
    .read    = seq_read,
    .llseek  = seq_lseek,
    .release = seq_release,
};

static ssize_t proc_main_register_read(struct file *filp,
    char __user *buffer, size_t size, loff_t *ppos)
{
    struct chipone_ts_data *cts_data = NULL;
    char main_register_str[32];
    int ret, len;

    cts_data = (struct chipone_ts_data *)filp->private_data;
    if (cts_data == NULL) {
        cts_err("Read proc file '"PROC_MAIN_REGISTER_FILEPATH
                "' with cts_data = NULL");
        return -EFAULT;
    }

    if (buffer == NULL) {
        cts_err("Read proc file '"PROC_MAIN_REGISTER_FILEPATH
                "' with buffer = NULL");
        return -EINVAL;
    }

	if (*ppos != 0) {
	    return 0;
    }

    cts_info("Read '"PROC_MAIN_REGISTER_FILEPATH"'");

	len = scnprintf(main_register_str, sizeof(main_register_str),
	    "FW_VERSION = %04x\n", cts_data->cts_dev.fwdata.version);
	*ppos += len;

    ret = copy_to_user(buffer, main_register_str, len);
	if (ret) {
        cts_err("Copy data to user buffer failed %d(%s)",
            ret, cts_strerror(ret));
		return 0;
    }

	return len;
}
static const struct file_operations proc_main_register_fops = {
	.owner = THIS_MODULE,
	.open  = proc_simple_open,
	.read = proc_main_register_read,
};

struct proc_file_create_data {
    const char *filename;
    const char *filepath;
    const struct file_operations *fops;
};

struct proc_dir_create_data {
    const char *dirname;
    const char *dirpath;
    int nfiles;
    const struct proc_file_create_data *files;
};

static const struct proc_file_create_data proc_dir_debuginfo_files_create_data[] = {
    {
        .filename = PROC_BASELINE_FILENAME,
        .filepath = PROC_BASELINE_FILEPATH,
        .fops = &proc_baseline_fops,
    },
    {
        .filename = PROC_DATA_LIMIT_FILENAME,
        .filepath = PROC_DATA_LIMIT_FILEPATH,
        .fops = &proc_data_limit_fops,
    },
    {
        .filename = PROC_DELTA_FILENAME,
        .filepath = PROC_DELTA_FILEPATH,
        .fops = &proc_delta_fops,
    },
    {
        .filename = PROC_MAIN_REGISTER_FILENAME,
        .filepath = PROC_MAIN_REGISTER_FILEPATH,
        .fops = &proc_main_register_fops,
    },
};

static const struct proc_dir_create_data proc_dir_debuginfo_create_data = {
    .dirname = PROC_DEBUG_INFO_DIR_NAME,
    .dirpath = PROC_DEBUG_INFO_DIR_PATH,
    .nfiles = ARRAY_SIZE(proc_dir_debuginfo_files_create_data),
    .files = proc_dir_debuginfo_files_create_data,
};

static const struct proc_file_create_data proc_dir_touchpanel_files_create_data [] = {
    {
        .filename = PROC_BASELINE_TEST_FILENAME,
        .filepath = PROC_BASELINE_TEST_FILEPATH,
        .fops = &proc_baseline_test_fops,
    },
    {
        .filename = PROC_BLACK_SCREEN_TEST_FILENAME,
        .filepath = PROC_BLACK_SCREEN_TEST_FILEPATH,
        .fops = &proc_black_screen_test_fops,
    },
    {
        .filename = PROC_COORDINATE_FILENAME,
        .filepath = PROC_COORDINATE_FILEPATH,
        .fops = &proc_coordinate_fops,
    },
    {
        .filename = PROC_DEBUG_LEVEL_FILENAME,
        .filepath = PROC_DEBUG_LEVEL_FILEPATH,
        .fops = &proc_debug_level_fops,
    },
    {
        .filename = PROC_DOUBLE_TAP_ENABLE_FILENAME,
        .filepath = PROC_DOUBLE_TAP_ENABLE_FILEPATH,
        .fops = &proc_double_tap_enable_fops,
    },
    {
        .filename = PROC_GAME_SWITCH_ENABLE_FILENAME,
        .filepath = PROC_GAME_SWITCH_ENABLE_FILEPATH,
        .fops = &proc_game_switch_enable_fops,
    },
    {
        .filename = PROC_INCELL_PANEL_FILENAME,
        .filepath = PROC_INCELL_PANEL_FILEPATH,
        .fops = &proc_incell_panel_fops,
    },
    {
        .filename = PROC_IRQ_DEPTH_FILENAME,
        .filepath = PROC_IRQ_DEPTH_FILEPATH,
        .fops = &proc_irq_depth_fops,
    },
    {
        .filename = PROC_OPLUS_REGISTER_INFO_FILENAME,
        .filepath = PROC_OPLUS_REGISTER_INFO_FILEPATH,
        .fops = &proc_oplus_register_info_fops,
    },
    {
        .filename = PROC_OPLUS_TP_DIRECTION_FILENAME,
        .filepath = PROC_OPLUS_TP_DIRECTION_FILEPATH,
        .fops = &proc_oplus_tp_direction_fops,
    },
    {
        .filename = PROC_OPLUS_TP_LIMIT_ENABLE_FILENAME,
        .filepath = PROC_OPLUS_TP_LIMIT_ENABLE_FILEPATH,
        .fops = &proc_oplus_tp_limit_enable_fops,
    },
    {
        .filename = PROC_TP_FW_UPDATE_FILENAME,
        .filepath = PROC_TP_FW_UPDATE_FILEPATH,
        .fops = &proc_tp_fw_update_fops,
    },
};

static const struct proc_dir_create_data proc_dir_touchpanel_create_data = {
    .dirname = PROC_TOUCHPANEL_DIR_NAME,
    .dirpath = PROC_TOUCHPANEL_DIR_PATH,
    .nfiles = ARRAY_SIZE(proc_dir_touchpanel_files_create_data),
    .files = proc_dir_touchpanel_files_create_data,
};

static int create_proc_file(struct cts_vendor_data *vdata,
    struct proc_dir_entry *parent,
    const struct proc_file_create_data *create_data)
{
    struct proc_dir_entry *file_entry;
    umode_t mode = 0044;

    if (create_data == NULL) {
        cts_err("Create proc file with create_data = NULL");
        return -EINVAL;
    }
    if (create_data->fops->read == NULL &&
        create_data->fops->write == NULL) {
        cts_err("Create proc file '%s' both ops->read & ops->write = NULL",
            create_data->filepath);
        return -EINVAL;
    }

    if(create_data->fops->read) {
        mode |= S_IRUSR;
    }
    if(create_data->fops->write) {
        mode |= (S_IWUSR | S_IWUGO);
    }

    cts_info("Create proc file '%s', mode: 0%o",
        create_data->filepath, mode);

    file_entry = proc_create_data(create_data->filename, mode,
        parent, create_data->fops, vdata->cts_data);
    if (file_entry == NULL) {
        cts_err("Create proc file '%s' failed", create_data->filename);
        return -EIO;
    }

    return 0;
}

static struct proc_dir_entry * create_proc_dir(struct cts_vendor_data *vdata,
     struct proc_dir_entry *parent,
     const struct proc_dir_create_data *create_data)
{
    struct proc_dir_entry *dir_entry;
    int i, ret;

    if (create_data == NULL) {
        cts_err("Create proc file with create_data = NULL");
        return ERR_PTR(-EINVAL);
    }

    cts_info("Create proc dir '%s' include %d files parent %p",
        create_data->dirpath, create_data->nfiles, parent);

    dir_entry = proc_mkdir(create_data->dirname, parent);
    if (dir_entry == NULL) {
        cts_err("Create proc dir '%s' failed", create_data->dirpath);
        return ERR_PTR(-EIO);
    }

    for (i = 0; i < create_data->nfiles; i++) {
        ret = create_proc_file(vdata, dir_entry, &create_data->files[i]);
        if (ret) {
            cts_err("Create proc file '%s' failed %d(%s)",
                create_data->files[i].filepath, ret, cts_strerror(ret));
            /* Ignore ??? */

            /* Roll back */
            remove_proc_subtree(create_data->dirname, parent);

            return ERR_PTR(ret);
        }
    }

    return dir_entry;
}
#endif /* CONFIG_PROC_FS */

static const char *headset_state_str(bool state)
{
    return state ? "ATTACHED" : "DETACHED";
}

static void set_dev_headset_state_work(struct work_struct *work)
{
    struct cts_vendor_data *vdata;
    int ret;

    vdata = container_of(work, struct cts_vendor_data,
        set_headset_state_work);

    cts_lock_device(&vdata->cts_data->cts_dev);
    ret = cts_set_dev_earjack_attached(&vdata->cts_data->cts_dev,
        vdata->headset_state);
    cts_unlock_device(&vdata->cts_data->cts_dev);
    if (ret) {
        cts_err("Set dev headset state to %s failed %d(%s)",
            headset_state_str(vdata->headset_state),
            ret, cts_strerror(ret));
        /* Set to previous state, try set again in next loop */
        vdata->headset_state = !vdata->headset_state;
    }
}

static int headset_notify_callback(struct notifier_block *nb,
                               unsigned long state, void *data)
{
    struct cts_vendor_data *vdata;
    bool   prev_state;

    if (nb == NULL) {
        cts_err("Headset notify with notifier block = NULL");
        return NOTIFY_DONE;
    }

    vdata = container_of(nb, struct cts_vendor_data, headset_notifier);

    prev_state = vdata->headset_state;
    vdata->headset_state = state;

    if (vdata->headset_state != prev_state) {
        cts_info("Headset state changed: %s -> %s",
            headset_state_str(prev_state),
            headset_state_str(vdata->headset_state));

        if (cts_is_device_enabled(&vdata->cts_data->cts_dev)) {
            if (!queue_work(vdata->cts_data->workqueue,
                    &vdata->set_headset_state_work)) {
                    cts_warn("Set device headset state work is PENDING");
            }
        }
    }

    return NOTIFY_OK;
}

static const char *usb_state_str(bool state)
{
    return state ? "ATTACHED" : "DETACHED";
}

static void set_dev_usb_state_work(struct work_struct *work)
{
    struct cts_vendor_data *vdata;
    int ret;

    vdata = container_of(work, struct cts_vendor_data,
        set_usb_state_work);

    cts_lock_device(&vdata->cts_data->cts_dev);
    ret = cts_set_dev_charger_attached(&vdata->cts_data->cts_dev,
        vdata->usb_state);
    cts_unlock_device(&vdata->cts_data->cts_dev);
    if (ret) {
        cts_err("Set dev USB state to %s failed %d(%s)",
            usb_state_str(vdata->headset_state),
            ret, cts_strerror(ret));
        /* Set to previous state, try set again in next loop */
        vdata->usb_state = !vdata->usb_state;
    }
}

static int usb_notify_callback(struct notifier_block *nb,
                           unsigned long value, void *data)
{
    struct cts_vendor_data *vdata;
    bool   prev_state;

    if (nb == NULL) {
        cts_err("USB notify with notifier block = NULL");
        return NOTIFY_DONE;
    }

    vdata = container_of(nb, struct cts_vendor_data, usb_notifier);

    prev_state = vdata->usb_state;
    vdata->usb_state = value;

    if (vdata->usb_state != prev_state) {
        cts_info("State changed: %s -> %s",
            usb_state_str(prev_state),
            usb_state_str(vdata->usb_state));

        if (cts_is_device_enabled(&vdata->cts_data->cts_dev)) {
            if (!queue_work(vdata->cts_data->workqueue,
                &vdata->set_usb_state_work)) {
                cts_warn("Set device USB state work is PENDING");
            }
        }
    }

    return NOTIFY_OK;
}

int cts_vendor_init(struct chipone_ts_data *cts_data)
{
    struct cts_vendor_data *vdata = NULL;
    int ret;

    if (cts_data == NULL) {
        cts_err("Init with cts_data = NULL");
        return -EINVAL;
    }

    cts_info("Init");

    cts_data->vendor_data = NULL;

    vdata = kzalloc(sizeof(*vdata), GFP_KERNEL);
    if (vdata == NULL) {
        cts_err("Alloc vendor data failed");
        return -ENOMEM;
    }

    cts_data->vendor_data = vdata;
	vdata->cts_data = cts_data;

#ifdef CONFIG_PROC_FS
    vdata->proc_dir_touchpanel_entry = create_proc_dir(vdata,
        NULL, &proc_dir_touchpanel_create_data);
    if (IS_ERR_OR_NULL(vdata->proc_dir_touchpanel_entry)) {
        ret = PTR_ERR(vdata->proc_dir_touchpanel_entry);
        vdata->proc_dir_touchpanel_entry = NULL;
        cts_err("Create proc dir '"PROC_TOUCHPANEL_DIR_PATH"' failed %d(%s)",
            ret, cts_strerror(ret));
        goto free_vendor_data;
    }
    vdata->proc_dir_debuginfo_entry = create_proc_dir(vdata,
        vdata->proc_dir_touchpanel_entry,
        &proc_dir_debuginfo_create_data);
    if (IS_ERR_OR_NULL(vdata->proc_dir_debuginfo_entry)) {
        ret = PTR_ERR(vdata->proc_dir_debuginfo_entry);
        vdata->proc_dir_debuginfo_entry = NULL;
        cts_err("Create proc dir '"PROC_DEBUG_INFO_DIR_PATH"' failed %d(%s)",
            ret, cts_strerror(ret));
        goto remove_proc_dir_touchpanel;
    }
#endif /* CONFIG_PROC_FS */

    /* Headset */
    INIT_WORK(&vdata->set_headset_state_work, set_dev_headset_state_work);

    vdata->headset_notifier.notifier_call = headset_notify_callback;
    ret = headset_register_client(&vdata->headset_notifier);
    if (ret) {
        cts_err("Register headset notifier failed: %d(%s)",
            ret, cts_strerror(ret));
        // Ignore this error
    }
    vdata->headset_notifier_registered = true;

    /* USB */
    INIT_WORK(&vdata->set_usb_state_work, set_dev_usb_state_work);

    vdata->usb_notifier.notifier_call = usb_notify_callback;
    ret = tp_usb_register_client(&vdata->usb_notifier);
    if (ret) {
        cts_err("Register usb notifier failed: %d(%s)",
            ret, cts_strerror(ret));
        // Ignore this error
    }
    vdata->usb_notifier_registered = true;

    cts_data->vendor_data = vdata;
	vdata->cts_data = cts_data;

    return 0;

#ifdef CONFIG_PROC_FS
remove_proc_dir_touchpanel:
    if (vdata->proc_dir_touchpanel_entry) {
        if (vdata->proc_dir_debuginfo_entry) {
            cts_info("Remove proc dir '"PROC_DEBUG_INFO_DIR_PATH"'");
            remove_proc_subtree(PROC_DEBUG_INFO_DIR_NAME,
                vdata->proc_dir_touchpanel_entry);
            vdata->proc_dir_debuginfo_entry = NULL;
        }

        cts_info("Remove proc dir '"PROC_TOUCHPANEL_DIR_PATH"'");
        remove_proc_subtree(PROC_TOUCHPANEL_DIR_NAME, NULL);
        vdata->proc_dir_touchpanel_entry = NULL;
    }

free_vendor_data:
    kfree(vdata);
    cts_data->vendor_data = NULL;

    return ret;
#endif /* CONFIG_PROC_FS */
}

int cts_vendor_deinit(struct chipone_ts_data *cts_data)
{
    struct cts_vendor_data *vdata = NULL;

    if (cts_data == NULL) {
        cts_err("Deinit with cts_data = NULL");
        return -EINVAL;
    }

    if (cts_data->vendor_data == NULL) {
        cts_warn("Deinit with vendor_data = NULL");
        return -EINVAL;
    }

    cts_info("Deinit");

    vdata = cts_data->vendor_data;

#ifdef CONFIG_PROC_FS
    if (vdata->proc_dir_touchpanel_entry) {
        if (vdata->proc_dir_debuginfo_entry) {
            cts_info("Remove proc dir '"PROC_DEBUG_INFO_DIR_PATH"'");
            remove_proc_subtree(PROC_DEBUG_INFO_DIR_NAME,
                vdata->proc_dir_touchpanel_entry);
            vdata->proc_dir_debuginfo_entry = NULL;
        }

        cts_info("Remove proc dir '"PROC_TOUCHPANEL_DIR_PATH"'");
        remove_proc_subtree(PROC_TOUCHPANEL_DIR_NAME, NULL);
        vdata->proc_dir_touchpanel_entry = NULL;
    }

    free_baseline_test_data_mem(vdata);
    //free_black_screen_test_data_mem(vdata);
    if (vdata->rawdata) {
        kfree(vdata->rawdata);
        vdata->rawdata = NULL;
    }
    if (vdata->delta) {
        kfree(vdata->delta);
        vdata->delta = NULL;
    }
#endif /* CONFIG_PROC_FS */

    kfree(cts_data->vendor_data);
    cts_data->vendor_data = NULL;

    return 0;
}

int cts_vendor_config_firmware(struct chipone_ts_data *cts_data)
{   
    int ret;
	struct cts_vendor_data *vdata = NULL;

    if (cts_data == NULL) {
        cts_err("Config firmware with cts_data = NULL");
        return -EINVAL;
    }

    if (cts_data->vendor_data == NULL) {
        cts_warn("Config firmware with vendor_data = NULL");
        return -EINVAL;
    }

	vdata = cts_data->vendor_data;
    cts_info("Config firmware");

    ret = cts_set_dev_charger_attached(&cts_data->cts_dev, vdata->usb_state);
    if (ret) {
        cts_err("Config firmware USB state %s failed %d(%s)",
            usb_state_str(vdata->usb_state), ret, cts_strerror(ret));
        // Ignore this error
    }

    ret = cts_set_dev_earjack_attached(&cts_data->cts_dev, vdata->headset_state);
    if (ret) {
        cts_err("Config firmware headset state %s failed %d(%s)",
            headset_state_str(vdata->usb_state), ret, cts_strerror(ret));
        // Ignore this error
    }

    ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x086E,
        vdata->game_switch_enable ? 1 : 0);
    if (ret) {
        cts_err("Config firmware game mode failed %d(%s)",
            ret, cts_strerror(ret));
        return ret;
    }

    ret = cts_fw_reg_writeb(&cts_data->cts_dev, 0x0801,
        (u8)vdata->oplus_tp_direction);
    if (ret) {
        cts_err("Config firmware tp direction failed %d(%d)",
            ret, cts_strerror(ret));
        return ret;
    }

    return 0;
}

