#ifndef CTS_VENDOR_H
#define CTS_VENDOR_H

struct chipone_ts_data;

extern int cts_vendor_init(struct chipone_ts_data *cts_data);
extern int cts_vendor_deinit(struct chipone_ts_data *cts_data);
extern int cts_vendor_config_firmware(struct chipone_ts_data *cts_data);

#endif /* CTS_VENDOR_H */

