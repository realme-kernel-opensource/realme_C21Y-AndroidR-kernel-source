/*
 * Copyright (C) 2015 Spreadtrum Communications Inc.
 *
 * Authors	:
 * Baolei Yuan <baolei.yuan@spreadtrum.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __SPRDWL_VENDOR_H__
#define __SPRDWL_VENDOR_H__

#include <net/netlink.h>
#include <net/cfg80211.h>
#include <linux/ctype.h>

#define OUI_SPREAD 0x001374
#define EXTRA_LEN  500
#define VENDOR_SCAN_RESULT_EXPIRE	(7 * HZ)

#define WIFI_FEATURE_INFRA              0x0001
#define WIFI_FEATURE_INFRA_5G           0x0002
#define WIFI_FEATURE_HOTSPOT            0x0004
#define WIFI_FEATURE_P2P                0x0008
#define WIFI_FEATURE_SOFT_AP            0x0010
#define WIFI_FEATURE_GSCAN              0x0020
#define WIFI_FEATURE_NAN                0x0040
#define WIFI_FEATURE_D2D_RTT            0x0080
#define WIFI_FEATURE_D2AP_RTT           0x0100
#define WIFI_FEATURE_BATCH_SCAN         0x0200
#define WIFI_FEATURE_PNO                0x0400
#define WIFI_FEATURE_ADDITIONAL_STA     0x0800
#define WIFI_FEATURE_TDLS               0x1000
#define WIFI_FEATURE_TDLS_OFFCHANNEL    0x2000
#define WIFI_FEATURE_EPR                0x4000
#define WIFI_FEATURE_AP_STA             0x8000
#define WIFI_FEATURE_LINK_LAYER_STATS   0x10000
#define WIFI_FEATURE_LOGGER             0x20000
#define WIFI_FEATURE_HAL_EPNO           0x40000
#define WIFI_FEATURE_RSSI_MONITOR       0x80000
#define WIFI_FEATURE_MKEEP_ALIVE        0x100000
#define WIFI_FEATURE_CONFIG_NDO         0x200000
#define WIFI_FEATURE_TX_TRANSMIT_POWER  0x400000
#define WIFI_FEATURE_CONTROL_ROAMING    0x800000
#define WIFI_FEATURE_IE_WHITELIST       0x1000000
#define WIFI_FEATURE_SCAN_RAND          0x2000000

/*reinit acs*/
#define SPRDWL_REINIT_ACS	0x35
enum {
	/* Memory dump of FW */
	WIFI_LOGGER_MEMORY_DUMP_SUPPORTED = (1 << (0)),
	/*PKT status*/
	WIFI_LOGGER_PER_PACKET_TX_RX_STATUS_SUPPORTED = (1 << (1)),
	/*Connectivity event*/
	WIFI_LOGGER_CONNECT_EVENT_SUPPORTED = (1 << (2)),
	/* POWER of Driver */
	WIFI_LOGGER_POWER_EVENT_SUPPORTED = (1 << (3)),
	/*WAKE LOCK of Driver*/
	WIFI_LOGGER_WAKE_LOCK_SUPPORTED = (1 << (4)),
	/*verbose log of FW*/
	WIFI_LOGGER_VERBOSE_SUPPORTED = (1 << (5)),
	/*monitor the health of FW*/
	WIFI_LOGGER_WATCHDOG_TIMER_SUPPORTED = (1 << (6)),
	/*dumps driver state*/
	WIFI_LOGGER_DRIVER_DUMP_SUPPORTED = (1 << (7)),
	/*tracks connection packets' fate*/
	WIFI_LOGGER_PACKET_FATE_SUPPORTED = (1 << (8)),
};

enum sprdwl_enable_gscan_status {
	SPRDWL_GSCAN_STOP = 0,
	SPRDWL_GSCAN_START = 1,
};

enum sprdwl_wifi_error {
	WIFI_SUCCESS = 0,
	WIFI_ERROR_UNKNOWN = -1,
	WIFI_ERROR_UNINITIALIZED = -2,
	WIFI_ERROR_NOT_SUPPORTED = -3,
	WIFI_ERROR_NOT_AVAILABLE = -4,
	WIFI_ERROR_INVALID_ARGS = -5,
	WIFI_ERROR_INVALID_REQUEST_ID = -6,
	WIFI_ERROR_TIMED_OUT = -7,
	WIFI_ERROR_TOO_MANY_REQUESTS = -8,
	WIFI_ERROR_OUT_OF_MEMORY = -9,
	WIFI_ERROR_BUSY = -10,
};

enum sprdwl_vendor_subcommand_id {
	SPRDWL_VENDOR_GSCAN_START = 20,
	SPRDWL_VENDOR_GSCAN_STOP = 21,
	SPRDWL_VENDOR_GSCAN_GET_CHANNEL = 22,
	SPRDWL_VENDOE_GSCAN_GET_CAPABILITIES = 23,
	SPRDWL_VENDOR_GSCAN_GET_CACHED_RESULTS = 24,
	/* Used when report_threshold is reached in scan cache. */
	SPRDWL_VENDOR_GSCAN_SCAN_RESULTS_AVAILABLE = 25,
	/* Used to report scan results when each probe rsp. is received,
	* if report_events enabled in wifi_scan_cmd_params.
	*/
	SPRDWL_VENDOR_GSCAN_FULL_SCAN_RESULT = 26,
	/* Indicates progress of scanning state-machine. */
	SPRDWL_VENDOR_GSCAN_SCAN_EVENT = 27,
	/* Indicates BSSID Hotlist. */
	SPRDWL_VENDOR_GSCAN_HOTLIST_AP_FOUND = 28,
	SPRDWL_VENDOR_GSCAN_SET_BSSID_HOTLIST = 29,
	SPRDWL_VENDOR_GSCAN_RESET_BSSID_HOTLIST = 30,
	SPRDWL_VENDOR_GSCAN_SIGNIFICANT_CHANGE = 31,
	SPRDWL_VENDOR_GSCAN_SET_SIGNIFICANT_CHANGE = 32,
	SPRDWL_VENDOR_GSCAN_RESET_SIGNIFICANT_CHANGE = 33,
	SPRDWL_VENDOR_SUBCMD_GET_FEATURE = 38,
	SPRDWL_VENDOR_SUBCMD_SET_MAC_OUI = 39,
	SPRDWL_VENDOR_SUBCMD_GSCAN_HOTLIST_AP_LOST = 41,
	SPRDWL_VENDOR_SUBCMD_GET_CONCURRENCY_MATRIX = 42,
	SPRDWL_VENDOR_SUBCMD_GET_WIFI_INFO = 61,
	SPRDWL_VENDOR_SUBCMD_START_LOGGING = 62,
	SPRDWL_VENDOR_SUBCMD_WIFI_LOGGER_MEMORY_DUMP = 63,
	SPRDWL_VENDOR_SUBCMD_GET_LOGGER_FEATURE_SET = 76,
	SPRDWL_VENDOR_SUBCMD_GET_RING_DATA = 77,
	SPRDWL_VENDOR_SUBCMD_OFFLOADED_PACKETS = 79,
	SPRDWL_VENDOR_SUBCMD_ENABLE_ND_OFFLOAD = 82,
	SPRDWL_VENDOR_SUBCMD_GET_WAKE_REASON_STATS = 85,
	SPRDWL_VENDOR_SET_COUNTRY_CODE = 0x100E,
	SPRDWL_VENDOR_SET_LLSTAT = 14,
	SPRDWL_VENDOR_GET_LLSTAT,
	SPRDWL_VENDOR_CLR_LLSTAT,
	SPRDWL_VENDOR_SUBCOMMAND_MAX
};

#define SPRD_NL80211_VENDOR_SUBCMD_GET_AKM_SUITE 0xB0
/* attribute id */

enum sprdwl_vendor_attr_gscan_id {
	SPRDWL_VENDOR_ATTR_FEATURE_SET = 1,
	SPRDWL_VENDOR_ATTR_GSCAN_NUM_CHANNELS = 3,
	SPRDWL_VENDOR_ATTR_GSCAN_CHANNELS = 4,
	SPRDWL_VENDOR_ATTR_MAX
};

enum sprdwl_vendor_attr_get_wifi_info {
	SPRDWL_VENDOR_ATTR_WIFI_INFO_GET_INVALID = 0,
	SPRDWL_VENDOR_ATTR_WIFI_INFO_DRIVER_VERSION = 1,
	SPRDWL_VENDOR_ATTR_WIFI_INFO_FIRMWARE_VERSION = 2,
};

/*link layer stats*/
enum sprdwl_vendor_attribute {
	SPRDWL_VENDOR_ATTR_UNSPEC,
	SPRDWL_VENDOR_ATTR_GET_LLSTAT,
	SPRDWL_VENDOR_ATTR_CLR_LLSTAT,
};

enum sprdwl_wlan_vendor_attr_ll_stats_set {
	SPRDWL_ATTR_LL_STATS_SET_INVALID = 0,
	/* Unsigned 32-bit value */
	SPRDWL_ATTR_LL_STATS_MPDU_THRESHOLD = 1,
	SPRDWL_ATTR_LL_STATS_GATHERING = 2,
	/* keep last */
	SPRDWL_ATTR_LL_STATS_SET_AFTER_LAST,
	SPRDWL_ATTR_LL_STATS_SET_MAX =
	SPRDWL_ATTR_LL_STATS_SET_AFTER_LAST - 1,
};

static const struct nla_policy
sprdwl_ll_stats_policy[SPRDWL_ATTR_LL_STATS_SET_MAX + 1] = {
	[SPRDWL_ATTR_LL_STATS_MPDU_THRESHOLD] = { .type = NLA_U32 },
	[SPRDWL_ATTR_LL_STATS_GATHERING] = { .type = NLA_U32 },
};

enum sprdwl_vendor_attr_ll_stats_results {
	SPRDWL_ATTR_LL_STATS_INVALID = 0,
	SPRDWL_ATTR_LL_STATS_RESULTS_REQ_ID = 1,
	SPRDWL_ATTR_LL_STATS_IFACE_BEACON_RX = 2,
	SPRDWL_ATTR_LL_STATS_IFACE_MGMT_RX = 3,
	SPRDWL_ATTR_LL_STATS_IFACE_MGMT_ACTION_RX = 4,
	SPRDWL_ATTR_LL_STATS_IFACE_MGMT_ACTION_TX = 5,
	SPRDWL_ATTR_LL_STATS_IFACE_RSSI_MGMT = 6,
	SPRDWL_ATTR_LL_STATS_IFACE_RSSI_DATA = 7,
	SPRDWL_ATTR_LL_STATS_IFACE_RSSI_ACK = 8,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_MODE = 9,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_MAC_ADDR = 10,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_STATE = 11,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_ROAMING = 12,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_CAPABILITIES = 13,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_SSID = 14,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_BSSID = 15,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_AP_COUNTRY_STR = 16,
	SPRDWL_ATTR_LL_STATS_IFACE_INFO_COUNTRY_STR = 17,
	SPRDWL_ATTR_LL_STATS_WMM_AC_AC = 18,
	SPRDWL_ATTR_LL_STATS_WMM_AC_TX_MPDU = 19,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RX_MPDU = 20,
	SPRDWL_ATTR_LL_STATS_WMM_AC_TX_MCAST = 21,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RX_MCAST = 22,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RX_AMPDU = 23,
	SPRDWL_ATTR_LL_STATS_WMM_AC_TX_AMPDU = 24,
	SPRDWL_ATTR_LL_STATS_WMM_AC_MPDU_LOST = 25,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RETRIES = 26,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RETRIES_SHORT = 27,
	SPRDWL_ATTR_LL_STATS_WMM_AC_RETRIES_LONG = 28,
	SPRDWL_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MIN = 29,
	SPRDWL_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_MAX = 30,
	SPRDWL_ATTR_LL_STATS_WMM_AC_CONTENTION_TIME_AVG = 31,
	SPRDWL_ATTR_LL_STATS_WMM_AC_CONTENTION_NUM_SAMPLES = 32,
	SPRDWL_ATTR_LL_STATS_IFACE_NUM_PEERS = 33,
	SPRDWL_ATTR_LL_STATS_PEER_INFO_TYPE = 34,
	SPRDWL_ATTR_LL_STATS_PEER_INFO_MAC_ADDRESS = 35,
	SPRDWL_ATTR_LL_STATS_PEER_INFO_CAPABILITIES = 36,
	SPRDWL_ATTR_LL_STATS_PEER_INFO_NUM_RATES = 37,
	SPRDWL_ATTR_LL_STATS_RATE_PREAMBLE = 38,
	SPRDWL_ATTR_LL_STATS_RATE_NSS = 39,
	SPRDWL_ATTR_LL_STATS_RATE_BW = 40,
	SPRDWL_ATTR_LL_STATS_RATE_MCS_INDEX = 41,
	SPRDWL_ATTR_LL_STATS_RATE_BIT_RATE = 42,
	SPRDWL_ATTR_LL_STATS_RATE_TX_MPDU = 43,
	SPRDWL_ATTR_LL_STATS_RATE_RX_MPDU = 44,
	SPRDWL_ATTR_LL_STATS_RATE_MPDU_LOST = 45,
	SPRDWL_ATTR_LL_STATS_RATE_RETRIES = 46,
	SPRDWL_ATTR_LL_STATS_RATE_RETRIES_SHORT = 47,
	SPRDWL_ATTR_LL_STATS_RATE_RETRIES_LONG = 48,
	SPRDWL_ATTR_LL_STATS_RADIO_ID = 49,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME = 50,
	SPRDWL_ATTR_LL_STATS_RADIO_TX_TIME = 51,
	SPRDWL_ATTR_LL_STATS_RADIO_RX_TIME = 52,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_SCAN = 53,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_NBD = 54,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_GSCAN = 55,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_ROAM_SCAN = 56,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_PNO_SCAN = 57,
	SPRDWL_ATTR_LL_STATS_RADIO_ON_TIME_HS20 = 58,
	SPRDWL_ATTR_LL_STATS_RADIO_NUM_CHANNELS = 59,
	SPRDWL_ATTR_LL_STATS_CHANNEL_INFO_WIDTH = 60,
	SPRDWL_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ = 61,
	SPRDWL_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ0 = 62,
	SPRDWL_ATTR_LL_STATS_CHANNEL_INFO_CENTER_FREQ1 = 63,
	SPRDWL_ATTR_LL_STATS_CHANNEL_ON_TIME = 64,
	SPRDWL_ATTR_LL_STATS_CHANNEL_CCA_BUSY_TIME = 65,
	SPRDWL_ATTR_LL_STATS_NUM_RADIOS = 66,
	SPRDWL_ATTR_LL_STATS_CH_INFO = 67,
	SPRDWL_ATTR_LL_STATS_PEER_INFO = 68,
	SPRDWL_ATTR_LL_STATS_PEER_INFO_RATE_INFO = 69,
	SPRDWL_ATTR_LL_STATS_WMM_INFO = 70,
	SPRDWL_ATTR_LL_STATS_RESULTS_MORE_DATA = 71,
	SPRDWL_ATTR_LL_STATS_IFACE_AVERAGE_TSF_OFFSET = 72,
	SPRDWL_ATTR_LL_STATS_IFACE_LEAKY_AP_DETECTED = 73,
	SPRDWL_ATTR_LL_STATS_IFACE_LEAKY_AP_AVG_NUM_FRAMES_LEAKED = 74,
	SPRDWL_ATTR_LL_STATS_IFACE_LEAKY_AP_GUARD_TIME = 75,
	SPRDWL_ATTR_LL_STATS_TYPE = 76,
	SPRDWL_ATTR_LL_STATS_RADIO_NUM_TX_LEVELS = 77,
	SPRDWL_ATTR_LL_STATS_RADIO_TX_TIME_PER_LEVEL = 78,
	SPRDWL_ATTR_LL_STATS_IFACE_RTS_SUCC_CNT = 79,
	SPRDWL_ATTR_LL_STATS_IFACE_RTS_FAIL_CNT = 80,
	SPRDWL_ATTR_LL_STATS_IFACE_PPDU_SUCC_CNT = 81,
	SPRDWL_ATTR_LL_STATS_IFACE_PPDU_FAIL_CNT = 82,

	/* keep last */
	SPRDWL_ATTR_LL_STATS_AFTER_LAST,
	SPRDWL_ATTR_LL_STATS_MAX =
	SPRDWL_ATTR_LL_STATS_AFTER_LAST - 1,
};

enum sprdwl_wlan_vendor_attr_ll_stats_type
{
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_INVALID = 0,
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_RADIO = 1,
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_IFACE = 2,
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_PEERS = 3,

	/* keep last */
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_AFTER_LAST,
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_MAX =
	SPRDWL_NL80211_VENDOR_SUBCMD_LL_STATS_TYPE_AFTER_LAST - 1,
};

enum sprdwl_attr_ll_stats_clr {
	SPRDWL_ATTR_LL_STATS_CLR_INVALID = 0,
	SPRDWL_ATTR_LL_STATS_CLR_CONFIG_REQ_MASK = 1,
	SPRDWL_ATTR_LL_STATS_CLR_CONFIG_STOP_REQ = 2,
	SPRDWL_ATTR_LL_STATS_CLR_CONFIG_RSP_MASK = 3,
	SPRDWL_ATTR_LL_STATS_CLR_CONFIG_STOP_RSP = 4,
	/* keep last */
	SPRDWL_ATTR_LL_STATS_CLR_AFTER_LAST,
	SPRDWL_ATTR_LL_STATS_CLR_MAX =
	SPRDWL_ATTR_LL_STATS_CLR_AFTER_LAST - 1,
};

/*end of link layer stats*/
enum sprdwl_vendor_attr_gscan_config_params {
	SPRDWL_ATTR_GSCAN_SUBCMD_CONFIG_PARAM_INVALID = 0,
	SPRDWL_ATTR_GSCAN_REQUEST_ID = 1,
	SPRDWL_VENDOR_ATTR_WIFI_BAND = 2,
	/* Unsigned 32-bit value */
	SPRDWL_VENDOR_ATTR_MAX_CHANNELS = 3,

	/* Attributes for input params used by
	 * SPRDWL_NL80211_VENDOR_SUBCMD_GSCAN_START sub command.
	 */

	/* Unsigned 32-bit value; channel frequency */
	SPRDWL_ATTR_GSCAN_CHANNEL = 4,
	/* Unsigned 32-bit value; dwell time in ms. */
	SPRDWL_ATTR_GSCAN_CHANNEL_DWELL_TIME = 5,
	/* Unsigned 8-bit value; 0: active; 1: passive; N/A for DFS */
	SPRDWL_ATTR_GSCAN_CHANNEL_PASSIVE = 6,
	/* Unsigned 8-bit value; channel class */
	SPRDWL_ATTR_GSCAN_CHANNEL_CLASS = 7,

	/* Unsigned 8-bit value; bucket index, 0 based */
	SPRDWL_ATTR_GSCAN_BUCKET_INDEX = 8,
	/* Unsigned 8-bit value; band. */
	SPRDWL_ATTR_GSCAN_BUCKET_BAND = 9,
	/* Unsigned 32-bit value; desired period, in ms. */
	SPRDWL_ATTR_GSCAN_BUCKET_PERIOD = 10,
	/* Unsigned 8-bit value; report events semantics. */
	SPRDWL_ATTR_GSCAN_BUCKET_REPORT_EVENTS = 11,
	/* Unsigned 32-bit value. Followed by a nested array of
	 * GSCAN_CHANNEL_* attributes.
	 */
	SPRDWL_ATTR_GSCAN_BUCKET_NUM_CHANNEL_SPECS = 12,

	/* Array of SPRDWL_ATTR_GSCAN_CHANNEL_* attributes.
	 * Array size: SPRDWL_ATTR_GSCAN_BUCKET_NUM_CHANNEL_SPECS
	 */
	SPRDWL_ATTR_GSCAN_CHANNEL_SPEC = 13,
	SPRDWL_ATTR_GSCAN_BASE_PERIOD = 14,
	SPRDWL_ATTR_GSCAN_MAX_AP_PER_SCAN = 15,
	SPRDWL_ATTR_GSCAN_THRESHOLD_PERCENT = 16,
	SPRDWL_ATTR_GSCAN_NUM_BUCKETS = 17,

	/* Array of SPRDWL_ATTR_GSCAN_BUCKET_* attributes.
	 * Array size: SPRDWL_ATTR_GSCAN_SCAN_CMD_PARAMS_NUM_BUCKETS
	 */
	SPRDWL_ATTR_GSCAN_BUCKET_SPEC = 18,

	/* Unsigned 8-bit value */
	SPRDWL_ATTR_GSCAN_PARAM_FLUSH = 19,
	/* Unsigned 32-bit value; maximum number of results to be returned. */
	SPRDWL_ATTR_GSCAN_CACHED_PARAM_MAX = 20,

	/* An array of 6 x unsigned 8-bit value */
	SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM_BSSID = 21,
	/* Signed 32-bit value */
	SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM_RSSI_LOW = 22,
	/* Signed 32-bit value */
	SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM_RSSI_HIGH = 23,
	/* Unsigned 32-bit value */
	SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM_CHANNEL = 24,

	/* Number of hotlist APs as unsigned 32-bit value, followed by a nested
	 * array of AP_THRESHOLD_PARAM attributes and values. The size of the
	 * array is determined by NUM_AP.
	 */
	SPRDWL_ATTR_GSCAN_BSSID_HOTLIST_PARAMS_NUM_AP = 25,

	/* Array of SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM_* attributes.
	 * Array size: SPRDWL_ATTR_GSCAN_BUCKET_NUM_CHANNEL_SPECS
	 */
	SPRDWL_ATTR_GSCAN_AP_THRESHOLD_PARAM = 26,

	/* Unsigned 32-bit value; number of samples for averaging RSSI. */
	SPRDWL_ATTR_GSCAN_SIGNIFICANT_CHANGE_PARAMS_RSSI_SAMPLE_SIZE
	= 27,
	/* Unsigned 32-bit value; number of samples to confirm AP loss. */
	SPRDWL_ATTR_GSCAN_SIGNIFICANT_CHANGE_PARAMS_LOST_AP_SAMPLE_SIZE
	= 28,
	/* Unsigned 32-bit value; number of APs breaching threshold. */
	SPRDWL_ATTR_GSCAN_SIGNIFICANT_CHANGE_PARAMS_MIN_BREACHING = 29,
	/* Unsigned 32-bit value; number of APs. Followed by an array of
	 * AP_THRESHOLD_PARAM attributes. Size of the array is NUM_AP.
	 */
	SPRDWL_ATTR_GSCAN_SIGNIFICANT_CHANGE_PARAMS_NUM_AP = 30,
	/* Unsigned 32-bit value; number of samples to confirm AP loss. */
	SPRDWL_ATTR_GSCAN_BSSID_HOTLIST_PARAMS_LOST_AP_SAMPLE_SIZE
	= 31,
	/* Unsigned 32-bit value. If max_period is non zero or different than
	 * period, then this bucket is an exponential backoff bucket.
	 */
	SPRDWL_ATTR_GSCAN_BUCKET_MAX_PERIOD = 32,
	/* Unsigned 32-bit value. */
	SPRDWL_ATTR_GSCAN_BUCKET_BASE = 33,
	/* Unsigned 32-bit value. For exponential back off bucket, number of
	 * scans to perform for a given period.
	 */
	SPRDWL_ATTR_GSCAN_BUCKET_STEP_COUNT = 34,
	SPRDWL_ATTR_GSCAN_REPORT_NUM_SCANS = 35,

	/* Attributes for data used by
	 * SPRDWL_NL80211_VENDOR_SUBCMD_GSCAN_SET_SSID_HOTLIST sub command.
	 */
	/* Unsigned 3-2bit value; number of samples to confirm SSID loss. */
	SPRDWL_ATTR_GSCAN_SSID_HOTLIST_PARAMS_LOST_SSID_SAMPLE_SIZE
	= 36,
	/* Number of hotlist SSIDs as unsigned 32-bit value, followed by a
	 * nested array of SSID_THRESHOLD_PARAM_* attributes and values. The
	 * size of the array is determined by NUM_SSID.
	 */
	SPRDWL_ATTR_GSCAN_SSID_HOTLIST_PARAMS_NUM_SSID = 37,
	/* Array of SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM_*
	 * attributes.
	 * Array size: SPRDWL_ATTR_GSCAN_SSID_HOTLIST_PARAMS_NUM_SSID
	 */
	SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM = 38,

	/* An array of 33 x unsigned 8-bit value; NULL terminated SSID */
	SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM_SSID = 39,
	/* Unsigned 8-bit value */
	SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM_BAND = 40,
	/* Signed 32-bit value */
	SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM_RSSI_LOW = 41,
	/* Signed 32-bit value */
	SPRDWL_ATTR_GSCAN_SSID_THRESHOLD_PARAM_RSSI_HIGH = 42,
	/* Unsigned 32-bit value; a bitmask with additional gscan config flag.
	 */
	SPRDWL_ATTR_GSCAN_CONFIGURATION_FLAGS = 43,

	/* keep last */
	SPRDWL_ATTR_GSCAN_SUBCMD_CONFIG_PARAM_AFTER_LAST,
	SPRDWL_ATTR_GSCAN_MAX,
};

enum SPRDWL_GSCAN_results {
	GSCAN_RESULTS_INVALID = 0,
	GSCAN_RESULTS_REQUEST_ID = 1,
	GSCAN_STATUS = 2,
	GSCAN_RESULTS_NUM_CHANNELS = 3,
	GSCAN_RESULTS_CHANNELS = 4,
	GSCAN_SCAN_CACHE_SIZE = 5,
	GSCAN_MAX_SCAN_BUCKETS = 6,
	GSCAN_MAX_AP_CACHE_PER_SCAN = 7,
	GSCAN_MAX_RSSI_SAMPLE_SIZE = 8,
	/* Signed 32-bit value */
	GSCAN_MAX_SCAN_REPORTING_THRESHOLD = 9,
	/* Unsigned 32-bit value */
	GSCAN_MAX_HOTLIST_BSSIDS = 10,
	/* Unsigned 32-bit value */
	GSCAN_MAX_SIGNIFICANT_WIFI_CHANGE_APS = 11,
	/* Unsigned 32-bit value */
	GSCAN_MAX_BSSID_HISTORY_ENTRIES = 12,
	/* Unsigned 32-bit value */
	GSCAN_RESULTS_NUM_RESULTS_AVAILABLE = 13,
	GSCAN_RESULTS_LIST = 14,
	/* Unsigned 64-bit value; age of sample at the time of retrieval */
	GSCAN_RESULTS_SCAN_RESULT_TIME_STAMP = 15,
	/* 33 x unsigned 8-bit value; NULL terminated SSID */
	GSCAN_RESULTS_SCAN_RESULT_SSID = 16,
	/* An array of 6 x unsigned 8-bit value */
	GSCAN_RESULTS_SCAN_RESULT_BSSID = 17,
	/* Unsigned 32-bit value; channel frequency in MHz */
	GSCAN_RESULTS_SCAN_RESULT_CHANNEL = 18,
	/* Signed 32-bit value */
	GSCAN_RESULTS_SCAN_RESULT_RSSI = 19,
	/* Unsigned 32-bit value */
	GSCAN_RESULTS_SCAN_RESULT_RTT = 20,
	/* Unsigned 32-bit value */
	GSCAN_RESULTS_SCAN_RESULT_RTT_SD = 21,
	/* Unsigned 16-bit value */
	GSCAN_RESULTS_SCAN_RESULT_BEACON_PERIOD = 22,
	/* Unsigned 16-bit value */
	GSCAN_RESULTS_SCAN_RESULT_CAPABILITY = 23,
	/* Unsigned 32-bit value; size of the IE DATA blob */
	GSCAN_RESULTS_SCAN_RESULT_IE_LENGTH = 24,
	/* An array of IE_LENGTH x unsigned 8-bit value; blob of all the
	 * information elements found in the beacon; this data should be a
	 * packed list of wifi_information_element objects, one after the
	 * other.
	 */
	GSCAN_RESULTS_SCAN_RESULT_IE_DATA = 25,

	/* Unsigned 8-bit value; set by driver to indicate more scan results are
	 * available.
	 */
	GSCAN_RESULTS_SCAN_RESULT_MORE_DATA = 26,

	/* GSCAN attributes for
	 * QCA_NL80211_VENDOR_SUBCMD_GSCAN_SCAN_EVENT sub-command.
	 */
	/* Unsigned 8-bit value */
	GSCAN_RESULTS_SCAN_EVENT_TYPE = 27,
	/* Unsigned 32-bit value */
	GSCAN_RESULTS_SCAN_EVENT_STATUS = 28,

	/* GSCAN attributes for
	 * QCA_NL80211_VENDOR_SUBCMD_GSCAN_HOTLIST_AP_FOUND sub-command.
	 */
	/* Use attr GSCAN_RESULTS_NUM_RESULTS_AVAILABLE
	 * to indicate number of results.
	 * Also, use GSCAN_RESULTS_LIST to indicate the
	 * list of results.
	 */

	/* GSCAN attributes for
	 * QCA_NL80211_VENDOR_SUBCMD_GSCAN_SIGNIFICANT_CHANGE sub-command.
	 */
	/* An array of 6 x unsigned 8-bit value */
	GSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_BSSID = 29,
	/* Unsigned 32-bit value */
	GSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_CHANNEL
	= 30,
	/* Unsigned 32-bit value. */
	GSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_NUM_RSSI
	= 31,
	/* A nested array of signed 32-bit RSSI values. Size of the array is
	 * determined by (NUM_RSSI of SIGNIFICANT_CHANGE_RESULT_NUM_RSSI.
	 */
	GSCAN_RESULTS_SIGNIFICANT_CHANGE_RESULT_RSSI_LIST
	= 32,

	/* GSCAN attributes used with
	 * QCA_NL80211_VENDOR_SUBCMD_GSCAN_GET_CACHED_RESULTS sub-command.
	 */
	/* Use attr GSCAN_RESULTS_NUM_RESULTS_AVAILABLE
	 * to indicate number of gscan cached results returned.
	 * Also, use GSCAN_CACHED_RESULTS_LIST to indicate
	 *  the list of gscan cached results.
	 */

	/* An array of NUM_RESULTS_AVAILABLE x
	 * QCA_NL80211_VENDOR_ATTR_GSCAN_CACHED_RESULTS_*
	 */
	GSCAN_CACHED_RESULTS_LIST = 33,
	/* Unsigned 32-bit value; a unique identifier for the scan unit. */
	GSCAN_CACHED_RESULTS_SCAN_ID = 34,
	/* Unsigned 32-bit value; a bitmask w/additional information about scan.
	 */
	GSCAN_CACHED_RESULTS_FLAGS = 35,
	/* Use attr GSCAN_RESULTS_NUM_RESULTS_AVAILABLE
	 * to indicate number of wifi scan results/bssids retrieved by the scan.
	 * Also, use GSCAN_RESULTS_LIST to indicate the
	 * list of wifi scan results returned for each cached result block.
	 */

	/* GSCAN attributes for
	 * QCA_NL80211_VENDOR_SUBCMD_PNO_NETWORK_FOUND sub-command.
	 */
	/* Use GSCAN_RESULTS_NUM_RESULTS_AVAILABLE for
	 * number of results.
	 * Use GSCAN_RESULTS_LIST to indicate the nested
	 * list of wifi scan results returned for each
	 * wifi_passpoint_match_result block.
	 * Array size: GSCAN_RESULTS_NUM_RESULTS_AVAILABLE.
	 */

	/* GSCAN attributes for
	 * QCA_NL80211_VENDOR_SUBCMD_PNO_PASSPOINT_NETWORK_FOUND sub-command.
	 */
	/* Unsigned 32-bit value */
	GSCAN_PNO_RESULTS_PASSPOINT_NETWORK_FOUND_NUM_MATCHES
	= 36,
	/* A nested array of
	 * GSCAN_PNO_RESULTS_PASSPOINT_MATCH_*
	 * attributes. Array size =
	 * *_ATTR_GSCAN_PNO_RESULTS_PASSPOINT_NETWORK_FOUND_NUM_MATCHES.
	 */
	GSCAN_PNO_RESULTS_PASSPOINT_MATCH_RESULT_LIST = 37,

	/* Unsigned 32-bit value; network block id for the matched network */
	GSCAN_PNO_RESULTS_PASSPOINT_MATCH_ID = 38,
	/* Use GSCAN_RESULTS_LIST to indicate the nested
	 * list of wifi scan results returned for each
	 * wifi_passpoint_match_result block.
	 */
	/* Unsigned 32-bit value */
	GSCAN_PNO_RESULTS_PASSPOINT_MATCH_ANQP_LEN = 39,
	/* An array size of PASSPOINT_MATCH_ANQP_LEN of unsigned 8-bit values;
	 * ANQP data in the information_element format.
	 */
	GSCAN_PNO_RESULTS_PASSPOINT_MATCH_ANQP = 40,

	/* Unsigned 32-bit value; a GSCAN Capabilities attribute. */
	GSCAN_RESULTS_CAPABILITIES_MAX_HOTLIST_SSIDS = 41,
	/* Unsigned 32-bit value; a GSCAN Capabilities attribute. */
	GSCAN_RESULTS_CAPABILITIES_MAX_NUM_EPNO_NETS = 42,
	/* Unsigned 32-bit value; a GSCAN Capabilities attribute. */
	GSCAN_MAX_NUM_EPNO_NETS_BY_SSID = 43,
	/* Unsigned 32-bit value; a GSCAN Capabilities attribute. */
	GSCAN_MAX_NUM_WHITELISTED_SSID = 44,

	GSCAN_RESULTS_BUCKETS_SCANNED = 45,
	/* Unsigned 32bit value; a GSCAN Capabilities attribute. */
	SPRDWL_VENDOR_ATTR_CAPABILITIES_MAX_NUM_BLACKLISTED_BSSID = 46,

	/* keep last */
	GSCAN_RESULTS_AFTER_LAST,
	GSCAN_RESULTS_MAX,
};

enum offloaded_packets_sending_control {
	OFFLOADED_PACKETS_SENDING_CONTROL_INVALID = 0,
	OFFLOADED_PACKETS_SENDING_START,
	OFFLOADED_PACKETS_SENDING_STOP
};

enum sprdwl_attr_offloaded_packets {
	OFFLOADED_PACKETS_INVALID = 0,
	OFFLOADED_PACKETS_SENDING_CONTROL,
	OFFLOADED_PACKETS_REQUEST_ID,
	OFFLOADED_PACKETS_IP_PACKET_DATA,
	OFFLOADED_PACKETS_SRC_MAC_ADDR,
	OFFLOADED_PACKETS_DST_MAC_ADDR,
	OFFLOADED_PACKETS_PERIOD,
	OFFLOADED_PACKETS_ETHER_PROTO_TYPE,
	OFFLOADED_PACKETS_AFTER_LAST,
	OFFLOADED_PACKETS_MAX =
	OFFLOADED_PACKETS_AFTER_LAST - 1,
};

/* NL attributes for data used by
 * NL80211_VENDOR_SUBCMD_GET_WAKE_REASON_STATS.
 */
enum sprd_wlan_vendor_attr_wake_stats {
	SPRDWL_WAKE_INVALID = 0,
	SPRDWL_WAKE_TOTAL_CMD_EVENT_WAKE,
	SPRDWL_WAKE_CMD_EVENT_WAKE_CNT_PTR,
	SPRDWL_WAKE_CMD_EVENT_WAKE_CNT_SZ,
	SPRDWL_WAKE_TOTAL_DRIVER_FW_LOCAL_WAKE,
	SPRDWL_WAKE_DRIVER_FW_LOCAL_WAKE_CNT_PTR,
	SPRDWL_WAKE_DRIVER_FW_LOCAL_WAKE_CNT_SZ,
	SPRDWL_WAKE_TOTAL_RX_DATA_WAKE,
	SPRDWL_WAKE_RX_UNICAST_CNT,
	SPRDWL_WAKE_RX_MULTICAST_CNT,
	SPRDWL_WAKE_RX_BROADCAST_CNT,
	SPRDWL_WAKE_ICMP_PKT,
	SPRDWL_WAKE_ICMP6_PKT,
	SPRDWL_WAKE_ICMP6_RA,
	SPRDWL_WAKE_ICMP6_NA,
	SPRDWL_WAKE_ICMP6_NS,
	SPRDWL_WAKE_ICMP4_RX_MULTICAST_CNT,
	SPRDWL_WAKE_ICMP6_RX_MULTICAST_CNT,
	SPRDWL_WAKE_OTHER_RX_MULTICAST_CNT,
	/* keep last */
	SPRDWL_WAKE_AFTER_LAST,
	SPRDWL_WAKE_MAX = SPRDWL_WAKE_AFTER_LAST - 1,
};

enum sprdwl_attr_wifi_logger_start {
	SPRDWL_LOGGER_START_INVALID = 0,
	SPRDWL_LOGGER_RING_ID = 1,
	SPRDWL_LOGGER_VERBOSE_LEVEL = 2,
	SPRDWL_LOGGER_FLAGS = 3,

	/* keep last */
	SPRDWL_LOGGER_START_AFTER_LAST,
	SPRDWL_LOGGER_START_GET_MAX =
	SPRDWL_LOGGER_START_AFTER_LAST - 1,
};
enum nl80211_vendor_subcmds_index {
	SPRDWL_GSCAN_START_INDEX,
	SPRDWL_GSCAN_STOP_INDEX,
	SPRDWL_GSCAN_GET_CAPABILITIES_INDEX,
	SPRDWL_GSCAN_GET_CACHED_RESULTS_INDEX,
	SPRDWL_GSCAN_SCAN_RESULTS_AVAILABLE_INDEX,
	SPRDWL_GSCAN_FULL_SCAN_RESULT_INDEX,
	SPRDWL_GSCAN_SCAN_EVENT_INDEX,
	SPRDWL_GSCAN_HOTLIST_AP_FOUND_INDEX,
	SPRDWL_GSCAN_SET_BSSID_HOTLIST_INDEX,
	SPRDWL_GSCAN_RESET_BSSID_HOTLIST_INDEX,
	SPRDWL_GSCAN_SIGNIFICANT_CHANGE_INDEX,
	SPRDWL_GSCAN_SET_SIGNIFICANT_CHANGE_INDEX,
	SPRDWL_GSCAN_RESET_SIGNIFICANT_CHANGE_INDEX,
	SPRDWL_ACS_LTE_EVENT_INDEX,
};

enum wifi_connection_state {
	WIFI_DISCONNECTED = 0,
	WIFI_AUTHENTICATING = 1,
	WIFI_ASSOCIATING = 2,
	WIFI_ASSOCIATED = 3,
	WIFI_EAPOL_STARTED = 4,
	WIFI_EAPOL_COMPLETED = 5,
};

enum wifi_roam_state {
	WIFI_ROAMING_IDLE = 0,
	WIFI_ROAMING_ACTIVE = 1,
};

/* access categories */
enum wifi_traffic_ac {
	WIFI_AC_VO = 0,
	WIFI_AC_VI = 1,
	WIFI_AC_BE = 2,
	WIFI_AC_BK = 3,
	WIFI_AC_MAX = 4,
};

/* FTM/indoor location subcommands */
#define SPRD_NL80211_VENDOR_SUBCMD_RTT_GET_CAPA  0x1102

/* configuration params */
struct wifi_link_layer_params {
	u32 mpdu_size_threshold;
	u32 aggressive_statistics_gathering;
} __packed;

struct wifi_clr_llstat_rsp {
	u32 stats_clear_rsp_mask;
	u8 stop_rsp;
};

/* wifi rate */
struct wifi_rate {
	u32 preamble:3;
	u32 nss:2;
	u32 bw:3;
	u32 ratemcsidx:8;
	u32 reserved:16;
	u32 bitrate;
};

struct wifi_rate_stat {
	struct wifi_rate rate;
	u32 tx_mpdu;
	u32 rx_mpdu;
	u32 mpdu_lost;
	u32 retries;
	u32 retries_short;
	u32 retries_long;
};

/* per peer statistics */
struct wifi_peer_info {
	u8 type;
	u8 peer_mac_address[6];
	u32 capabilities;
	u32 num_rate;
	struct wifi_rate_stat rate_stats[];
};

struct wifi_interface_link_layer_info {
	enum sprdwl_mode mode;
	u8 mac_addr[6];
	enum wifi_connection_state state;
	enum wifi_roam_state roaming;
	u32 capabilities;
	u8 ssid[33];
	u8 bssid[6];
	u8 ap_country_str[3];
	u8 country_str[3];
};

/* Per access category statistics */
struct wifi_wmm_ac_stat {
	enum wifi_traffic_ac ac;
	u32 tx_mpdu;
	u32 rx_mpdu;
	u32 tx_mcast;
	u32 rx_mcast;
	u32 rx_ampdu;
	u32 tx_ampdu;
	u32 mpdu_lost;
	u32 retries;
	u32 retries_short;
	u32 retries_long;
	u32 contention_time_min;
	u32 contention_time_max;
	u32 contention_time_avg;
	u32 contention_num_samples;
};

/* interface statistics */
struct wifi_iface_stat {
	void *iface;
	struct wifi_interface_link_layer_info info;
	u32 beacon_rx;
	u64 average_tsf_offset;
	u32 leaky_ap_detected;
	u32 leaky_ap_avg_num_frames_leaked;
	u32 leaky_ap_guard_time;
	u32 mgmt_rx;
	u32 mgmt_action_rx;
	u32 mgmt_action_tx;
	u32 rssi_mgmt;
	u32 rssi_data;
	u32 rssi_ack;
	struct wifi_wmm_ac_stat ac[WIFI_AC_MAX];
	u32 num_peers;
	struct wifi_peer_info peer_info[];
};

/* WiFi Common definitions */
/* channel operating width */
enum wifi_channel_width {
	WIFI_CHAN_WIDTH_20 = 0,
	WIFI_CHAN_WIDTH_40 = 1,
	WIFI_CHAN_WIDTH_80 = 2,
	WIFI_CHAN_WIDTH_160 = 3,
	WIFI_CHAN_WIDTH_80P80 = 4,
	WIFI_CHAN_WIDTH_5 = 5,
	WIFI_CHAN_WIDTH_10 = 6,
	WIFI_CHAN_WIDTH_INVALID = -1
};

/* channel information */
struct wifi_channel_info {
	enum wifi_channel_width width;
	u32 center_freq;
	u32 center_freq0;
	u32 center_freq1;
};

/* channel statistics */
struct wifi_channel_stat {
	struct wifi_channel_info channel;
	u32 on_time;
	u32 cca_busy_time;
};

/* radio statistics */
#define SPRDWL_MAX_CHANNEL_NUM	1
struct wifi_radio_stat {
	u32 radio;
	u32 on_time;
	u32 tx_time;
	u32 num_tx_levels;
	u32 *tx_time_per_levels;
	u32 rx_time;
	u32 on_time_scan;
	u32 on_time_nbd;
	u32 on_time_gscan;
	u32 on_time_roam_scan;
	u32 on_time_pno_scan;
	u32 on_time_hs20;
	u32 num_channels;
	struct wifi_channel_stat channels[SPRDWL_MAX_CHANNEL_NUM];
};

struct sprdwl_llstat_channel_info
{	
	u32 channel_width;
	u32 center_freq;
	u32 center_freq0;
	u32 center_freq1;
	u32 on_time;
	u32 cca_busy_time;
} __packed;

struct sprdwl_wmm_ac_stat {
	u8 ac_num;
	u32 tx_mpdu;
	u32 rx_mpdu;
	u32 mpdu_lost;
	u32 retries;
} __packed;

struct sprdwl_llstat_data {
	u32 beacon_rx;
	u8 rssi_mgmt;
	struct sprdwl_wmm_ac_stat ac[WIFI_AC_MAX];
	u32 on_time;
	u32 tx_time;
	u32 rx_time;
	u32 on_time_scan;
} __packed;

struct sprdwl_vendor_data {
	struct wifi_radio_stat radio_st;
	struct wifi_iface_stat iface_st;
};

/*end of link layer stats*/

int sprdwl_vendor_init(struct wiphy *wiphy);
int sprdwl_vendor_deinit(struct wiphy *wiphy);

#define MAX_CHANNELS 16
#define MAX_BUCKETS 2
#define MAX_HOTLIST_APS 16
#define MAX_SIGNIFICANT_CHANGE_APS 16
#define MAX_AP_CACHE_PER_SCAN 32
#define MAX_CHANNLES_NUM 14

enum sprdwl_gscan_wifi_band {
	WIFI_BAND_UNSPECIFIED,
	WIFI_BAND_BG = 1,
	WIFI_BAND_A = 2,
	WIFI_BAND_A_DFS = 4,
	WIFI_BAND_A_WITH_DFS = 6,
	WIFI_BAND_ABG = 3,
	WIFI_BAND_ABG_WITH_DFS = 7,
};

struct sprdwl_gscan_channel_spec {
	int channel;
	int dwelltimems;
	int passive;
};

#define REPORT_EVENTS_BUFFER_FULL      0
#define REPORT_EVENTS_EACH_SCAN        1
#define REPORT_EVENTS_FULL_RESULTS     2
#define REPORT_EVENTS_NO_BATCH         4

enum sprdwl_gscan_event {
	WIFI_SCAN_BUFFER_FULL,
	WIFI_SCAN_COMPLETE,
};

struct sprdwl_gscan_bucket_spec {
	int bucket;
	enum sprdwl_gscan_wifi_band band;
	int period;
	u8 report_events;
	int max_period;
	int exponent;
	int step_count;
	int num_channels;
	struct sprdwl_gscan_channel_spec channels[MAX_CHANNELS];
};

struct sprdwl_gscan_cmd_params {
	int base_period;
	int max_ap_per_scan;
	int report_threshold_percent;
	int report_threshold_num_scans;
	int num_buckets;
	struct sprdwl_gscan_bucket_spec buckets[MAX_BUCKETS];
};

struct sprdwl_cmd_gscan_set_config {
	int base_period;
	int num_buckets;
	struct sprdwl_gscan_bucket_spec buckets[MAX_BUCKETS];
};

struct sprdwl_cmd_gscan_set_scan_config {
	int max_ap_per_scan;
	int report_threshold_percent;
	int report_threshold_num_scans;
};

struct sprdwl_cmd_gscan_rsp_header {
	u8 subcmd;
	u8 status;
	u16 data_len;
} __packed;

struct sprdwl_cmd_gscan_channel_list {
	int num_channels;
	int channels[MAX_CHANNLES_NUM];
};

struct sprdwl_gscan_capabilities {
	int max_scan_cache_size;
	int max_scan_buckets;
	int max_ap_cache_per_scan;
	int max_rssi_sample_size;
	int max_scan_reporting_threshold;
	int max_hotlist_bssids;
	int max_hotlist_ssids;
	int max_significant_wifi_change_aps;
	int max_bssid_history_entries;
	int max_number_epno_networks;
	int max_number_epno_networks_by_ssid;
	int max_number_of_white_listed_ssid;
};

struct sprdwl_gscan_result {
	u64 ts;
	char ssid[32 + 1];
	char bssid[ETH_ALEN];
	int channel;
	int rssi;
	u64 rtt;
	u64 rtt_sd;
	unsigned short beacon_period;
	unsigned short capability;
	unsigned int ie_length;
	char ie_data[1];
};

struct sprdwl_gscan_cached_results {
	int scan_id;
	u8 flags;
	int num_results;
	struct sprdwl_gscan_result results[MAX_AP_CACHE_PER_SCAN];
};

#define SPRD_NL80211_VENDOR_SUBCMD_SET_SAE_PASSWORD 0x2B

#define SPRDWL_VENDOR_SAE_ENTRY       0x0
#define SPRDWL_VENDOR_SAE_PASSWORD    0x1
#define SPRDWL_VENDOR_SAE_IDENTIFIER  0x2
#define SPRDWL_VENDOR_SAE_PEER_ADDR   0x3
#define SPRDWL_VENDOR_SAE_VLAN_ID     0x4
#define SPRDWL_VENDOR_SAE_GROUP_ID    0x5
#define SPRDWL_VENDOR_SAE_ACT         0x6
#define SPRDWL_VENDOR_SAE_PWD         0x7
#define SPRDWL_VENDOR_SAE_END         0xFF

void sprdwl_report_gscan_result(struct sprdwl_vif *vif,
				u32 report_event, u8 bucketid,
				u16 chan, s16 rssi, const u8 *buf, u16 len);
int sprdwl_gscan_done(struct sprdwl_vif *vif, u8 bucketid);
int sprdwl_buffer_full_event(struct sprdwl_vif *vif);
int sprdwl_available_event(struct sprdwl_vif *vif);
int sprdwl_report_acs_lte_event(struct sprdwl_vif *vif);
int sprdwl_set_packet_offload(struct sprdwl_priv *priv, u8 vif_mode,
			      u32 req, u8 enable, u32 interval,
			      u32 len, u8 *data);
int sprdwl_softap_set_sae_para(struct sprdwl_vif *vif,
			       struct sprdwl_softap_sae_setting *setting);

#endif
