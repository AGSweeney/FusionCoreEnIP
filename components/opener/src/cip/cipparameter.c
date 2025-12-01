/*******************************************************************************
 * Copyright (c) 2025, Adam G. Sweeney <agsweeney@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ******************************************************************************/
/**
 * @file cipparameter.c
 *
 * CIP Parameter Object (Class 0x0F)
 * ==================================
 *
 * This module implements the CIP Parameter Object as specified in
 * Volume 1, Section 5-7 of the CIP specification.
 *
 * Implemented Attributes
 * ----------------------
 * - Attribute 1: Parameter Name (SHORT_STRING)
 * - Attribute 2: Parameter Value (varies by data type)
 * - Attribute 3: Parameter Units (SHORT_STRING)
 * - Attribute 4: Help String (SHORT_STRING)
 * - Attribute 5: Minimum Value (varies by data type)
 * - Attribute 6: Maximum Value (varies by data type)
 * - Attribute 7: Default Value (varies by data type)
 * - Attribute 8: Data Type Code (USINT)
 *
 * Implemented Services
 * --------------------
 * - GetAttributeSingle
 * - GetAttributeAll
 * - SetAttributeSingle (for writable parameters)
 */

#include "cipparameter.h"

#include <string.h>
#include <stddef.h>
#include <stdlib.h>

#include "opener_user_conf.h"
#include "cipcommon.h"
#include "cipmessagerouter.h"
#include "ciperror.h"
#include "cipstring.h"
#include "endianconv.h"
#include "opener_api.h"
#include "trace.h"
#include "ciptcpipinterface.h"
#include "nvtcpip.h"
#include "ciptypes.h"

/* Include system configuration for accessing network and device settings */
#include "system_config.h"

/* Parameter Object Class Revision */
#define PARAMETER_CLASS_REVISION 1

/* Maximum number of parameter instances */
#define MAX_PARAMETER_INSTANCES 100

/* Parameter Instance IDs */
/* Network Configuration Parameters (1-10) */
#define PARAM_ID_IP_ADDRESS           1
#define PARAM_ID_SUBNET_MASK          2
#define PARAM_ID_GATEWAY              3
#define PARAM_ID_DNS_SERVER_1          4
#define PARAM_ID_DNS_SERVER_2          5
#define PARAM_ID_DHCP_ENABLED          6
#define PARAM_ID_HOSTNAME              7
#define PARAM_ID_DOMAIN_NAME            8
#define PARAM_ID_MCAST_TTL             9
#define PARAM_ID_ACD_ENABLED          20
#define PARAM_ID_ACD_TIMEOUT          21

/* NAU7802 Scale Parameters (10-20) */
#define PARAM_ID_NAU7802_ENABLED      10
#define PARAM_ID_NAU7802_UNIT         11
#define PARAM_ID_NAU7802_GAIN         12
#define PARAM_ID_NAU7802_SAMPLE_RATE  13
#define PARAM_ID_NAU7802_CHANNEL      14
#define PARAM_ID_NAU7802_LDO          15
#define PARAM_ID_NAU7802_AVERAGE      16
#define PARAM_ID_NAU7802_CAL_FACTOR   17
#define PARAM_ID_NAU7802_ZERO_OFFSET  18
/* PARAM_ID_NAU7802_BYTE_OFFSET removed - byte offsets are statically mapped, not configurable */

/* VL53L1X Time-of-Flight Sensor Parameters (22-37) */
#define PARAM_ID_VL53L1X_ENABLED           22
#define PARAM_ID_VL53L1X_DISTANCE_MODE     23
#define PARAM_ID_VL53L1X_TIMING_BUDGET     24
#define PARAM_ID_VL53L1X_INTER_MEASUREMENT 25
#define PARAM_ID_VL53L1X_ROI_X_SIZE        26
#define PARAM_ID_VL53L1X_ROI_Y_SIZE        27
#define PARAM_ID_VL53L1X_ROI_CENTER_SPAD   28
#define PARAM_ID_VL53L1X_OFFSET_MM         29
#define PARAM_ID_VL53L1X_XTALK_CPS         30
#define PARAM_ID_VL53L1X_SIGNAL_THRESHOLD  31
#define PARAM_ID_VL53L1X_SIGMA_THRESHOLD   32
#define PARAM_ID_VL53L1X_THRESHOLD_LOW     33
#define PARAM_ID_VL53L1X_THRESHOLD_HIGH    34
#define PARAM_ID_VL53L1X_THRESHOLD_WINDOW  35
#define PARAM_ID_VL53L1X_INTERRUPT_POLARITY 36
#define PARAM_ID_VL53L1X_I2C_ADDRESS       37

/* Connection Parameters (50-55) */
#define PARAM_ID_DEFAULT_RPI          50
#define PARAM_ID_MAX_CONNECTIONS      51
#define PARAM_ID_ASSEMBLY_100_SIZE    53
#define PARAM_ID_ASSEMBLY_150_SIZE    54

/* Global pointer to Parameter Object class for instance creation */
static CipClass *s_parameter_class = NULL;

/* VL53L1X configuration structure - maintained for parameter access */
static system_vl53l1x_config_t s_vl53l1x_config;

/* Forward declarations */
static void InitializeCipParameter(CipClass *class);
static EipStatus CreateParameterInstance(
    CipClass *parameter_class,
    CipUint instance_id,
    const char *name,
    CipUsint data_type_code,
    void *value_ptr,
    const char *units,
    const char *help,
    void *min_ptr,
    void *max_ptr,
    void *default_ptr,
    CIPAttributeFlag attribute_flags);
static EipStatus ParameterPostSetCallback(CipInstance *const instance,
                                          CipAttributeStruct *const attribute,
                                          CipByte service);
static CipAttributeEncodeInMessage GetEncodeFunction(CipUsint data_type_code);
static CipAttributeDecodeFromMessage GetDecodeFunction(CipUsint data_type_code);

/* Initialize Parameter Object class */
static void InitializeCipParameter(CipClass *class) {
  CipClass *meta_class = class->class_instance.cip_class;

  /* Register class-level services */
  InsertService(meta_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");
  InsertService(meta_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
}

/* Parameter Object initialization */
EipStatus CipParameterInit(void) {
  /* Calculate number of instances we'll create
   * We need to allocate enough instances to cover the highest instance ID we use (54)
   * Active instances: 1-6, 9-19, 20-21, 22-37, 50-51, 53-54 (38 total)
   * Unused instances: 7-8, 38-49, 52 (14 total)
   * This creates some unused instances, but is necessary for non-sequential numbering
   */
  const CipInstanceNum num_instances = 54; /* Highest instance ID is 54 (Assembly 150 Size) */

  /* Create the Parameter Object class */
  s_parameter_class = CreateCipClass(
    kCipParameterClassCode,           /* class code */
    0,                                /* # of non-default class attributes */
    7,                                /* # highest class attribute number */
    2,                                /* # of class services */
    8,                                /* # of instance attributes */
    8,                                /* # highest instance attribute number */
    4,                                /* # of instance services (GetAttributeSingle, GetAttributeAll, SetAttributeSingle, SetAttributeAll) */
    num_instances,                    /* # of instances */
    "parameter",                      /* class name */
    PARAMETER_CLASS_REVISION,         /* class revision */
    &InitializeCipParameter          /* initialization function */
  );

  if (s_parameter_class == NULL) {
    OPENER_TRACE_ERR("Failed to create Parameter Object class\n");
    return kEipStatusError;
  }

  /* Set PostSetCallback for parameter value changes */
  s_parameter_class->PostSetCallback = ParameterPostSetCallback;

  /* Register instance services */
  InsertService(s_parameter_class, kGetAttributeSingle, &GetAttributeSingle, "GetAttributeSingle");
  InsertService(s_parameter_class, kGetAttributeAll, &GetAttributeAll, "GetAttributeAll");
  InsertService(s_parameter_class, kSetAttributeSingle, &SetAttributeSingle, "SetAttributeSingle");
  InsertService(s_parameter_class, kSetAttributeAll, &SetAttributeAll, "SetAttributeAll");

  /* Create network configuration parameters */
  extern CipTcpIpObject g_tcpip;
  
  /* Parameter 1: IP Address */
  CreateParameterInstance(s_parameter_class, PARAM_ID_IP_ADDRESS,
    "IP Address", kCipDword,
    &g_tcpip.interface_configuration.ip_address,
    "", "Device IP address in network byte order",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 2: Subnet Mask */
  CreateParameterInstance(s_parameter_class, PARAM_ID_SUBNET_MASK,
    "Subnet Mask", kCipDword,
    &g_tcpip.interface_configuration.network_mask,
    "", "Network subnet mask in network byte order",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 3: Gateway */
  CreateParameterInstance(s_parameter_class, PARAM_ID_GATEWAY,
    "Gateway", kCipDword,
    &g_tcpip.interface_configuration.gateway,
    "", "Default gateway IP address in network byte order",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 4: DNS Server 1 */
  CreateParameterInstance(s_parameter_class, PARAM_ID_DNS_SERVER_1,
    "DNS Server 1", kCipDword,
    &g_tcpip.interface_configuration.name_server,
    "", "Primary DNS server IP address in network byte order",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 5: DNS Server 2 */
  CreateParameterInstance(s_parameter_class, PARAM_ID_DNS_SERVER_2,
    "DNS Server 2", kCipDword,
    &g_tcpip.interface_configuration.name_server_2,
    "", "Secondary DNS server IP address in network byte order",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 6: DHCP Enabled - derive from config_control */
  static CipBool dhcp_enabled = 0;
  /* Initialize from config_control: 0x02 = DHCP, 0x00 = Static */
  dhcp_enabled = (g_tcpip.config_control & 0x0F) == 0x02 ? 1 : 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_DHCP_ENABLED,
    "DHCP Enabled", kCipBool,
    &dhcp_enabled,
    "", "Enable DHCP for automatic IP configuration (1=enabled, 0=static)",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);

  /* Parameter 7: Hostname */
  CreateParameterInstance(s_parameter_class, PARAM_ID_HOSTNAME,
    "Hostname", kCipString,
    &g_tcpip.hostname,
    "", "Device hostname (max 64 characters)",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);

  /* Parameter 8: Domain Name */
  CreateParameterInstance(s_parameter_class, PARAM_ID_DOMAIN_NAME,
    "Domain Name", kCipString,
    &g_tcpip.interface_configuration.domain_name,
    "", "Network domain name (max 48 characters)",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);

  /* Parameter 9: Multicast TTL Value */
  static CipUsint mcast_ttl_min = 0;
  static CipUsint mcast_ttl_max = 255;
  static CipUsint mcast_ttl_default = 1;
  CreateParameterInstance(s_parameter_class, PARAM_ID_MCAST_TTL,
    "Multicast TTL", kCipUsint,
    &g_tcpip.mcast_ttl_value,
    "", "Time-to-live value for multicast connections (0-255, default: 1)",
    &mcast_ttl_min, &mcast_ttl_max, &mcast_ttl_default, kGetableSingleAndAll | kSetable);

  /* Parameter 20: ACD Enable */
  CreateParameterInstance(s_parameter_class, PARAM_ID_ACD_ENABLED,
    "ACD Enabled", kCipBool,
    &g_tcpip.select_acd,
    "", "Enable Address Conflict Detection (1=enabled, 0=disabled)",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);

  /* Parameter 21: ACD Timeout */
  static CipUint acd_timeout = 10;  /* Default: 10 seconds (RFC 5227 DEFEND_INTERVAL) */
  static CipUint acd_timeout_min = 1;
  static CipUint acd_timeout_max = 3600;
  static CipUint acd_timeout_default = 10;
  CreateParameterInstance(s_parameter_class, PARAM_ID_ACD_TIMEOUT,
    "ACD Timeout", kCipUint,
    &acd_timeout,
    "seconds", "ACD timeout in seconds (1-3600, default: 10). Timeout for ACD conflict detection operations.",
    &acd_timeout_min, &acd_timeout_max, &acd_timeout_default, kGetableSingleAndAll | kSetable);

  /* Create NAU7802 scale parameters - load initial values from NVS at runtime */
  /* Parameter 10: NAU7802 Enabled */
  static CipBool nau7802_enabled;
  nau7802_enabled = system_nau7802_enabled_load() ? 1 : 0;
  static CipUsint nau7802_unit_min = 0;
  static CipUsint nau7802_unit_max = 2;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_ENABLED,
    "NAU7802 Enabled", kCipBool,
    &nau7802_enabled,
    "", "Enable/disable NAU7802 weight scale sensor",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 11: NAU7802 Unit */
  static CipUsint nau7802_unit; /* 0=grams, 1=pounds, 2=kilograms */
  nau7802_unit = system_nau7802_unit_load();
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_UNIT,
    "NAU7802 Unit", kCipUsint,
    &nau7802_unit,
    "", "Weight unit: 0=grams, 1=pounds, 2=kilograms",
    &nau7802_unit_min, &nau7802_unit_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 12: NAU7802 Gain */
  static CipUsint nau7802_gain; /* 0=x1, 7=x128 */
  nau7802_gain = system_nau7802_gain_load();
  static CipUsint nau7802_gain_min = 0;
  static CipUsint nau7802_gain_max = 7;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_GAIN,
    "NAU7802 Gain", kCipUsint,
    &nau7802_gain,
    "", "PGA gain setting: 0=x1, 1=x2, ..., 7=x128",
    &nau7802_gain_min, &nau7802_gain_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 13: NAU7802 Sample Rate */
  static CipUsint nau7802_sample_rate; /* 0=10 SPS, 7=320 SPS */
  nau7802_sample_rate = system_nau7802_sample_rate_load();
  static CipUsint nau7802_sample_rate_min = 0;
  static CipUsint nau7802_sample_rate_max = 7;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_SAMPLE_RATE,
    "NAU7802 Sample Rate", kCipUsint,
    &nau7802_sample_rate,
    "SPS", "Sample rate: 0=10, 1=20, 2=40, 3=80, 7=320 samples per second",
    &nau7802_sample_rate_min, &nau7802_sample_rate_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 14: NAU7802 Channel */
  static CipUsint nau7802_channel; /* 0=Channel 1, 1=Channel 2 */
  nau7802_channel = system_nau7802_channel_load();
  static CipUsint nau7802_channel_min = 0;
  static CipUsint nau7802_channel_max = 1;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_CHANNEL,
    "NAU7802 Channel", kCipUsint,
    &nau7802_channel,
    "", "Active channel: 0=Channel 1, 1=Channel 2",
    &nau7802_channel_min, &nau7802_channel_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 15: NAU7802 LDO */
  static CipUsint nau7802_ldo; /* 0=4.5V, 4=3.3V, 7=2.4V */
  nau7802_ldo = system_nau7802_ldo_load();
  static CipUsint nau7802_ldo_min = 0;
  static CipUsint nau7802_ldo_max = 7;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_LDO,
    "NAU7802 LDO", kCipUsint,
    &nau7802_ldo,
    "V", "LDO voltage setting: 0=4.5V, ..., 4=3.3V, ..., 7=2.4V",
    &nau7802_ldo_min, &nau7802_ldo_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 16: NAU7802 Average */
  static CipUsint nau7802_average; /* 1-50 samples */
  nau7802_average = system_nau7802_average_load();
  static CipUsint nau7802_average_min = 1;
  static CipUsint nau7802_average_max = 50;
  CreateParameterInstance(s_parameter_class, PARAM_ID_NAU7802_AVERAGE,
    "NAU7802 Average", kCipUsint,
    &nau7802_average,
    "samples", "Number of samples to average for readings (1-50)",
    &nau7802_average_min, &nau7802_average_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 19: NAU7802 Byte Offset - REMOVED
   * Byte offsets in assembly data are statically mapped (NAU7802_BYTE_START = 24)
   * and cannot be configured. The parameter was removed as it was never used.
   */

  /* Create VL53L1X time-of-flight sensor parameters */
  /* Load config structure from NVS - we'll maintain a static copy */
  system_vl53l1x_config_load(&s_vl53l1x_config);

  /* Parameter 21: VL53L1X Enabled */
  static CipBool vl53l1x_enabled;
  vl53l1x_enabled = system_vl53l1x_enabled_load() ? 1 : 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_ENABLED,
    "VL53L1X Enabled", kCipBool,
    &vl53l1x_enabled,
    "", "Enable/disable VL53L1X time-of-flight sensor",
    NULL, NULL, NULL, kGetableSingleAndAll | kSetable);

  /* Parameter 22: VL53L1X Distance Mode */
  static CipUint vl53l1x_distance_mode_min = 1;
  static CipUint vl53l1x_distance_mode_max = 2;
  static CipUint vl53l1x_distance_mode_default = 2;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_DISTANCE_MODE,
    "VL53L1X Distance Mode", kCipUint,
    &s_vl53l1x_config.distance_mode,
    "", "Distance mode: 1=Short (<1.3m), 2=Long (<4m)",
    &vl53l1x_distance_mode_min, &vl53l1x_distance_mode_max, &vl53l1x_distance_mode_default, kGetableSingleAndAll | kSetable);

  /* Parameter 23: VL53L1X Timing Budget */
  static CipUint vl53l1x_timing_budget_min = 15;
  static CipUint vl53l1x_timing_budget_max = 500;
  static CipUint vl53l1x_timing_budget_default = 100;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_TIMING_BUDGET,
    "VL53L1X Timing Budget", kCipUint,
    &s_vl53l1x_config.timing_budget_ms,
    "ms", "Timing budget in milliseconds (15, 20, 33, 50, 100, 200, 500)",
    &vl53l1x_timing_budget_min, &vl53l1x_timing_budget_max, &vl53l1x_timing_budget_default, kGetableSingleAndAll | kSetable);

  /* Parameter 24: VL53L1X Inter-Measurement Period */
  static CipUdint vl53l1x_inter_measurement_min = 15;
  static CipUdint vl53l1x_inter_measurement_max = 10000;
  static CipUdint vl53l1x_inter_measurement_default = 100;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_INTER_MEASUREMENT,
    "VL53L1X Inter-Measurement", kCipUdint,
    &s_vl53l1x_config.inter_measurement_ms,
    "ms", "Inter-measurement period in milliseconds (must be >= timing budget)",
    &vl53l1x_inter_measurement_min, &vl53l1x_inter_measurement_max, &vl53l1x_inter_measurement_default, kGetableSingleAndAll | kSetable);

  /* Parameter 25: VL53L1X ROI X Size */
  static CipUint vl53l1x_roi_x_min = 4;
  static CipUint vl53l1x_roi_x_max = 16;
  static CipUint vl53l1x_roi_x_default = 16;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_ROI_X_SIZE,
    "VL53L1X ROI X Size", kCipUint,
    &s_vl53l1x_config.roi_x_size,
    "", "Region of Interest width (4-16)",
    &vl53l1x_roi_x_min, &vl53l1x_roi_x_max, &vl53l1x_roi_x_default, kGetableSingleAndAll | kSetable);

  /* Parameter 26: VL53L1X ROI Y Size */
  static CipUint vl53l1x_roi_y_min = 4;
  static CipUint vl53l1x_roi_y_max = 16;
  static CipUint vl53l1x_roi_y_default = 16;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_ROI_Y_SIZE,
    "VL53L1X ROI Y Size", kCipUint,
    &s_vl53l1x_config.roi_y_size,
    "", "Region of Interest height (4-16)",
    &vl53l1x_roi_y_min, &vl53l1x_roi_y_max, &vl53l1x_roi_y_default, kGetableSingleAndAll | kSetable);

  /* Parameter 27: VL53L1X ROI Center SPAD */
  static CipUsint vl53l1x_roi_center_min = 0;
  static CipUsint vl53l1x_roi_center_max = 199;
  static CipUsint vl53l1x_roi_center_default = 199;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_ROI_CENTER_SPAD,
    "VL53L1X ROI Center SPAD", kCipUsint,
    &s_vl53l1x_config.roi_center_spad,
    "", "ROI center SPAD index (0-199, 199=center)",
    &vl53l1x_roi_center_min, &vl53l1x_roi_center_max, &vl53l1x_roi_center_default, kGetableSingleAndAll | kSetable);

  /* Parameter 28: VL53L1X Offset */
  static CipInt vl53l1x_offset_min = -128;
  static CipInt vl53l1x_offset_max = 127;
  static CipInt vl53l1x_offset_default = 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_OFFSET_MM,
    "VL53L1X Offset", kCipInt,
    (void*)&s_vl53l1x_config.offset_mm,
    "mm", "Range offset in millimeters (-128 to +127)",
    &vl53l1x_offset_min, &vl53l1x_offset_max, &vl53l1x_offset_default, kGetableSingleAndAll | kSetable);

  /* Parameter 29: VL53L1X Crosstalk */
  static CipUint vl53l1x_xtalk_min = 0;
  static CipUint vl53l1x_xtalk_max = 65535;
  static CipUint vl53l1x_xtalk_default = 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_XTALK_CPS,
    "VL53L1X Crosstalk", kCipUint,
    &s_vl53l1x_config.xtalk_cps,
    "cps", "Crosstalk compensation in counts per second (0-65535)",
    &vl53l1x_xtalk_min, &vl53l1x_xtalk_max, &vl53l1x_xtalk_default, kGetableSingleAndAll | kSetable);

  /* Parameter 30: VL53L1X Signal Threshold */
  static CipUint vl53l1x_signal_threshold_min = 0;
  static CipUint vl53l1x_signal_threshold_max = 65535;
  static CipUint vl53l1x_signal_threshold_default = 1024;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_SIGNAL_THRESHOLD,
    "VL53L1X Signal Threshold", kCipUint,
    &s_vl53l1x_config.signal_threshold_kcps,
    "kcps", "Signal threshold in kilocounts per second (0-65535)",
    &vl53l1x_signal_threshold_min, &vl53l1x_signal_threshold_max, &vl53l1x_signal_threshold_default, kGetableSingleAndAll | kSetable);

  /* Parameter 31: VL53L1X Sigma Threshold */
  static CipUint vl53l1x_sigma_threshold_min = 0;
  static CipUint vl53l1x_sigma_threshold_max = 65535;
  static CipUint vl53l1x_sigma_threshold_default = 15;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_SIGMA_THRESHOLD,
    "VL53L1X Sigma Threshold", kCipUint,
    &s_vl53l1x_config.sigma_threshold_mm,
    "mm", "Sigma threshold in millimeters (0-65535)",
    &vl53l1x_sigma_threshold_min, &vl53l1x_sigma_threshold_max, &vl53l1x_sigma_threshold_default, kGetableSingleAndAll | kSetable);

  /* Parameter 32: VL53L1X Threshold Low */
  static CipUint vl53l1x_threshold_low_min = 0;
  static CipUint vl53l1x_threshold_low_max = 4000;
  static CipUint vl53l1x_threshold_low_default = 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_THRESHOLD_LOW,
    "VL53L1X Threshold Low", kCipUint,
    &s_vl53l1x_config.threshold_low_mm,
    "mm", "Distance threshold low in millimeters (0-4000, 0=disabled)",
    &vl53l1x_threshold_low_min, &vl53l1x_threshold_low_max, &vl53l1x_threshold_low_default, kGetableSingleAndAll | kSetable);

  /* Parameter 33: VL53L1X Threshold High */
  static CipUint vl53l1x_threshold_high_min = 0;
  static CipUint vl53l1x_threshold_high_max = 4000;
  static CipUint vl53l1x_threshold_high_default = 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_THRESHOLD_HIGH,
    "VL53L1X Threshold High", kCipUint,
    &s_vl53l1x_config.threshold_high_mm,
    "mm", "Distance threshold high in millimeters (0-4000, 0=disabled)",
    &vl53l1x_threshold_high_min, &vl53l1x_threshold_high_max, &vl53l1x_threshold_high_default, kGetableSingleAndAll | kSetable);

  /* Parameter 34: VL53L1X Threshold Window */
  static CipUsint vl53l1x_threshold_window_min = 0;
  static CipUsint vl53l1x_threshold_window_max = 3;
  static CipUsint vl53l1x_threshold_window_default = 0;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_THRESHOLD_WINDOW,
    "VL53L1X Threshold Window", kCipUsint,
    &s_vl53l1x_config.threshold_window,
    "", "Threshold window: 0=Below, 1=Above, 2=Out, 3=In",
    &vl53l1x_threshold_window_min, &vl53l1x_threshold_window_max, &vl53l1x_threshold_window_default, kGetableSingleAndAll | kSetable);

  /* Parameter 35: VL53L1X Interrupt Polarity */
  static CipUsint vl53l1x_interrupt_polarity_min = 0;
  static CipUsint vl53l1x_interrupt_polarity_max = 1;
  static CipUsint vl53l1x_interrupt_polarity_default = 1;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_INTERRUPT_POLARITY,
    "VL53L1X Interrupt Polarity", kCipUsint,
    &s_vl53l1x_config.interrupt_polarity,
    "", "Interrupt polarity: 0=Active Low, 1=Active High",
    &vl53l1x_interrupt_polarity_min, &vl53l1x_interrupt_polarity_max, &vl53l1x_interrupt_polarity_default, kGetableSingleAndAll | kSetable);

  /* Parameter 36: VL53L1X I2C Address */
  static CipUsint vl53l1x_i2c_address_min = 0x29;
  static CipUsint vl53l1x_i2c_address_max = 0x7F;
  static CipUsint vl53l1x_i2c_address_default = 0x29;
  CreateParameterInstance(s_parameter_class, PARAM_ID_VL53L1X_I2C_ADDRESS,
    "VL53L1X I2C Address", kCipUsint,
    &s_vl53l1x_config.i2c_address,
    "", "I2C address (0x29-0x7F, default: 0x29)",
    &vl53l1x_i2c_address_min, &vl53l1x_i2c_address_max, &vl53l1x_i2c_address_default, kGetableSingleAndAll | kSetable);

  /* Create connection parameters */
  /* Parameter 50: Default RPI */
  static CipUint default_rpi = 1000; /* milliseconds */
  static CipUint default_rpi_min = 1;
  static CipUint default_rpi_max = 65535;
  CreateParameterInstance(s_parameter_class, PARAM_ID_DEFAULT_RPI,
    "Default RPI", kCipUint,
    &default_rpi,
    "ms", "Default Requested Packet Interval in milliseconds",
    &default_rpi_min, &default_rpi_max, NULL, kGetableSingleAndAll | kSetable);
  
  /* Parameter 51: Max Connections */
  static CipUsint max_connections = 24;
  static CipUsint max_connections_min = 1;
  static CipUsint max_connections_max = 24;
  CreateParameterInstance(s_parameter_class, PARAM_ID_MAX_CONNECTIONS,
    "Max Connections", kCipUsint,
    &max_connections,
    "", "Maximum number of simultaneous EtherNet/IP connections",
    &max_connections_min, &max_connections_max, NULL, kGetableSingleAndAll);
  
  /* Parameter 53: Assembly 100 Size */
  static CipUsint assembly_100_size = 72;
  CreateParameterInstance(s_parameter_class, PARAM_ID_ASSEMBLY_100_SIZE,
    "Assembly 100 Size", kCipUsint,
    &assembly_100_size,
    "bytes", "Size of Input Assembly 100 in bytes",
    NULL, NULL, NULL, kGetableSingleAndAll);
  
  /* Parameter 54: Assembly 150 Size */
  static CipUsint assembly_150_size = 40;
  CreateParameterInstance(s_parameter_class, PARAM_ID_ASSEMBLY_150_SIZE,
    "Assembly 150 Size", kCipUsint,
    &assembly_150_size,
    "bytes", "Size of Output Assembly 150 in bytes",
    NULL, NULL, NULL, kGetableSingleAndAll);

  OPENER_TRACE_INFO("Parameter Object initialized with %d instances (38 active)\n", num_instances);

  return kEipStatusOk;
}

/* MODIFICATION: Get ACD timeout value from Parameter Object Instance #21
 * Added by: Adam G. Sweeney <agsweeney@gmail.com>
 * Returns the ACD timeout in seconds, falling back to 10 seconds (RFC 5227 DEFEND_INTERVAL)
 * if the value is not set or invalid
 */
uint16_t CipParameterGetAcdTimeout(void) {
  if (s_parameter_class == NULL) {
    /* Parameter Object not initialized yet, return default */
    return 10;
  }

  CipInstance *instance = GetCipInstance(s_parameter_class, PARAM_ID_ACD_TIMEOUT);
  if (instance == NULL || instance->data == NULL) {
    /* Instance not found, return default */
    return 10;
  }

  CipParameterInstance *param = (CipParameterInstance *)instance->data;
  if (param->parameter_value == NULL) {
    /* Value not set, return default */
    return 10;
  }

  CipUint timeout = *(CipUint *)param->parameter_value;
  
  /* Validate range: 1-3600 seconds, otherwise fallback to 10 seconds */
  if (timeout >= 1 && timeout <= 3600) {
    return (uint16_t)timeout;
  }
  
  /* Fallback to RFC 5227 default DEFEND_INTERVAL (10 seconds) */
  return 10;
}

/* Helper function to get encode function for a data type */
static CipAttributeEncodeInMessage GetEncodeFunction(CipUsint data_type_code) {
  switch (data_type_code) {
    case kCipUdint: return EncodeCipUdint;
    case kCipUint: return EncodeCipUint;
    case kCipUsint: return EncodeCipUsint;
    case kCipBool: return EncodeCipBool;
    case kCipDword: return EncodeCipDword;
    case kCipWord: return EncodeCipWord;
    case kCipString: return EncodeCipString;
    case kCipInt: return EncodeCipInt;
    default: return NULL;
  }
}

/* Helper function to get decode function for a data type
 * Note: We cast through void * to handle the type mismatch between
 * specific typed decode functions and the generic CipAttributeDecodeFromMessage signature.
 * The typed decode functions (DecodeCipUdint, etc.) expect specific pointer types,
 * but CipAttributeDecodeFromMessage expects void *. This is safe because the actual
 * data pointer passed will be the correct type.
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wincompatible-pointer-types"
static CipAttributeDecodeFromMessage GetDecodeFunction(CipUsint data_type_code) {
  switch (data_type_code) {
    case kCipUdint: return (CipAttributeDecodeFromMessage)DecodeCipUdint;
    case kCipUint: return (CipAttributeDecodeFromMessage)DecodeCipUint;
    case kCipUsint: return (CipAttributeDecodeFromMessage)DecodeCipUsint;
    case kCipBool: return (CipAttributeDecodeFromMessage)DecodeCipBool;
    case kCipDword: return (CipAttributeDecodeFromMessage)DecodeCipDword;
    case kCipWord: return (CipAttributeDecodeFromMessage)DecodeCipWord;
    case kCipString: return (CipAttributeDecodeFromMessage)DecodeCipString;
    case kCipInt: return (CipAttributeDecodeFromMessage)DecodeCipInt;
    default: return NULL;
  }
}
#pragma GCC diagnostic pop

/* PostSetCallback to handle parameter value changes */
static EipStatus ParameterPostSetCallback(CipInstance *const instance,
                                          CipAttributeStruct *const attribute,
                                          CipByte service) {
  if (instance == NULL || instance->data == NULL) {
    return kEipStatusError;
  }

  CipParameterInstance *param = (CipParameterInstance *)instance->data;
  
  /* Only handle Attribute 2 (Parameter Value) changes */
  if (attribute->attribute_number != 2) {
    return kEipStatusOk;
  }

  OPENER_TRACE_INFO("Parameter %d (%s) value changed\n", 
                    param->parameter_id, 
                    param->parameter_name.string);

  /* Handle parameter-specific save logic */
  switch (param->parameter_id) {
    case PARAM_ID_DHCP_ENABLED: {
      extern CipTcpIpObject g_tcpip;
      CipBool *dhcp_val = (CipBool *)param->parameter_value;
      if (*dhcp_val) {
        g_tcpip.config_control = (g_tcpip.config_control & ~0x0F) | 0x02; /* Set to DHCP */
      } else {
        g_tcpip.config_control = (g_tcpip.config_control & ~0x0F) | 0x00; /* Set to Static */
      }
      /* Save to NVS via TCP/IP interface save function */
      extern CipTcpIpObject g_tcpip;
      NvTcpipStore(&g_tcpip);
      OPENER_TRACE_INFO("DHCP setting changed, saved to NVS\n");
      break;
    }

    case PARAM_ID_IP_ADDRESS:
    case PARAM_ID_SUBNET_MASK:
    case PARAM_ID_GATEWAY:
    case PARAM_ID_DNS_SERVER_1:
    case PARAM_ID_DNS_SERVER_2: {
      /* Network config changes - save to NVS */
      extern CipTcpIpObject g_tcpip;
      NvTcpipStore(&g_tcpip);
      OPENER_TRACE_INFO("Network configuration changed, saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_HOSTNAME:
    case PARAM_ID_DOMAIN_NAME: {
      /* Hostname and domain name changes - save to NVS */
      extern CipTcpIpObject g_tcpip;
      NvTcpipStore(&g_tcpip);
      OPENER_TRACE_INFO("Hostname/Domain name changed, saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_MCAST_TTL: {
      /* Multicast TTL changes - save to NVS */
      extern CipTcpIpObject g_tcpip;
      NvTcpipStore(&g_tcpip);
      OPENER_TRACE_INFO("Multicast TTL changed, saved to NVS (takes effect on next multicast connection)\n");
      break;
    }

    case PARAM_ID_ACD_ENABLED: {
      /* ACD enable changes - save to NVS */
      extern CipTcpIpObject g_tcpip;
      NvTcpipStore(&g_tcpip);
      OPENER_TRACE_INFO("ACD enabled state changed, saved to NVS (takes effect on next network configuration)\n");
      break;
    }

    case PARAM_ID_ACD_TIMEOUT: {
      /* ACD timeout changes - value is stored in parameter, no NVS needed (runtime setting) */
      OPENER_TRACE_INFO("ACD timeout changed to %u seconds (takes effect on next conflict)\n", 
                        *(CipUint *)param->parameter_value);
      break;
    }

    case PARAM_ID_VL53L1X_ENABLED: {
      CipBool *enabled = (CipBool *)param->parameter_value;
      system_vl53l1x_enabled_save(*enabled ? true : false);
      OPENER_TRACE_INFO("VL53L1X enabled state saved to NVS\n");
      break;
    }

    case PARAM_ID_VL53L1X_DISTANCE_MODE:
    case PARAM_ID_VL53L1X_TIMING_BUDGET:
    case PARAM_ID_VL53L1X_INTER_MEASUREMENT:
    case PARAM_ID_VL53L1X_ROI_X_SIZE:
    case PARAM_ID_VL53L1X_ROI_Y_SIZE:
    case PARAM_ID_VL53L1X_ROI_CENTER_SPAD:
    case PARAM_ID_VL53L1X_OFFSET_MM:
    case PARAM_ID_VL53L1X_XTALK_CPS:
    case PARAM_ID_VL53L1X_SIGNAL_THRESHOLD:
    case PARAM_ID_VL53L1X_SIGMA_THRESHOLD:
    case PARAM_ID_VL53L1X_THRESHOLD_LOW:
    case PARAM_ID_VL53L1X_THRESHOLD_HIGH:
    case PARAM_ID_VL53L1X_THRESHOLD_WINDOW:
    case PARAM_ID_VL53L1X_INTERRUPT_POLARITY:
    case PARAM_ID_VL53L1X_I2C_ADDRESS: {
      /* VL53L1X config changes - the parameter_value pointer points directly to s_vl53l1x_config,
       * so the value has already been updated by the decode function. Just save the config to NVS. */
      if (system_vl53l1x_config_save(&s_vl53l1x_config)) {
        OPENER_TRACE_INFO("VL53L1X parameter %d saved to NVS (reboot required for sensor reconfiguration)\n", param->parameter_id);
      } else {
        OPENER_TRACE_ERR("Failed to save VL53L1X parameter %d to NVS\n", param->parameter_id);
      }
      break;
    }

    case PARAM_ID_NAU7802_ENABLED: {
      CipBool *enabled = (CipBool *)param->parameter_value;
      system_nau7802_enabled_save(*enabled ? true : false);
      OPENER_TRACE_INFO("NAU7802 enabled state saved to NVS\n");
      break;
    }

    case PARAM_ID_NAU7802_UNIT: {
      CipUsint *unit = (CipUsint *)param->parameter_value;
      system_nau7802_unit_save(*unit);
      OPENER_TRACE_INFO("NAU7802 unit saved to NVS\n");
      break;
    }

    case PARAM_ID_NAU7802_GAIN: {
      CipUsint *gain = (CipUsint *)param->parameter_value;
      system_nau7802_gain_save(*gain);
      OPENER_TRACE_INFO("NAU7802 gain saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_NAU7802_SAMPLE_RATE: {
      CipUsint *sample_rate = (CipUsint *)param->parameter_value;
      system_nau7802_sample_rate_save(*sample_rate);
      OPENER_TRACE_INFO("NAU7802 sample rate saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_NAU7802_CHANNEL: {
      CipUsint *channel = (CipUsint *)param->parameter_value;
      system_nau7802_channel_save(*channel);
      OPENER_TRACE_INFO("NAU7802 channel saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_NAU7802_LDO: {
      CipUsint *ldo = (CipUsint *)param->parameter_value;
      system_nau7802_ldo_save(*ldo);
      OPENER_TRACE_INFO("NAU7802 LDO saved to NVS (reboot required)\n");
      break;
    }

    case PARAM_ID_NAU7802_AVERAGE: {
      CipUsint *average = (CipUsint *)param->parameter_value;
      system_nau7802_average_save(*average);
      OPENER_TRACE_INFO("NAU7802 average saved to NVS (takes effect immediately)\n");
      break;
    }

    /* PARAM_ID_NAU7802_BYTE_OFFSET removed - byte offsets are statically mapped */

    case PARAM_ID_DEFAULT_RPI: {
      /* RPI changes take effect on next connection */
      OPENER_TRACE_INFO("Default RPI changed (takes effect on next connection)\n");
      break;
    }

    default:
      OPENER_TRACE_WARN("Parameter %d change handler not implemented\n", param->parameter_id);
      break;
  }

  return kEipStatusOk;
}

/* Create a parameter instance and register its attributes */
static EipStatus CreateParameterInstance(
    CipClass *parameter_class,
    CipUint instance_id,
    const char *name,
    CipUsint data_type_code,
    void *value_ptr,
    const char *units,
    const char *help,
    void *min_ptr,
    void *max_ptr,
    void *default_ptr,
    CIPAttributeFlag attribute_flags)
{
  CipInstance *instance = GetCipInstance(parameter_class, instance_id);
  if (instance == NULL) {
    OPENER_TRACE_ERR("Failed to get Parameter instance %d\n", instance_id);
    return kEipStatusError;
  }

  /* Allocate parameter data structure with space for placeholder values */
  /* We need to allocate space for min/max/default placeholders if not provided */
  size_t alloc_size = sizeof(CipParameterInstance);
  size_t placeholder_size = 0;
  
  /* Determine placeholder size based on data type */
  switch (data_type_code) {
    case kCipUdint:
    case kCipDword:
      placeholder_size = sizeof(CipUdint);
      break;
    case kCipUint:
    case kCipWord:
      placeholder_size = sizeof(CipUint);
      break;
    case kCipUsint:
    case kCipBool:
      placeholder_size = sizeof(CipUsint);
      break;
    default:
      placeholder_size = sizeof(CipUdint); /* Default to largest */
      break;
  }
  
  /* Allocate space for parameter structure + 3 placeholders (min, max, default) */
  CipParameterInstance *param = (CipParameterInstance *)calloc(1, alloc_size + (placeholder_size * 3));
  if (param == NULL) {
    OPENER_TRACE_ERR("Failed to allocate Parameter instance data\n");
    return kEipStatusError;
  }

  instance->data = param;

  /* Initialize parameter data */
  SetCipShortStringByCstr(&param->parameter_name, name);
  param->data_type_code = data_type_code;
  param->parameter_value = value_ptr;
  param->parameter_id = instance_id;
  SetCipShortStringByCstr(&param->units, units ? units : "");
  SetCipShortStringByCstr(&param->help_string, help ? help : "");
  
  /* Set up placeholder pointers (stored after the structure) */
  void *placeholder_base = ((EipUint8 *)param) + alloc_size;
  void *min_placeholder = placeholder_base;
  void *max_placeholder = ((EipUint8 *)placeholder_base) + placeholder_size;
  void *default_placeholder = ((EipUint8 *)placeholder_base) + (placeholder_size * 2);

  CipAttributeEncodeInMessage encode_func = GetEncodeFunction(data_type_code);
  CipAttributeDecodeFromMessage decode_func = NULL;
  
  if (encode_func == NULL) {
    OPENER_TRACE_ERR("Unsupported parameter data type: %d for parameter %s\n", data_type_code, name);
    free(param);
    return kEipStatusError;
  }

  /* Get decode function if settable */
  if (attribute_flags & kSetable) {
    decode_func = GetDecodeFunction(data_type_code);
  }

  /* Register attributes using InsertAttribute */
  InsertAttribute(instance, 1, kCipShortString, EncodeCipShortString,
                  NULL, &param->parameter_name, kGetableSingleAndAll);
  
  /* Attribute 2: Parameter Value - include decode function if settable */
  if (value_ptr) {
    CIPAttributeFlag flags = attribute_flags;
    if (flags & kSetable) {
      flags |= kPostSetFunc; /* Enable PostSetCallback */
    }
    InsertAttribute(instance, 2, data_type_code, encode_func,
                   decode_func, value_ptr, flags);
  }

  InsertAttribute(instance, 3, kCipShortString, EncodeCipShortString,
                  NULL, &param->units, kGetableSingleAndAll);
  InsertAttribute(instance, 4, kCipShortString, EncodeCipShortString,
                  NULL, &param->help_string, kGetableSingleAndAll);
  
  /* Attributes 5, 6, 7 (Min, Max, Default) - always register to avoid "attribute not defined" warnings
   * Use the provided pointers if available, otherwise use placeholder values stored after the structure
   */
  if (min_ptr) {
    param->minimum_value = min_ptr;
    InsertAttribute(instance, 5, data_type_code, encode_func,
                    NULL, min_ptr, kGetableSingleAndAll);
  } else {
    /* Use placeholder - already zero-initialized by calloc */
    param->minimum_value = min_placeholder;
    InsertAttribute(instance, 5, data_type_code, encode_func,
                    NULL, min_placeholder, kGetableSingleAndAll);
  }
  
  if (max_ptr) {
    param->maximum_value = max_ptr;
    InsertAttribute(instance, 6, data_type_code, encode_func,
                    NULL, max_ptr, kGetableSingleAndAll);
  } else {
    /* Use placeholder - already zero-initialized by calloc */
    param->maximum_value = max_placeholder;
    InsertAttribute(instance, 6, data_type_code, encode_func,
                    NULL, max_placeholder, kGetableSingleAndAll);
  }
  
  if (default_ptr) {
    param->default_value = default_ptr;
    InsertAttribute(instance, 7, data_type_code, encode_func,
                    NULL, default_ptr, kGetableSingleAndAll);
  } else {
    /* Use placeholder - already zero-initialized by calloc */
    param->default_value = default_placeholder;
    InsertAttribute(instance, 7, data_type_code, encode_func,
                    NULL, default_placeholder, kGetableSingleAndAll);
  }
  
  InsertAttribute(instance, 8, kCipUsint, EncodeCipUsint,
                  NULL, &param->data_type_code, kGetableSingleAndAll);

  return kEipStatusOk;
}

