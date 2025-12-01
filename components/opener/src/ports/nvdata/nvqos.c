/*******************************************************************************
 * Copyright (c) 2019, Rockwell Automation, Inc.
 * All rights reserved.
 *
 * MODIFICATIONS:
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

/** @file nvqos.c
 *  @brief This file implements the functions to handle QoS object's NV data.
 *
 *  ESP32-specific implementation using NVS (Non-Volatile Storage) for persistence.
 *  Falls back to default values if NVS load fails.
 */
#include "nvqos.h"

#include <string.h>
#include <inttypes.h>

#include "cipqos.h"
#include "trace.h"

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"

#define QOS_NVS_NAMESPACE  "opener"   /**< NVS namespace for QoS data */
#define QOS_NVS_KEY        "qos_cfg"
#define QOS_NV_VERSION     1U

static const char *kTag = "NvQos";

/* Default DSCP values (from cipqos.c) */
#define DEFAULT_DSCP_EVENT 59U
#define DEFAULT_DSCP_GENERAL 47U
#define DEFAULT_DSCP_URGENT 55U
#define DEFAULT_DSCP_SCHEDULED 47U
#define DEFAULT_DSCP_HIGH 43U
#define DEFAULT_DSCP_LOW 31U
#define DEFAULT_DSCP_EXPLICIT 27U

typedef struct __attribute__((packed)) {
  uint32_t version;
  uint8_t q_frames_enable;
  uint8_t dscp_event;
  uint8_t dscp_general;
  uint8_t dscp_urgent;
  uint8_t dscp_scheduled;
  uint8_t dscp_high;
  uint8_t dscp_low;
  uint8_t dscp_explicit;
} QosNvBlob;

static esp_err_t QosNvOpen(nvs_handle_t *handle, nvs_open_mode_t mode) {
  esp_err_t err = nvs_open(QOS_NVS_NAMESPACE, mode, handle);
  if (ESP_ERR_NVS_NOT_FOUND == err && mode == NVS_READWRITE) {
    /* Namespace does not exist yet - create by opening in read/write */
    err = nvs_open(QOS_NVS_NAMESPACE, NVS_READWRITE, handle);
  }
  if (ESP_OK != err) {
    ESP_LOGE(kTag, "nvs_open failed (%s)", esp_err_to_name(err));
  }
  return err;
}

/** @brief Load NV data of the QoS object from NVS
 *
 *  @param  p_qos pointer to the QoS object's data structure
 *  @return kEipStatusOk: success; kEipStatusError: failure (will use defaults)
 */
EipStatus NvQosLoad(CipQosObject *p_qos) {
  nvs_handle_t handle;
  esp_err_t err = QosNvOpen(&handle, NVS_READONLY);
  if (ESP_ERR_NVS_NOT_FOUND == err) {
    ESP_LOGI(kTag, "No stored QoS configuration found, using defaults");
    return kEipStatusError;  /* Use defaults */
  } else if (ESP_OK != err) {
    ESP_LOGW(kTag, "Failed to open NVS for QoS (%s), using defaults", esp_err_to_name(err));
    return kEipStatusError;  /* Use defaults */
  }

  QosNvBlob blob = {0};
  size_t length = sizeof(blob);
  err = nvs_get_blob(handle, QOS_NVS_KEY, &blob, &length);
  nvs_close(handle);
  
  if (ESP_ERR_NVS_NOT_FOUND == err) {
    ESP_LOGI(kTag, "No stored QoS configuration found, using defaults");
    return kEipStatusError;  /* Use defaults */
  }
  
  if (ESP_OK != err) {
    ESP_LOGE(kTag, "Failed to load QoS configuration (%s), using defaults", esp_err_to_name(err));
    return kEipStatusError;  /* Use defaults */
  }

  if (length != sizeof(QosNvBlob)) {
    ESP_LOGW(kTag, "QoS configuration size mismatch (expected %zu, got %zu), using defaults",
             sizeof(QosNvBlob), length);
    return kEipStatusError;  /* Use defaults */
  }

  if (blob.version != QOS_NV_VERSION) {
    ESP_LOGW(kTag, "QoS configuration version mismatch (expected %" PRIu32 ", got %" PRIu32 "), using defaults",
             QOS_NV_VERSION, blob.version);
    return kEipStatusError;  /* Use defaults */
  }

  /* Validate DSCP values (must be < 64 as per RFC 2474) */
  if (blob.dscp_event >= 64 || blob.dscp_general >= 64 || blob.dscp_urgent >= 64 ||
      blob.dscp_scheduled >= 64 || blob.dscp_high >= 64 || blob.dscp_low >= 64 ||
      blob.dscp_explicit >= 64) {
    ESP_LOGW(kTag, "Invalid DSCP values in stored configuration (must be < 64), using defaults");
    return kEipStatusError;  /* Use defaults */
  }

  /* Load values from NVS */
  p_qos->q_frames_enable = blob.q_frames_enable;
  p_qos->dscp.event = blob.dscp_event;
  p_qos->dscp.general = blob.dscp_general;
  p_qos->dscp.urgent = blob.dscp_urgent;
  p_qos->dscp.scheduled = blob.dscp_scheduled;
  p_qos->dscp.high = blob.dscp_high;
  p_qos->dscp.low = blob.dscp_low;
  p_qos->dscp.explicit_msg = blob.dscp_explicit;

  ESP_LOGI(kTag, "QoS configuration loaded from NVS: Event=%d, General=%d, Urgent=%d, Scheduled=%d, High=%d, Low=%d, Explicit=%d",
           p_qos->dscp.event, p_qos->dscp.general, p_qos->dscp.urgent,
           p_qos->dscp.scheduled, p_qos->dscp.high, p_qos->dscp.low, p_qos->dscp.explicit_msg);
  
  return kEipStatusOk;
}

/** @brief Store NV data of the QoS object to NVS
 *
 *  @param  p_qos pointer to the QoS object's data structure
 *  @return kEipStatusOk: success; kEipStatusError: failure
 */
EipStatus NvQosStore(const CipQosObject *p_qos) {
  nvs_handle_t handle;
  esp_err_t err = QosNvOpen(&handle, NVS_READWRITE);
  if (ESP_OK != err) {
    ESP_LOGE(kTag, "Failed to open NVS for QoS store (%s)", esp_err_to_name(err));
    return kEipStatusError;
  }

  QosNvBlob blob = {0};
  blob.version = QOS_NV_VERSION;
  blob.q_frames_enable = p_qos->q_frames_enable;
  blob.dscp_event = p_qos->dscp.event;
  blob.dscp_general = p_qos->dscp.general;
  blob.dscp_urgent = p_qos->dscp.urgent;
  blob.dscp_scheduled = p_qos->dscp.scheduled;
  blob.dscp_high = p_qos->dscp.high;
  blob.dscp_low = p_qos->dscp.low;
  blob.dscp_explicit = p_qos->dscp.explicit_msg;

  err = nvs_set_blob(handle, QOS_NVS_KEY, &blob, sizeof(blob));
  if (ESP_OK != err) {
    ESP_LOGE(kTag, "Failed to set QoS blob in NVS (%s)", esp_err_to_name(err));
    nvs_close(handle);
    return kEipStatusError;
  }

  err = nvs_commit(handle);
  nvs_close(handle);
  if (ESP_OK != err) {
    ESP_LOGE(kTag, "Failed to commit QoS configuration to NVS (%s)", esp_err_to_name(err));
    return kEipStatusError;
  }

  ESP_LOGI(kTag, "QoS configuration saved to NVS: Event=%d, General=%d, Urgent=%d, Scheduled=%d, High=%d, Low=%d, Explicit=%d",
           p_qos->dscp.event, p_qos->dscp.general, p_qos->dscp.urgent,
           p_qos->dscp.scheduled, p_qos->dscp.high, p_qos->dscp.low, p_qos->dscp.explicit_msg);
  
  return kEipStatusOk;
}

#else  /* !ESP_PLATFORM */
/* Non-ESP32 implementation (file-based, original OpENer code) */

#include <limits.h>
#include <stdio.h>
#include "conffile.h"

#define QOS_CFG_NAME  "qos.cfg"

/** @brief Load NV data of the QoS object from file
 *
 *  @param  p_qos pointer to the QoS object's data structure
 *  @return kEipStatusOk: success; kEipStatusError: failure
 */
EipStatus NvQosLoad(CipQosObject *p_qos) {
  int rd_cnt = 0;
  EipStatus eip_status = kEipStatusError;

  /*
   * Data type for these parameters explicitly differs from the specification(CipUsint),
   * because the required scanf specifier to read that type, "hhu", is not supported
   * in MinGW. The generic unsigned type is used instead, which is guaranteed to fit
   * a CipUsint, followed by a check after scanf to ensure valid values before
   * casting them to CipUsint upon assignment to the output structure.
   */
  unsigned int dscp_urgent = 0;
  unsigned int dscp_scheduled = 0;
  unsigned int dscp_high = 0;
  unsigned int dscp_low = 0;
  unsigned int dscp_explicit = 0;

  FILE  *p_file = ConfFileOpen(false, QOS_CFG_NAME);
  if (NULL != p_file) {

/* Disable VS fscanf depreciation warning. */
#ifdef _MSC_VER
#pragma warning(disable : 4996)
#endif /* _MSC_VER */

    /* Read input data */
    rd_cnt = fscanf(p_file,
                    " %u, %u, %u, %u, %u\n",
                    &dscp_urgent,
                    &dscp_scheduled,
                    &dscp_high,
                    &dscp_low,
                    &dscp_explicit);

/* Restore default depreciation warning behavior. */
#ifdef _MSC_VER
#pragma warning(default : 4996)
#endif /* _MSC_VER */

    /* Need to try to close all stuff in any case. */
    eip_status = ConfFileClose(&p_file);
  }
  if (kEipStatusOk == eip_status) {

    /*
     * Ensure all values read from the configuration file are within the CipUsint range;
     * see above comments for these dscp_* local variables for details.
     */
    if ( (dscp_urgent > UCHAR_MAX)
         || (dscp_scheduled > UCHAR_MAX)
         || (dscp_high > UCHAR_MAX)
         || (dscp_low > UCHAR_MAX)
         || (dscp_explicit > UCHAR_MAX) ) {
      rd_cnt = 0;
    }

    /* If all data were read copy them to the global QoS object. */
    if (5 == rd_cnt) {
      p_qos->dscp.urgent = (CipUsint)dscp_urgent;
      p_qos->dscp.scheduled = (CipUsint)dscp_scheduled;
      p_qos->dscp.high = (CipUsint)dscp_high;
      p_qos->dscp.low = (CipUsint)dscp_low;
      p_qos->dscp.explicit_msg = (CipUsint)dscp_explicit;
    } else {
      eip_status = kEipStatusError;
    }
  }
  return eip_status;
}

/** @brief Store NV data of the QoS object to file
 *
 *  @param  p_qos pointer to the QoS object's data structure
 *  @return kEipStatusOk: success; kEipStatusError: failure
 */
EipStatus NvQosStore(const CipQosObject *p_qos) {
  FILE  *p_file = ConfFileOpen(true, QOS_CFG_NAME);
  EipStatus eip_status = kEipStatusOk;
  if (NULL != p_file) {
    /* Print output data */
    if ( 0 >= fprintf(p_file,
                      " %" PRIu8 ", %" PRIu8 ", %" PRIu8 ", %" PRIu8 ", %" PRIu8
                      "\n",
                      p_qos->dscp.urgent,
                      p_qos->dscp.scheduled,
                      p_qos->dscp.high,
                      p_qos->dscp.low,
                      p_qos->dscp.explicit_msg) ) {
      eip_status = kEipStatusError;
    }

    /* Need to try to close all stuff in any case. */
    eip_status =
      ( kEipStatusError ==
        ConfFileClose(&p_file) ) ? kEipStatusError : eip_status;
  }
  return eip_status;
}

#endif /* ESP_PLATFORM */
