/*
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
 */

#include "webui_api.h"
#include "ota_manager.h"
#include "system_config.h"
#include "driver/i2c_master.h"
#include "ciptcpipinterface.h"
#include "nvtcpip.h"
#include "log_buffer.h"
#include "nau7802.h"
#include "nau7802_calibration_storage.h"
#include "vl53l1x_manager.h"
#include "lsm6ds3_manager.h"
#include "gp8403_dac_manager.h"
#include "mcp230xx_manager.h"
#include "mcp_config.h"
#include "fusion_core_assembly.h"
#include "i2c_bus_manager.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/inet.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef _GNU_SOURCE
#define _GNU_SOURCE  /* For memmem() */
#endif

// Forward declarations for assembly access
extern uint8_t g_assembly_data064[72];
extern uint8_t g_assembly_data096[40];
extern uint8_t g_assembly_data097[10];

// Forward declaration for assembly mutex access
extern SemaphoreHandle_t fusion_core_get_assembly_mutex(void);
extern SemaphoreHandle_t scale_application_get_assembly_mutex(void);  // Backward compatibility

static const char *TAG = "webui_api";

// Cache for I2C pull-up enabled state to avoid frequent NVS reads
static bool s_cached_i2c_pullup_enabled = false;
static bool s_i2c_pullup_enabled_cached = false;
static bool s_cached_i2c_primary_pullup = false;
static bool s_i2c_primary_pullup_cached = false;
static bool s_cached_i2c_secondary_pullup = false;
static bool s_i2c_secondary_pullup_cached = false;

// Cache for device enabled states to avoid frequent NVS reads
static bool s_cached_vl53l1x_enabled = false;
static bool s_vl53l1x_enabled_cached = false;
static bool s_cached_lsm6ds3_enabled = false;
static bool s_lsm6ds3_enabled_cached = false;
static bool s_cached_gp8403_enabled = false;
static bool s_gp8403_enabled_cached = false;
static bool s_cached_mcp_enabled = false;
static bool s_mcp_enabled_cached = false;

// Cache for MCP config to avoid frequent NVS reads
static mcp_config_t s_cached_mcp_config = {0};
static bool s_mcp_config_cached = false;

// Cache for NAU7802 enabled state to avoid frequent NVS reads
static bool s_cached_nau7802_enabled = false;
static bool s_nau7802_enabled_cached = false;
static uint8_t s_cached_nau7802_unit = 1;  // Default to lbs
static bool s_nau7802_unit_cached = false;
static uint8_t s_cached_nau7802_gain = 7;  // Default to x128
static bool s_nau7802_gain_cached = false;
static uint8_t s_cached_nau7802_sample_rate = 3;  // Default to 80 SPS
static bool s_nau7802_sample_rate_cached = false;
static uint8_t s_cached_nau7802_channel = 0;  // Default to Channel 1
static bool s_nau7802_channel_cached = false;
static uint8_t s_cached_nau7802_ldo = 4;  // Default to 3.3V
static bool s_nau7802_ldo_cached = false;
static uint8_t s_cached_nau7802_average = 1;  // Default to 1 sample (no averaging)
static bool s_nau7802_average_cached = false;

// Cache for NAU7802 calibration values from NVS (avoid device reads)
static float s_cached_nau7802_cal_factor = 1.0f;
static float s_cached_nau7802_zero_offset = 0.0f;
static int32_t s_cached_nau7802_ch1_offset = 0;
static bool s_nau7802_cal_cached = false;

// Forward declarations for NAU7802 access functions (implemented in main.c)
// Note: Handles return NULL as NAU7802 is now managed by nau7802_manager component
extern void* fusion_core_get_nau7802_handle(void);
extern bool fusion_core_is_nau7802_initialized(void);
extern SemaphoreHandle_t fusion_core_get_nau7802_mutex(void);
extern void* scale_application_get_nau7802_handle(void);  // Backward compatibility
extern bool scale_application_is_nau7802_initialized(void);  // Backward compatibility
extern SemaphoreHandle_t scale_application_get_nau7802_mutex(void);  // Backward compatibility

// Mutex for protecting g_tcpip structure access (shared between OpENer task and API handlers)
static SemaphoreHandle_t s_tcpip_mutex = NULL;

// Helper function to send JSON response
static esp_err_t send_json_response(httpd_req_t *req, cJSON *json, esp_err_t status_code)
{
    char *json_str = cJSON_Print(json);
    if (json_str == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status_code == ESP_OK ? "200 OK" : "400 Bad Request");
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// Helper function to send JSON error response
static esp_err_t send_json_error(httpd_req_t *req, const char *message, int http_status)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "error");
    cJSON_AddStringToObject(json, "message", message);
    
    char *json_str = cJSON_Print(json);
    if (json_str == NULL) {
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    httpd_resp_set_type(req, "application/json");
    if (http_status == 400) {
        httpd_resp_set_status(req, "400 Bad Request");
    } else if (http_status == 500) {
        httpd_resp_set_status(req, "500 Internal Server Error");
    } else {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    httpd_resp_send(req, json_str, strlen(json_str));
    
    free(json_str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /api/reboot - Reboot the device
static esp_err_t api_reboot_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Reboot requested via web UI");
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "Device rebooting...");
    
    esp_err_t ret = send_json_response(req, response, ESP_OK);
    
    // Give a small delay to ensure response is sent
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Reboot the device
    esp_restart();
    
    return ret; // This will never be reached
}

// POST /api/ota/update - Trigger OTA update (supports both URL and file upload)
static esp_err_t api_ota_update_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "OTA update request received");
    
    // Check content type
    char content_type[256];
    esp_err_t ret = httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Missing Content-Type header (ret=%d)", ret);
        return send_json_error(req, "Missing Content-Type", 400);
    }
    
    ESP_LOGI(TAG, "OTA update request, Content-Type: %s", content_type);
    
    // Handle file upload (multipart/form-data) - Use streaming to avoid memory issues
    if (strstr(content_type, "multipart/form-data") != NULL) {
        // Get content length - may be 0 if chunked transfer
        size_t content_len = req->content_len;
        ESP_LOGI(TAG, "Content-Length: %d", content_len);
        
        // Get OTA partition size to validate against actual partition capacity
        const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
        size_t max_firmware_size = 0;
        if (update_partition != NULL) {
            max_firmware_size = update_partition->size;
            ESP_LOGI(TAG, "OTA partition size: %d bytes", max_firmware_size);
        } else {
            // Fallback to 1.5MB if partition lookup fails (matches partition table)
            max_firmware_size = 0x180000; // 1,572,864 bytes
            ESP_LOGW(TAG, "Could not determine partition size, using default: %d bytes", max_firmware_size);
        }
        
        // Validate size against partition capacity
        if (content_len > 0 && content_len > max_firmware_size) {
            ESP_LOGW(TAG, "Content length too large: %d bytes (max: %d bytes)", content_len, max_firmware_size);
            return send_json_error(req, "File too large for OTA partition", 400);
        }
        
        // Parse multipart boundary first
        const char *boundary_str = strstr(content_type, "boundary=");
        if (boundary_str == NULL) {
            ESP_LOGW(TAG, "No boundary found in Content-Type");
            return send_json_error(req, "Invalid multipart data: no boundary", 400);
        }
        boundary_str += 9; // Skip "boundary="
        
        // Extract boundary value
        char boundary[128];
        int boundary_len = 0;
        while (*boundary_str && *boundary_str != ';' && *boundary_str != ' ' && *boundary_str != '\r' && *boundary_str != '\n' && boundary_len < 127) {
            boundary[boundary_len++] = *boundary_str++;
        }
        boundary[boundary_len] = '\0';
        ESP_LOGI(TAG, "Multipart boundary: %s", boundary);
        
        // Use a small buffer to read multipart headers (64KB should be enough)
        const size_t header_buffer_size = 64 * 1024;
        char *header_buffer = malloc(header_buffer_size);
        if (header_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for header buffer");
            return send_json_error(req, "Failed to allocate memory", 500);
        }
        
        // Read enough to find the data separator (\r\n\r\n)
        size_t header_read = 0;
        bool found_separator = false;
        uint32_t header_timeout_count = 0;
        const uint32_t max_header_timeouts = 50; // Max 50 timeouts (~5 seconds at 100ms each)
        
        while (header_read < header_buffer_size - 1) {
            int ret = httpd_req_recv(req, header_buffer + header_read, header_buffer_size - header_read - 1);
            if (ret <= 0) {
                if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                    header_timeout_count++;
                    if (header_timeout_count > max_header_timeouts) {
                        ESP_LOGE(TAG, "Too many timeouts reading multipart headers");
                        free(header_buffer);
                        return send_json_error(req, "Timeout reading request headers", 408);
                    }
                    continue;
                }
                ESP_LOGE(TAG, "Error reading headers: %d", ret);
                free(header_buffer);
                return send_json_error(req, "Failed to read request headers", 500);
            }
            header_timeout_count = 0; // Reset timeout counter on successful read
            header_read += ret;
            header_buffer[header_read] = '\0'; // Null terminate for string search
            
            // Look for data separator
            if (strstr(header_buffer, "\r\n\r\n") != NULL || strstr(header_buffer, "\n\n") != NULL) {
                found_separator = true;
                break;
            }
        }
        
        if (!found_separator) {
            ESP_LOGW(TAG, "Could not find data separator in multipart headers");
            free(header_buffer);
            return send_json_error(req, "Invalid multipart format: no data separator", 400);
        }
        
        // Find where data starts
        char *data_start = strstr(header_buffer, "\r\n\r\n");
        size_t header_len = 0;
        if (data_start != NULL) {
            header_len = (data_start - header_buffer) + 4;
        } else {
            data_start = strstr(header_buffer, "\n\n");
            if (data_start != NULL) {
                header_len = (data_start - header_buffer) + 2;
            } else {
                free(header_buffer);
                return send_json_error(req, "Invalid multipart format", 400);
            }
        }
        
        // Calculate how much data we already have in the buffer
        size_t data_in_buffer = header_read - header_len;
        
        // Start streaming OTA update
        // Estimate firmware size: Content-Length minus multipart headers (typically ~1KB)
        // This gives us a reasonable estimate for progress tracking
        size_t estimated_firmware_size = 0;
        if (content_len > 0) {
            // Subtract estimated multipart header overhead (boundary + headers ~1KB)
            estimated_firmware_size = (content_len > 1024) ? (content_len - 1024) : content_len;
        }
        esp_ota_handle_t ota_handle = ota_manager_start_streaming_update(estimated_firmware_size);
        if (ota_handle == 0) {
            ESP_LOGE(TAG, "Failed to start streaming OTA update - check serial logs for details");
            free(header_buffer);
            return send_json_error(req, "Failed to start OTA update. Check device logs for details.", 500);
        }
        
        // Prepare boundary strings for detection
        char start_boundary[256];
        char end_boundary[256];
        snprintf(start_boundary, sizeof(start_boundary), "--%s", boundary);
        snprintf(end_boundary, sizeof(end_boundary), "--%s--", boundary);
        
        bool done = false;
        
        // Write data we already have in buffer (check for boundary first)
        if (data_in_buffer > 0) {
            // Check if boundary is already in the initial data
            char *boundary_in_header = strstr((char *)(header_buffer + header_len), start_boundary);
            
            if (boundary_in_header != NULL) {
                // Boundary found in initial data - only write up to it
                size_t initial_to_write = boundary_in_header - (header_buffer + header_len);
                // Remove trailing \r\n
                while (initial_to_write > 0 && 
                       (header_buffer[header_len + initial_to_write - 1] == '\r' || 
                        header_buffer[header_len + initial_to_write - 1] == '\n')) {
                    initial_to_write--;
                }
                if (initial_to_write > 0) {
                    if (!ota_manager_write_streaming_chunk(ota_handle, (const uint8_t *)(header_buffer + header_len), initial_to_write)) {
                        ESP_LOGE(TAG, "Failed to write initial chunk");
                        free(header_buffer);
                        return send_json_error(req, "Failed to write firmware data", 500);
                    }
                    data_in_buffer = initial_to_write;
                } else {
                    data_in_buffer = 0;  // No data to write, boundary was at start
                }
                done = true;  // We're done, boundary found
            } else {
                // No boundary in initial data, write it all
                if (!ota_manager_write_streaming_chunk(ota_handle, (const uint8_t *)(header_buffer + header_len), data_in_buffer)) {
                    ESP_LOGE(TAG, "Failed to write initial chunk");
                    free(header_buffer);
                    return send_json_error(req, "Failed to write firmware data", 500);
                }
            }
        }
        
        free(header_buffer); // Free header buffer, we'll use a smaller chunk buffer now
        
        // If boundary was found in initial data, we're done
        if (done) {
            ESP_LOGI(TAG, "Streamed %d bytes to OTA partition", data_in_buffer);
            
            // Finish OTA update
            cJSON *response = cJSON_CreateObject();
            cJSON_AddStringToObject(response, "status", "ok");
            cJSON_AddStringToObject(response, "message", "Firmware uploaded successfully. Finishing update and rebooting...");
            
            esp_err_t resp_err = send_json_response(req, response, ESP_OK);
            vTaskDelay(pdMS_TO_TICKS(100));
            
            if (!ota_manager_finish_streaming_update(ota_handle)) {
                ESP_LOGE(TAG, "Failed to finish streaming OTA update");
                return ESP_FAIL;
            }
            return resp_err;
        }
        
        // Stream remaining data in chunks (64KB chunks)
        const size_t chunk_size = 64 * 1024;
        char *chunk_buffer = malloc(chunk_size);
        if (chunk_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate chunk buffer");
            // Abort OTA update since we can't continue
            esp_ota_abort(ota_handle);
            return send_json_error(req, "Failed to allocate memory", 500);
        }
        
        // Keep a small rolling buffer to detect boundaries that might be split across chunks
        // Boundary can be up to ~200 bytes, so we need overlap of previous chunk's end
        // Use 2KB buffer - enough for boundary + some overlap, but small enough to manage
        const size_t boundary_overlap_buffer_size = 2048;
        char *boundary_overlap_buffer = malloc(boundary_overlap_buffer_size);
        size_t boundary_overlap_buffer_fill = 0;
        if (boundary_overlap_buffer == NULL) {
            ESP_LOGE(TAG, "Failed to allocate boundary overlap buffer");
            free(chunk_buffer);
            esp_ota_abort(ota_handle);
            return send_json_error(req, "Failed to allocate memory", 500);
        }
        
        size_t total_written = data_in_buffer;
        uint32_t timeout_count = 0;
        const uint32_t max_timeouts = 100; // Max 100 timeouts (~10 seconds at 100ms each)
        
        // Calculate expected firmware size for validation
        size_t expected_firmware_bytes = 0;
        if (content_len > 0) {
            // Account for multipart overhead (boundary + headers, typically ~1KB)
            expected_firmware_bytes = (content_len > 1024) ? (content_len - 1024) : content_len;
        }
        
        while (!done) {
            // Read chunk from socket - may return less than chunk_size (especially near end of upload)
            // This is normal socket behavior - ret will be the actual bytes available
            int ret = httpd_req_recv(req, chunk_buffer, chunk_size);
            if (ret <= 0) {
                if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                    timeout_count++;
                    if (timeout_count > max_timeouts) {
                        ESP_LOGE(TAG, "Too many timeouts during upload, aborting");
                        esp_ota_abort(ota_handle);
                        free(chunk_buffer);
                        return send_json_error(req, "Upload timeout - connection too slow", 408);
                    }
                    continue;
                }
                
                // ret == 0 means connection closed by client (EOF)
                // This is normal when client finishes sending data
                // If we haven't found the boundary yet, search the last chunk buffer
                if (ret == 0) {
                    if (done) {
                        // Already found boundary, connection close is expected
                        ESP_LOGI(TAG, "Connection closed by client after boundary found, total written: %d bytes", total_written);
                        break;
                    } else {
                        // Connection closed without finding boundary - search overlap buffer
                        if (boundary_overlap_buffer_fill > 0) {
                            ESP_LOGI(TAG, "Searching overlap buffer (%d bytes) for boundary after connection close", boundary_overlap_buffer_fill);
                            char *boundary_pos_found = NULL;
                            
                            // Search overlap buffer for boundary
                            char *end_match = (char *)memmem(boundary_overlap_buffer, boundary_overlap_buffer_fill, end_boundary, strlen(end_boundary));
                            if (end_match != NULL) {
                                bool is_valid = false;
                                if (end_match == boundary_overlap_buffer) {
                                    is_valid = true;
                                } else if (end_match > boundary_overlap_buffer && end_match[-1] == '\n') {
                                    if (end_match > boundary_overlap_buffer + 1 && end_match[-2] == '\r') {
                                        is_valid = true;
                                    } else if (end_match == boundary_overlap_buffer + 1) {
                                        is_valid = true;
                                    }
                                }
                                if (is_valid) {
                                    boundary_pos_found = end_match;
                                }
                            }
                            
                            // If not found, check for start boundary
                            if (boundary_pos_found == NULL) {
                                char *start_match = (char *)memmem(boundary_overlap_buffer, boundary_overlap_buffer_fill, start_boundary, strlen(start_boundary));
                                if (start_match != NULL) {
                                    bool is_valid = false;
                                    if (start_match == boundary_overlap_buffer) {
                                        is_valid = true;
                                    } else if (start_match > boundary_overlap_buffer && start_match[-1] == '\n') {
                                        if (start_match > boundary_overlap_buffer + 1 && start_match[-2] == '\r') {
                                            is_valid = true;
                                        } else if (start_match == boundary_overlap_buffer + 1) {
                                            is_valid = true;
                                        }
                                    }
                                    if (is_valid) {
                                        size_t end_boundary_len = strlen(end_boundary);
                                        if (start_match + end_boundary_len > boundary_overlap_buffer + boundary_overlap_buffer_fill || 
                                            memcmp(start_match, end_boundary, end_boundary_len) != 0) {
                                            boundary_pos_found = start_match;
                                        }
                                    }
                                }
                            }
                            
                            if (boundary_pos_found != NULL) {
                                // Found boundary in overlap buffer - this means we wrote too much
                                size_t bytes_before_boundary = boundary_pos_found - boundary_overlap_buffer;
                                // Remove trailing \r\n
                                while (bytes_before_boundary > 0 && 
                                       (boundary_overlap_buffer[bytes_before_boundary - 1] == '\r' || 
                                        boundary_overlap_buffer[bytes_before_boundary - 1] == '\n')) {
                                    bytes_before_boundary--;
                                }
                                if (bytes_before_boundary >= 2 && 
                                    boundary_overlap_buffer[bytes_before_boundary - 2] == '\r' && 
                                    boundary_overlap_buffer[bytes_before_boundary - 1] == '\n') {
                                    bytes_before_boundary -= 2;
                                } else if (bytes_before_boundary >= 1 && 
                                           boundary_overlap_buffer[bytes_before_boundary - 1] == '\n') {
                                    bytes_before_boundary -= 1;
                                }
                                
                                size_t extra_bytes = boundary_overlap_buffer_fill - bytes_before_boundary;
                                ESP_LOGE(TAG, "Found boundary in overlap buffer after close - estimated %d extra bytes written (boundary at offset %d), aborting OTA", 
                                         extra_bytes, (int)(boundary_pos_found - boundary_overlap_buffer));
                                esp_ota_abort(ota_handle);
                                free(chunk_buffer);
                                free(boundary_overlap_buffer);
                                return send_json_error(req, "Boundary detected in firmware data - upload corrupted, please retry", 400);
                            } else {
                                ESP_LOGW(TAG, "Boundary not found in overlap buffer (%d bytes)", boundary_overlap_buffer_fill);
                            }
                        }
                        
                        ESP_LOGW(TAG, "Connection closed by client without finding boundary, received %d bytes (expected ~%d)", 
                                 total_written, expected_firmware_bytes);
                        break;
                    }
                } else {
                    // Negative ret value indicates actual error
                    ESP_LOGE(TAG, "Connection error during upload (ret=%d), aborting OTA", ret);
                    esp_ota_abort(ota_handle);
                    free(chunk_buffer);
                    return send_json_error(req, "Connection error during upload", 500);
                }
            }
            timeout_count = 0; // Reset timeout counter on successful read
            
            // Search for boundary - check current chunk first, then check for split boundaries
            char *boundary_pos = NULL;
            size_t to_write = ret;
            size_t boundary_len = strlen(end_boundary);
            size_t max_boundary_len = (strlen(start_boundary) > boundary_len) ? strlen(start_boundary) : boundary_len;
            
            // Determine search range in current chunk:
            // - If close to expected end (within 20KB), search entire chunk aggressively
            // - Otherwise, search last portion to avoid false positives in binary data
            size_t chunk_search_start = 0;
            size_t chunk_search_len = ret;
            
            // Start searching entire chunks when we're within 20KB of expected end
            // This ensures we catch the boundary early enough (boundary + multipart overhead can be ~1KB)
            const size_t aggressive_search_threshold = 20 * 1024; // 20KB
            
            if (expected_firmware_bytes > 0) {
                size_t remaining_bytes = expected_firmware_bytes - total_written;
                if (remaining_bytes <= aggressive_search_threshold) {
                    // Close to expected end - search entire chunk aggressively
                    chunk_search_start = 0;
                    chunk_search_len = ret;
                    ESP_LOGD(TAG, "Aggressive search: remaining=%d, searching entire chunk of %d bytes", remaining_bytes, ret);
                } else if (ret > max_boundary_len + 1024) {
                    // Large chunk, not close to end - search last 1KB (increased from 512 for better detection)
                    chunk_search_start = ret - (max_boundary_len + 1024);
                    chunk_search_len = max_boundary_len + 1024;
                }
                // else: small chunk, search all of it
            } else if (ret > max_boundary_len + 1024) {
                // No expected size - search last 1KB
                chunk_search_start = ret - (max_boundary_len + 1024);
                chunk_search_len = max_boundary_len + 1024;
            }
            // else: small chunk, search all of it
            
            // First, search current chunk directly for boundary
            char *chunk_search_ptr = chunk_buffer + chunk_search_start;
            
            // Check for end boundary (--boundary--) first
            char *end_match = (char *)memmem(chunk_search_ptr, chunk_search_len, end_boundary, strlen(end_boundary));
            if (end_match != NULL) {
                // Verify it's actually a boundary - check for \r\n before it
                bool is_valid_boundary = false;
                if (end_match == chunk_buffer) {
                    is_valid_boundary = true;
                } else if (end_match > chunk_buffer) {
                    if (end_match[-1] == '\n') {
                        if (end_match > chunk_buffer + 1 && end_match[-2] == '\r') {
                            is_valid_boundary = true; // \r\n
                        } else if (end_match == chunk_buffer + 1) {
                            is_valid_boundary = true; // Just \n at start
                        }
                    }
                }
                if (is_valid_boundary) {
                    boundary_pos = end_match;
                    ESP_LOGI(TAG, "Found END boundary (--boundary--) at offset %d in chunk", (int)(end_match - chunk_buffer));
                }
            }
            
            // If not found in chunk, check for start boundary (--boundary)
            // Note: Start boundary indicates another part, but we only want the first part (firmware)
            // So we should also stop on start boundary if we're past expected size
            if (boundary_pos == NULL) {
                char *start_match = (char *)memmem(chunk_search_ptr, chunk_search_len, start_boundary, strlen(start_boundary));
                if (start_match != NULL) {
                    // Verify it's actually a boundary
                    bool is_valid_boundary = false;
                    if (start_match == chunk_buffer) {
                        is_valid_boundary = true;
                    } else if (start_match > chunk_buffer) {
                        if (start_match[-1] == '\n') {
                            if (start_match > chunk_buffer + 1 && start_match[-2] == '\r') {
                                is_valid_boundary = true;
                            } else if (start_match == chunk_buffer + 1) {
                                is_valid_boundary = true;
                            }
                        }
                    }
                    if (is_valid_boundary) {
                        // Make sure it's not the end boundary
                        size_t end_boundary_len = strlen(end_boundary);
                        if (start_match + end_boundary_len > chunk_buffer + ret || 
                            memcmp(start_match, end_boundary, end_boundary_len) != 0) {
                            // Found start boundary - this indicates another part
                            // If we're past expected size, treat this as end of firmware
                            if (expected_firmware_bytes > 0 && total_written >= expected_firmware_bytes - 500) {
                                ESP_LOGI(TAG, "Found START boundary (--boundary) at offset %d, but past expected size (%d >= %d), treating as end", 
                                         (int)(start_match - chunk_buffer), total_written, expected_firmware_bytes - 500);
                                boundary_pos = start_match;
                            } else {
                                ESP_LOGI(TAG, "Found START boundary (--boundary) at offset %d - indicates another part, stopping firmware write", 
                                         (int)(start_match - chunk_buffer));
                                boundary_pos = start_match;
                            }
                        }
                    }
                }
            }
            
            // If boundary not found in current chunk, check for split boundary across chunks
            // Combine overlap buffer (end of previous chunk) with start of current chunk
            if (boundary_pos == NULL && boundary_overlap_buffer_fill > 0) {
                // Create combined search area: overlap buffer + start of current chunk
                size_t combined_search_size = boundary_overlap_buffer_fill + (ret < max_boundary_len + 100 ? ret : max_boundary_len + 100);
                char *combined_search = malloc(combined_search_size);
                if (combined_search != NULL) {
                    // Copy overlap buffer (end of previous chunk)
                    memcpy(combined_search, boundary_overlap_buffer, boundary_overlap_buffer_fill);
                    // Copy start of current chunk
                    size_t chunk_copy_size = (combined_search_size - boundary_overlap_buffer_fill < ret) ? 
                                            (combined_search_size - boundary_overlap_buffer_fill) : ret;
                    memcpy(combined_search + boundary_overlap_buffer_fill, chunk_buffer, chunk_copy_size);
                    
                    // Search combined area for boundary
                    char *split_match = (char *)memmem(combined_search, combined_search_size, end_boundary, strlen(end_boundary));
                    if (split_match == NULL) {
                        split_match = (char *)memmem(combined_search, combined_search_size, start_boundary, strlen(start_boundary));
                    }
                    
                    if (split_match != NULL) {
                        // Verify it's a valid boundary
                        bool is_valid = false;
                        if (split_match == combined_search) {
                            is_valid = true;
                        } else if (split_match > combined_search && split_match[-1] == '\n') {
                            if (split_match > combined_search + 1 && split_match[-2] == '\r') {
                                is_valid = true;
                            } else if (split_match == combined_search + 1) {
                                is_valid = true;
                            }
                        }
                        
                        if (is_valid) {
                            // Calculate where boundary is in current chunk
                            size_t boundary_offset_in_combined = split_match - combined_search;
                            if (boundary_offset_in_combined >= boundary_overlap_buffer_fill) {
                                // Boundary is in current chunk
                                size_t offset_in_chunk = boundary_offset_in_combined - boundary_overlap_buffer_fill;
                                if (offset_in_chunk < ret) {
                                    boundary_pos = chunk_buffer + offset_in_chunk;
                                    ESP_LOGI(TAG, "Found split boundary at offset %d in chunk (split across chunks)", offset_in_chunk);
                                }
                            } else {
                                // Boundary is in previous chunk (already written) - error
                                ESP_LOGE(TAG, "Split boundary found in previous chunk data - may have written too much");
                            }
                        }
                    }
                    free(combined_search);
                }
            }
            
            if (boundary_pos != NULL) {
                // Found boundary - only write up to it (excluding the boundary itself and leading \r\n)
                to_write = boundary_pos - chunk_buffer;
                
                // Remove any trailing \r\n before the boundary (multipart boundaries are preceded by \r\n)
                // Go back to find the start of the line break sequence
                while (to_write > 0 && (chunk_buffer[to_write - 1] == '\r' || chunk_buffer[to_write - 1] == '\n')) {
                    to_write--;
                }
                
                // Make sure we have a complete \r\n sequence removed
                if (to_write >= 2 && chunk_buffer[to_write - 2] == '\r' && chunk_buffer[to_write - 1] == '\n') {
                    to_write -= 2;
                } else if (to_write >= 1 && chunk_buffer[to_write - 1] == '\n') {
                    to_write -= 1;
                }
                
                ESP_LOGI(TAG, "Boundary found at offset %d in chunk, writing %d bytes (chunk size: %d)", 
                         (int)(boundary_pos - chunk_buffer), to_write, ret);
                done = true;
            }
            
            // Safety check: if we're already past expected size, limit what we write
            // This prevents writing too much even if boundary detection fails
            if (expected_firmware_bytes > 0 && total_written + to_write > expected_firmware_bytes + 1000) {
                // We're writing way too much - limit to expected size
                size_t max_allowed = (expected_firmware_bytes + 1000 > total_written) ? 
                                     (expected_firmware_bytes + 1000 - total_written) : 0;
                if (to_write > max_allowed) {
                    ESP_LOGW(TAG, "Limiting write: would write %d bytes but only %d allowed (total_written=%d, expected=%d)", 
                             to_write, max_allowed, total_written, expected_firmware_bytes);
                    to_write = max_allowed;
                    if (to_write == 0) {
                        ESP_LOGW(TAG, "Stopping write - already exceeded expected size");
                        done = true;
                    }
                }
            }
            
            if (to_write > 0) {
                if (!ota_manager_write_streaming_chunk(ota_handle, (const uint8_t *)chunk_buffer, to_write)) {
                    ESP_LOGE(TAG, "Failed to write chunk at offset %d", total_written);
                    free(chunk_buffer);
                    free(boundary_overlap_buffer);
                    return send_json_error(req, "Failed to write firmware data", 500);
                }
                total_written += to_write;
                
                // Update overlap buffer with end of current chunk for next iteration
                // Keep last portion of chunk in case boundary is split across chunks
                size_t overlap_size = (ret < boundary_overlap_buffer_size) ? ret : boundary_overlap_buffer_size;
                if (boundary_pos != NULL) {
                    // Found boundary - only keep data before boundary
                    if (boundary_pos > chunk_buffer) {
                        size_t bytes_before_boundary = boundary_pos - chunk_buffer;
                        overlap_size = (bytes_before_boundary < boundary_overlap_buffer_size) ? bytes_before_boundary : boundary_overlap_buffer_size;
                    } else {
                        overlap_size = 0;
                    }
                }
                
                if (overlap_size > 0) {
                    // Copy end of chunk to overlap buffer
                    memcpy(boundary_overlap_buffer, chunk_buffer + ret - overlap_size, overlap_size);
                    boundary_overlap_buffer_fill = overlap_size;
                } else {
                    boundary_overlap_buffer_fill = 0;
                }
            }
            
            // If we found the end boundary or any boundary, we're done - break immediately
            if (done) {
                break;
            }
        }
        
        free(chunk_buffer);
        free(boundary_overlap_buffer);
        
        ESP_LOGI(TAG, "Streamed %d bytes to OTA partition", total_written);
        
        // Validate upload completeness if Content-Length was provided
        if (content_len > 0) {
            // Calculate actual multipart overhead from what we received
            // Actual overhead = Content-Length - firmware_bytes_written
            size_t actual_overhead = content_len - total_written;
            ESP_LOGI(TAG, "Multipart overhead: %d bytes (Content-Length: %d, Firmware: %d)", 
                     actual_overhead, content_len, total_written);
            
            // Validate that overhead is reasonable (should be between 100 bytes and 2KB)
            // This accounts for: boundary string, headers, boundary markers, line breaks
            if (actual_overhead < 100 || actual_overhead > 2048) {
                ESP_LOGE(TAG, "Invalid multipart overhead: %d bytes (expected 100-2048 bytes). Upload may be corrupted.", actual_overhead);
                esp_ota_abort(ota_handle);
                return send_json_error(req, "Upload validation failed: invalid multipart overhead", 400);
            }
            
            // Check if firmware is too small (might be incomplete)
            // Minimum expected: Content-Length minus maximum reasonable overhead (2KB)
            size_t min_expected = (content_len > 2048) ? (content_len - 2048) : content_len;
            size_t min_expected_95 = (min_expected * 95) / 100; // Allow 5% tolerance
            
            if (total_written < min_expected_95) {
                ESP_LOGE(TAG, "Upload incomplete: received %d bytes, expected at least %d bytes (Content-Length: %d)", 
                         total_written, min_expected_95, content_len);
                esp_ota_abort(ota_handle);
                return send_json_error(req, "Upload incomplete - connection may have been interrupted", 400);
            }
            
            // Success - overhead is reasonable and firmware size is acceptable
            ESP_LOGI(TAG, "Upload validation passed: firmware=%d bytes, overhead=%d bytes, Content-Length=%d bytes", 
                     total_written, actual_overhead, content_len);
        }
        
        // Finish OTA update (this will set boot partition and reboot)
        // Send HTTP response BEFORE finishing, as the device will reboot
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", "Firmware uploaded successfully. Finishing update and rebooting...");
        
        // Send response first
        esp_err_t resp_err = send_json_response(req, response, ESP_OK);
        
        // Small delay to ensure response is sent
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Now finish the update (this will reboot)
        if (!ota_manager_finish_streaming_update(ota_handle)) {
            ESP_LOGE(TAG, "Failed to finish streaming OTA update");
            // Response already sent, but update failed - device will not reboot
            return ESP_FAIL;
        }
        
        // This should never be reached as ota_manager_finish_streaming_update() reboots
        return resp_err;
    }
    
    // Handle URL-based update (existing JSON method)
    if (strstr(content_type, "application/json") == NULL) {
        ESP_LOGW(TAG, "Unsupported Content-Type for OTA update: %s", content_type);
        return send_json_error(req, "Unsupported Content-Type. Use multipart/form-data for file upload or application/json for URL", 400);
    }
    
    char content[256];
    int bytes_received = httpd_req_recv(req, content, sizeof(content) - 1);
    if (bytes_received <= 0) {
        ESP_LOGE(TAG, "Failed to read request body");
        return send_json_error(req, "Failed to read request body", 500);
    }
    content[bytes_received] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        ESP_LOGW(TAG, "Invalid JSON in request");
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *item = cJSON_GetObjectItem(json, "url");
    if (item == NULL || !cJSON_IsString(item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid URL", 400);
    }
    
    const char *url = cJSON_GetStringValue(item);
    if (url == NULL) {
        cJSON_Delete(json);
        return send_json_error(req, "Invalid URL", 400);
    }
    
    ESP_LOGI(TAG, "Starting OTA update from URL: %s", url);
    bool success = ota_manager_start_update(url);
    
    // Delete JSON after using the URL string (url pointer points into json memory)
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    if (success) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", "OTA update started");
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", "Failed to start OTA update");
    }
    
    return send_json_response(req, response, success ? ESP_OK : ESP_FAIL);
}

// GET /api/ota/status - Get OTA status
static esp_err_t api_ota_status_handler(httpd_req_t *req)
{
    ota_status_info_t status_info;
    if (!ota_manager_get_status(&status_info)) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    
    cJSON *json = cJSON_CreateObject();
    const char *status_str;
    switch (status_info.status) {
        case OTA_STATUS_IDLE:
            status_str = "idle";
            break;
        case OTA_STATUS_IN_PROGRESS:
            status_str = "in_progress";
            break;
        case OTA_STATUS_COMPLETE:
            status_str = "complete";
            break;
        case OTA_STATUS_ERROR:
            status_str = "error";
            break;
        default:
            status_str = "unknown";
            break;
    }
    
    cJSON_AddStringToObject(json, "status", status_str);
    cJSON_AddNumberToObject(json, "progress", status_info.progress);
    cJSON_AddStringToObject(json, "message", status_info.message);
    
    return send_json_response(req, json, ESP_OK);
}

// GET /api/assemblies/sizes - Get assembly sizes
static esp_err_t api_get_assemblies_sizes_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "input_assembly_size", sizeof(g_assembly_data064));
    cJSON_AddNumberToObject(json, "output_assembly_size", sizeof(g_assembly_data096));
    
    return send_json_response(req, json, ESP_OK);
}

// GET /api/status - Get unified assembly and sensor data for status pages
static esp_err_t api_get_status_handler(httpd_req_t *req)
{
    uint8_t input_assembly_copy[sizeof(g_assembly_data064)];
    uint8_t output_assembly_copy[sizeof(g_assembly_data096)];
    
    SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
    if (mutex == NULL) {
        return send_json_error(req, "Assembly mutex not available", 500);
    }
    
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return send_json_error(req, "Failed to acquire assembly mutex", 500);
    }
    
    memcpy(input_assembly_copy, g_assembly_data064, sizeof(g_assembly_data064));
    memcpy(output_assembly_copy, g_assembly_data096, sizeof(g_assembly_data096));
    
    xSemaphoreGive(mutex);
    
    cJSON *json = cJSON_CreateObject();
    if (json == NULL) {
        return send_json_error(req, "Failed to create JSON object", 500);
    }
    
    uint16_t distance = 0;
    uint8_t status = 0;
    uint16_t ambient = 0;
    uint16_t sig_per_spad = 0;
    uint16_t num_spads = 0;

    memcpy(&distance, &input_assembly_copy[VL53L1X_BYTE_START], sizeof(uint16_t));
    memcpy(&status, &input_assembly_copy[VL53L1X_BYTE_START + 2], sizeof(uint8_t));
    memcpy(&ambient, &input_assembly_copy[VL53L1X_BYTE_START + 3], sizeof(uint16_t));
    memcpy(&sig_per_spad, &input_assembly_copy[VL53L1X_BYTE_START + 5], sizeof(uint16_t));
    memcpy(&num_spads, &input_assembly_copy[VL53L1X_BYTE_START + 7], sizeof(uint16_t));

    cJSON *vl53l1x_data = cJSON_CreateObject();
    if (vl53l1x_data != NULL) {
        bool data_valid = true;
        bool initialized = true;
        
        cJSON_AddNumberToObject(vl53l1x_data, "distance_mm", distance);
        cJSON_AddNumberToObject(vl53l1x_data, "status", status);
        cJSON_AddNumberToObject(vl53l1x_data, "ambient", ambient);
        cJSON_AddNumberToObject(vl53l1x_data, "sig_per_spad", sig_per_spad);
        cJSON_AddNumberToObject(vl53l1x_data, "num_spads", num_spads);
        cJSON_AddBoolToObject(vl53l1x_data, "data_valid", data_valid);
        cJSON_AddBoolToObject(vl53l1x_data, "initialized", initialized);
        cJSON_AddItemToObject(json, "vl53l1x", vl53l1x_data);
    }

    int16_t roll_scaled = 0, pitch_scaled = 0, ground_angle_scaled = 0;
    memcpy(&roll_scaled, &input_assembly_copy[LSM6DS3_BYTE_START], sizeof(int16_t));
    memcpy(&pitch_scaled, &input_assembly_copy[LSM6DS3_BYTE_START + 2], sizeof(int16_t));
    memcpy(&ground_angle_scaled, &input_assembly_copy[LSM6DS3_BYTE_START + 4], sizeof(int16_t));

    cJSON *lsm6ds3_data = cJSON_CreateObject();
    if (lsm6ds3_data != NULL) {
        cJSON_AddNumberToObject(lsm6ds3_data, "roll", (float)roll_scaled / 100.0f);
        cJSON_AddNumberToObject(lsm6ds3_data, "pitch", (float)pitch_scaled / 100.0f);
        cJSON_AddNumberToObject(lsm6ds3_data, "ground_angle", (float)ground_angle_scaled / 100.0f);
        cJSON_AddBoolToObject(lsm6ds3_data, "initialized", true);
        
        if (!s_lsm6ds3_enabled_cached) {
            s_cached_lsm6ds3_enabled = system_lsm6ds3_enabled_load();
            s_lsm6ds3_enabled_cached = true;
        }
        cJSON_AddBoolToObject(lsm6ds3_data, "enabled", s_cached_lsm6ds3_enabled);
        
        cJSON_AddItemToObject(json, "lsm6ds3", lsm6ds3_data);
    }

    const uint8_t byte_offset = NAU7802_BYTE_START;
    int32_t weight_scaled = 0;
    memcpy(&weight_scaled, &input_assembly_copy[byte_offset], sizeof(int32_t));
    int32_t raw_reading = 0;
    memcpy(&raw_reading, &input_assembly_copy[byte_offset + 4], sizeof(int32_t));
    uint8_t unit_code = input_assembly_copy[byte_offset + 8];
    const char *unit_str = (unit_code == 0) ? "g" : (unit_code == 1) ? "lbs" : "kg";
    uint8_t status_byte = input_assembly_copy[byte_offset + 9];
    bool available = (status_byte & 0x01) != 0;
    bool connected = (status_byte & 0x02) != 0;
    bool initialized_nau = (status_byte & 0x04) != 0;
    float weight_actual = (float)weight_scaled / 100.0f;

    cJSON *nau7802_data = cJSON_CreateObject();
    if (nau7802_data != NULL) {
        cJSON_AddNumberToObject(nau7802_data, "weight", weight_actual);
        cJSON_AddStringToObject(nau7802_data, "unit", unit_str);
        cJSON_AddNumberToObject(nau7802_data, "raw_reading", raw_reading);
        cJSON_AddBoolToObject(nau7802_data, "available", available);
        cJSON_AddBoolToObject(nau7802_data, "connected", connected);
        cJSON_AddBoolToObject(nau7802_data, "initialized", initialized_nau);
        cJSON_AddItemToObject(json, "nau7802", nau7802_data);
    }

    cJSON *mcp230xx_data = cJSON_CreateObject();
    if (mcp230xx_data != NULL) {
        cJSON *output = cJSON_CreateArray();
        cJSON *feedback = cJSON_CreateArray();
        
        for (int i = 0; i < 16; i++) {
            cJSON_AddItemToArray(output, cJSON_CreateNumber(output_assembly_copy[MCP230XX_OUTPUT_BYTE_START + i]));
            cJSON_AddItemToArray(feedback, cJSON_CreateNumber(input_assembly_copy[MCP230XX_FEEDBACK_BYTE_START + i]));
        }
        
        cJSON_AddItemToObject(mcp230xx_data, "output", output);
        cJSON_AddItemToObject(mcp230xx_data, "feedback", feedback);
        cJSON_AddItemToObject(json, "mcp230xx", mcp230xx_data);
    }

    cJSON *gp8403_data = cJSON_CreateObject();
    if (gp8403_data != NULL) {
        cJSON *devices = cJSON_CreateArray();
        for (int i = 0; i < 4; i++) {
            uint16_t ch0 = 0, ch1 = 0;
            int offset = GP8403_DAC_BYTE_START + (i * 4);
            if (offset + 3 < sizeof(output_assembly_copy)) {
                memcpy(&ch0, &output_assembly_copy[offset], sizeof(uint16_t));
                memcpy(&ch1, &output_assembly_copy[offset + 2], sizeof(uint16_t));
            }
            cJSON *device = cJSON_CreateObject();
            cJSON_AddNumberToObject(device, "address", 0x58 + i);
            cJSON_AddNumberToObject(device, "channel_0", ch0);
            cJSON_AddNumberToObject(device, "channel_1", ch1);
            cJSON_AddItemToArray(devices, device);
        }
        cJSON_AddItemToObject(gp8403_data, "devices", devices);
        cJSON_AddItemToObject(json, "gp8403", gp8403_data);
    }
    
    return send_json_response(req, json, ESP_OK);
}


// GET /api/i2c/pullup - Get I2C pull-up enabled state
static esp_err_t api_get_i2c_pullup_handler(httpd_req_t *req)
{
    // Use cached value if available, otherwise load from NVS and cache it
    if (!s_i2c_pullup_enabled_cached) {
        s_cached_i2c_pullup_enabled = system_i2c_internal_pullup_load();
        s_i2c_pullup_enabled_cached = true;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", s_cached_i2c_pullup_enabled);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/i2c/pullup - Set I2C pull-up enabled state
static esp_err_t api_post_i2c_pullup_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *item = cJSON_GetObjectItem(json, "enabled");
    if (item == NULL || !cJSON_IsBool(item)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'enabled' field");
        return ESP_FAIL;
    }
    
    bool enabled = cJSON_IsTrue(item);
    cJSON_Delete(json);
    
    if (!system_i2c_internal_pullup_save(enabled)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save I2C pull-up setting");
        return ESP_FAIL;
    }
    
    // Update cache when state changes
    s_cached_i2c_pullup_enabled = enabled;
    s_i2c_pullup_enabled_cached = true;
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddBoolToObject(response, "enabled", enabled);
    cJSON_AddStringToObject(response, "message", "I2C pull-up setting saved. Restart required for changes to take effect.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/i2c/pullup/primary - Get primary I2C bus pull-up enabled state
static esp_err_t api_get_i2c_primary_pullup_handler(httpd_req_t *req)
{
    if (!s_i2c_primary_pullup_cached) {
        s_cached_i2c_primary_pullup = system_i2c_primary_pullup_load();
        s_i2c_primary_pullup_cached = true;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", s_cached_i2c_primary_pullup);
    cJSON_AddStringToObject(json, "bus", "primary");
    cJSON_AddNumberToObject(json, "sda_gpio", 7);
    cJSON_AddNumberToObject(json, "scl_gpio", 8);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/i2c/pullup/primary - Set primary I2C bus pull-up enabled state
static esp_err_t api_post_i2c_primary_pullup_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *item = cJSON_GetObjectItem(json, "enabled");
    if (item == NULL || !cJSON_IsBool(item)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'enabled' field");
        return ESP_FAIL;
    }
    
    bool enabled = cJSON_IsTrue(item);
    cJSON_Delete(json);
    
    if (!system_i2c_primary_pullup_save(enabled)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save primary I2C pull-up setting");
        return ESP_FAIL;
    }
    
    s_cached_i2c_primary_pullup = enabled;
    s_i2c_primary_pullup_cached = true;
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddBoolToObject(response, "enabled", enabled);
    cJSON_AddStringToObject(response, "message", "Primary I2C pull-up setting saved. Restart required for changes to take effect.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/i2c/pullup/secondary - Get secondary I2C bus pull-up enabled state
static esp_err_t api_get_i2c_secondary_pullup_handler(httpd_req_t *req)
{
    if (!s_i2c_secondary_pullup_cached) {
        s_cached_i2c_secondary_pullup = system_i2c_secondary_pullup_load();
        s_i2c_secondary_pullup_cached = true;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", s_cached_i2c_secondary_pullup);
    cJSON_AddStringToObject(json, "bus", "secondary");
    cJSON_AddNumberToObject(json, "sda_gpio", 33);
    cJSON_AddNumberToObject(json, "scl_gpio", 32);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/i2c/pullup/secondary - Set secondary I2C bus pull-up enabled state
static esp_err_t api_post_i2c_secondary_pullup_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    cJSON *item = cJSON_GetObjectItem(json, "enabled");
    if (item == NULL || !cJSON_IsBool(item)) {
        cJSON_Delete(json);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing or invalid 'enabled' field");
        return ESP_FAIL;
    }
    
    bool enabled = cJSON_IsTrue(item);
    cJSON_Delete(json);
    
    if (!system_i2c_secondary_pullup_save(enabled)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save secondary I2C pull-up setting");
        return ESP_FAIL;
    }
    
    s_cached_i2c_secondary_pullup = enabled;
    s_i2c_secondary_pullup_cached = true;
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddBoolToObject(response, "enabled", enabled);
    cJSON_AddStringToObject(response, "message", "Secondary I2C pull-up setting saved. Restart required for changes to take effect.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/logs - Get system logs
static esp_err_t api_get_logs_handler(httpd_req_t *req)
{
    if (!log_buffer_is_enabled()) {
        return send_json_error(req, "Log buffer not enabled", 503);
    }
    
    // Get log buffer size
    size_t log_size = log_buffer_get_size();
    
    // Allocate buffer for logs (limit to 32KB for API response)
    size_t buffer_size = (log_size < 32 * 1024) ? log_size + 1 : 32 * 1024;
    char *log_buffer = (char *)malloc(buffer_size);
    if (log_buffer == NULL) {
        return send_json_error(req, "Failed to allocate memory for logs", 500);
    }
    
    // Get logs
    size_t bytes_read = log_buffer_get(log_buffer, buffer_size);
    
    // Create JSON response
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "ok");
    cJSON_AddStringToObject(json, "logs", log_buffer);
    cJSON_AddNumberToObject(json, "size", bytes_read);
    cJSON_AddNumberToObject(json, "total_size", log_size);
    cJSON_AddBoolToObject(json, "truncated", bytes_read < log_size);
    
    free(log_buffer);
    
    return send_json_response(req, json, ESP_OK);
}

// Helper function to convert IP string to uint32_t (network byte order)
static uint32_t ip_string_to_uint32(const char *ip_str)
{
    if (ip_str == NULL || strlen(ip_str) == 0) {
        return 0;
    }
    struct in_addr addr;
    if (inet_aton(ip_str, &addr) == 0) {
        return 0;
    }
    return addr.s_addr;
}

// Helper function to convert uint32_t (network byte order) to IP string
static void ip_uint32_to_string(uint32_t ip, char *buf, size_t buf_size)
{
    struct in_addr addr;
    addr.s_addr = ip;
    const char *ip_str = inet_ntoa(addr);
    if (ip_str != NULL) {
        strncpy(buf, ip_str, buf_size - 1);
        buf[buf_size - 1] = '\0';
    } else {
        buf[0] = '\0';
    }
}

// GET /api/ipconfig - Get IP configuration
static esp_err_t api_get_ipconfig_handler(httpd_req_t *req)
{
    // Initialize mutex if needed (thread-safe lazy initialization)
    if (s_tcpip_mutex == NULL) {
        s_tcpip_mutex = xSemaphoreCreateMutex();
        if (s_tcpip_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create TCP/IP mutex");
            return send_json_error(req, "Internal error: mutex creation failed", 500);
        }
    }
    
    // Always read from OpENer's g_tcpip (single source of truth)
    // Protect with mutex to prevent race conditions with OpENer task
    if (xSemaphoreTake(s_tcpip_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for TCP/IP mutex");
        return send_json_error(req, "Timeout accessing IP configuration", 500);
    }
    
    bool use_dhcp = (g_tcpip.config_control & kTcpipCfgCtrlMethodMask) == kTcpipCfgCtrlDhcp;
    
    // Copy values to local variables while holding mutex
    uint32_t ip_address = g_tcpip.interface_configuration.ip_address;
    uint32_t network_mask = g_tcpip.interface_configuration.network_mask;
    uint32_t gateway = g_tcpip.interface_configuration.gateway;
    uint32_t name_server = g_tcpip.interface_configuration.name_server;
    uint32_t name_server_2 = g_tcpip.interface_configuration.name_server_2;
    
    xSemaphoreGive(s_tcpip_mutex);
    
    // Build JSON response outside of mutex (safer, no blocking)
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "use_dhcp", use_dhcp);
    
    char ip_str[16];
    ip_uint32_to_string(ip_address, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "ip_address", ip_str);
    
    ip_uint32_to_string(network_mask, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "netmask", ip_str);
    
    ip_uint32_to_string(gateway, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "gateway", ip_str);
    
    ip_uint32_to_string(name_server, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "dns1", ip_str);
    
    ip_uint32_to_string(name_server_2, ip_str, sizeof(ip_str));
    cJSON_AddStringToObject(json, "dns2", ip_str);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/ipconfig - Set IP configuration
static esp_err_t api_post_ipconfig_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    // Parse JSON first (before taking mutex)
    cJSON *item = cJSON_GetObjectItem(json, "use_dhcp");
    bool use_dhcp_requested = false;
    bool use_dhcp_set = false;
    if (item != NULL && cJSON_IsBool(item)) {
        use_dhcp_requested = cJSON_IsTrue(item);
        use_dhcp_set = true;
    }
    
    // Parse IP configuration values
    uint32_t ip_address_new = 0;
    uint32_t network_mask_new = 0;
    uint32_t gateway_new = 0;
    uint32_t name_server_new = 0;
    uint32_t name_server_2_new = 0;
    bool ip_address_set = false;
    bool network_mask_set = false;
    bool gateway_set = false;
    bool name_server_set = false;
    bool name_server_2_set = false;
    
    // Read current config_control to determine if we should parse IP settings
    bool is_static_ip = false;
    if (s_tcpip_mutex != NULL && xSemaphoreTake(s_tcpip_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        is_static_ip = ((g_tcpip.config_control & kTcpipCfgCtrlMethodMask) == kTcpipCfgCtrlStaticIp);
        xSemaphoreGive(s_tcpip_mutex);
    }
    
    if (is_static_ip || !use_dhcp_requested) {
        item = cJSON_GetObjectItem(json, "ip_address");
        if (item != NULL && cJSON_IsString(item)) {
            ip_address_new = ip_string_to_uint32(cJSON_GetStringValue(item));
            ip_address_set = true;
        }
        
        item = cJSON_GetObjectItem(json, "netmask");
        if (item != NULL && cJSON_IsString(item)) {
            network_mask_new = ip_string_to_uint32(cJSON_GetStringValue(item));
            network_mask_set = true;
        }
        
        item = cJSON_GetObjectItem(json, "gateway");
        if (item != NULL && cJSON_IsString(item)) {
            gateway_new = ip_string_to_uint32(cJSON_GetStringValue(item));
            gateway_set = true;
        }
    }
    
    item = cJSON_GetObjectItem(json, "dns1");
    if (item != NULL && cJSON_IsString(item)) {
        name_server_new = ip_string_to_uint32(cJSON_GetStringValue(item));
        name_server_set = true;
    }
    
    item = cJSON_GetObjectItem(json, "dns2");
    if (item != NULL && cJSON_IsString(item)) {
        name_server_2_new = ip_string_to_uint32(cJSON_GetStringValue(item));
        name_server_2_set = true;
    }
    
    cJSON_Delete(json);
    
    // Initialize mutex if needed
    if (s_tcpip_mutex == NULL) {
        s_tcpip_mutex = xSemaphoreCreateMutex();
        if (s_tcpip_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create TCP/IP mutex");
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Internal error: mutex creation failed");
            return ESP_FAIL;
        }
    }
    
    // Update g_tcpip with mutex protection
    if (xSemaphoreTake(s_tcpip_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGW(TAG, "Timeout waiting for TCP/IP mutex");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Timeout accessing IP configuration");
        return ESP_FAIL;
    }
    
    // Update configuration control
    if (use_dhcp_set) {
        if (use_dhcp_requested) {
            g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
            g_tcpip.config_control |= kTcpipCfgCtrlDhcp;
            g_tcpip.interface_configuration.ip_address = 0;
            g_tcpip.interface_configuration.network_mask = 0;
            g_tcpip.interface_configuration.gateway = 0;
        } else {
            g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
            g_tcpip.config_control |= kTcpipCfgCtrlStaticIp;
        }
    }
    
    // Update IP settings if provided
    if (ip_address_set) {
        g_tcpip.interface_configuration.ip_address = ip_address_new;
    }
    if (network_mask_set) {
        g_tcpip.interface_configuration.network_mask = network_mask_new;
    }
    if (gateway_set) {
        g_tcpip.interface_configuration.gateway = gateway_new;
    }
    if (name_server_set) {
        g_tcpip.interface_configuration.name_server = name_server_new;
    }
    if (name_server_2_set) {
        g_tcpip.interface_configuration.name_server_2 = name_server_2_new;
    }
    
    // Save to NVS while holding mutex
    EipStatus nvs_status = NvTcpipStore(&g_tcpip);
    xSemaphoreGive(s_tcpip_mutex);
    
    if (nvs_status != kEipStatusOk) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save IP configuration");
        return ESP_FAIL;
    }
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "IP configuration saved successfully. Reboot required to apply changes.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/nau7802 - Get NAU7802 scale reading, status, and configuration
static esp_err_t api_get_nau7802_handler(httpd_req_t *req)
{
    // Use cached value if available, otherwise load from NVS and cache it
    if (!s_nau7802_enabled_cached) {
        s_cached_nau7802_enabled = system_nau7802_enabled_load();
        s_nau7802_enabled_cached = true;
    }
    if (!s_nau7802_unit_cached) {
        s_cached_nau7802_unit = system_nau7802_unit_load();
        s_nau7802_unit_cached = true;
    }
    if (!s_nau7802_gain_cached) {
        s_cached_nau7802_gain = system_nau7802_gain_load();
        s_nau7802_gain_cached = true;
    }
    if (!s_nau7802_sample_rate_cached) {
        s_cached_nau7802_sample_rate = system_nau7802_sample_rate_load();
        s_nau7802_sample_rate_cached = true;
    }
    if (!s_nau7802_channel_cached) {
        s_cached_nau7802_channel = system_nau7802_channel_load();
        s_nau7802_channel_cached = true;
    }
    if (!s_nau7802_ldo_cached) {
        s_cached_nau7802_ldo = system_nau7802_ldo_load();
        s_nau7802_ldo_cached = true;
    }
    if (!s_nau7802_average_cached) {
        s_cached_nau7802_average = system_nau7802_average_load();
        s_nau7802_average_cached = true;
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", s_cached_nau7802_enabled);
    cJSON_AddNumberToObject(json, "unit", s_cached_nau7802_unit);
    cJSON_AddNumberToObject(json, "gain", s_cached_nau7802_gain);
    cJSON_AddNumberToObject(json, "sample_rate", s_cached_nau7802_sample_rate);
    cJSON_AddNumberToObject(json, "channel", s_cached_nau7802_channel);
    cJSON_AddNumberToObject(json, "ldo_value", s_cached_nau7802_ldo);
    cJSON_AddNumberToObject(json, "average", s_cached_nau7802_average);
    cJSON_AddBoolToObject(json, "initialized", fusion_core_is_nau7802_initialized());
    
    // Add labels for better readability
    const char *gain_labels[] = {"x1", "x2", "x4", "x8", "x16", "x32", "x64", "x128"};
    const char *sps_labels[] = {"10", "20", "40", "80", "", "", "", "320"};
    const char *unit_labels[] = {"g", "lbs", "kg"};
    const float ldo_voltages[] = {4.5f, 4.2f, 3.9f, 3.6f, 3.3f, 3.0f, 2.7f, 2.4f};
    
    if (s_cached_nau7802_gain < 8) {
        cJSON_AddStringToObject(json, "gain_label", gain_labels[s_cached_nau7802_gain]);
    }
    if (s_cached_nau7802_sample_rate < 8 && sps_labels[s_cached_nau7802_sample_rate][0] != '\0') {
        cJSON_AddStringToObject(json, "sample_rate_label", sps_labels[s_cached_nau7802_sample_rate]);
    }
    if (s_cached_nau7802_unit < 3) {
        cJSON_AddStringToObject(json, "unit_label", unit_labels[s_cached_nau7802_unit]);
    }
    if (s_cached_nau7802_channel < 2) {
        cJSON_AddStringToObject(json, "channel_label", s_cached_nau7802_channel == 0 ? "Channel 1" : "Channel 2");
    }
    if (s_cached_nau7802_ldo < 8) {
        cJSON_AddNumberToObject(json, "ldo_voltage", ldo_voltages[s_cached_nau7802_ldo]);
    }
    
    // Configuration endpoint - do NOT return live data here
    // Live data (weight, raw_reading, connected, available) should come from /api/status only
    // This prevents I2C transactions and ensures all live data comes from assembly buffers
    
    // Load calibration values from NVS (cached, no I2C transactions)
    // Only load once, and silently handle missing NVS namespace (normal if no calibration saved yet)
    if (!s_nau7802_cal_cached) {
        nau7802_calibration_data_t cal_data;
        esp_err_t ret = nau7802_calibration_load(&cal_data);
        if (ret == ESP_OK) {
            s_cached_nau7802_cal_factor = cal_data.calibration_factor;
            s_cached_nau7802_zero_offset = cal_data.zero_offset;
            s_cached_nau7802_ch1_offset = cal_data.channel1_offset;
        }
        // ESP_ERR_NVS_NOT_FOUND is normal if calibration hasn't been saved yet - don't log
        s_nau7802_cal_cached = true;
    }
    
    // Add calibration values from NVS cache (no device reads)
    cJSON_AddNumberToObject(json, "calibration_factor", s_cached_nau7802_cal_factor);
    cJSON_AddNumberToObject(json, "zero_offset", s_cached_nau7802_zero_offset);
    
    // Add channel 1 offset from cache
    cJSON *ch1 = cJSON_CreateObject();
    cJSON_AddNumberToObject(ch1, "offset", s_cached_nau7802_ch1_offset);
    cJSON_AddItemToObject(json, "channel1", ch1);
    
    // Channel 2 and register status are not critical - omit to avoid I2C transactions
    // These can be read on-demand if needed via a separate endpoint
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/nau7802 - Configure NAU7802 (enable/disable, byte offset)
static esp_err_t api_post_nau7802_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    bool config_changed = false;
    
    // Handle enabled state
    cJSON *item = cJSON_GetObjectItem(json, "enabled");
    if (item != NULL && cJSON_IsBool(item)) {
        bool enabled = cJSON_IsTrue(item);
        if (system_nau7802_enabled_save(enabled)) {
            s_cached_nau7802_enabled = enabled;
            s_nau7802_enabled_cached = true;
            config_changed = true;
        }
    }
    
    // Handle unit selection
    item = cJSON_GetObjectItem(json, "unit");
    if (item != NULL && cJSON_IsNumber(item)) {
        int unit_int = (int)cJSON_GetNumberValue(item);
        if (unit_int >= 0 && unit_int <= 2) {
            uint8_t unit = (uint8_t)unit_int;
            if (system_nau7802_unit_save(unit)) {
                s_cached_nau7802_unit = unit;
                s_nau7802_unit_cached = true;
                config_changed = true;
            }
        }
    }
    
    // Handle gain setting
    item = cJSON_GetObjectItem(json, "gain");
    if (item != NULL && cJSON_IsNumber(item)) {
        int gain_int = (int)cJSON_GetNumberValue(item);
        if (gain_int >= 0 && gain_int <= 7) {
            uint8_t gain = (uint8_t)gain_int;
            if (system_nau7802_gain_save(gain)) {
                s_cached_nau7802_gain = gain;
                s_nau7802_gain_cached = true;
                config_changed = true;
            }
        }
    }
    
    // Handle sample rate
    item = cJSON_GetObjectItem(json, "sample_rate");
    if (item != NULL && cJSON_IsNumber(item)) {
        int sample_rate_int = (int)cJSON_GetNumberValue(item);
        // Valid values: 0, 1, 2, 3, 7
        if (sample_rate_int == 0 || sample_rate_int == 1 || sample_rate_int == 2 || 
            sample_rate_int == 3 || sample_rate_int == 7) {
            uint8_t sample_rate = (uint8_t)sample_rate_int;
            if (system_nau7802_sample_rate_save(sample_rate)) {
                s_cached_nau7802_sample_rate = sample_rate;
                s_nau7802_sample_rate_cached = true;
                config_changed = true;
            }
        }
    }
    
    // Handle channel selection
    item = cJSON_GetObjectItem(json, "channel");
    if (item != NULL && cJSON_IsNumber(item)) {
        int channel_int = (int)cJSON_GetNumberValue(item);
        if (channel_int >= 0 && channel_int <= 1) {
            uint8_t channel = (uint8_t)channel_int;
            if (system_nau7802_channel_save(channel)) {
                s_cached_nau7802_channel = channel;
                s_nau7802_channel_cached = true;
                config_changed = true;
            }
        }
    }
    
    // Handle LDO voltage
    item = cJSON_GetObjectItem(json, "ldo_value");
    if (item != NULL && cJSON_IsNumber(item)) {
        int ldo_int = (int)cJSON_GetNumberValue(item);
        if (ldo_int >= 0 && ldo_int <= 7) {
            uint8_t ldo = (uint8_t)ldo_int;
            if (system_nau7802_ldo_save(ldo)) {
                s_cached_nau7802_ldo = ldo;
                s_nau7802_ldo_cached = true;
                config_changed = true;
            }
        }
    }
    
    // Handle averaging setting (for regular readings)
    item = cJSON_GetObjectItem(json, "average");
    if (item != NULL && cJSON_IsNumber(item)) {
        int avg_int = (int)cJSON_GetNumberValue(item);
        if (avg_int >= 1 && avg_int <= 50) {
            uint8_t average = (uint8_t)avg_int;
            if (system_nau7802_average_save(average)) {
                s_cached_nau7802_average = average;
                s_nau7802_average_cached = true;
                config_changed = true;
            }
        }
    }
    
    cJSON_Delete(json);
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", config_changed ? "Configuration saved. Reboot required to apply gain, sample rate, channel, or LDO changes." : "No changes");
    
    return send_json_response(req, response, ESP_OK);
}

// POST /api/nau7802/calibrate - Calibrate the scale
static esp_err_t api_post_nau7802_calibrate_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }
    
    void *nau7802_handle = fusion_core_get_nau7802_handle();
    if (nau7802_handle == NULL || !fusion_core_is_nau7802_initialized()) {
        cJSON_Delete(json);
        return send_json_error(req, "NAU7802 not initialized", 500);
    }
    
    nau7802_t *nau7802 = (nau7802_t *)nau7802_handle;
    
    cJSON *item = cJSON_GetObjectItem(json, "action");
    if (item == NULL || !cJSON_IsString(item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'action' field (must be 'tare' or 'calibrate')", 400);
    }
    
    const char *action = cJSON_GetStringValue(item);
    cJSON *response = cJSON_CreateObject();
    
    if (strcmp(action, "tare") == 0) {
        // Tare (zero offset) calibration
        // Use default of 10 samples for averaging during calibration
        SemaphoreHandle_t nau7802_mutex = fusion_core_get_nau7802_mutex();
        esp_err_t err = ESP_FAIL;
        
        if (nau7802_mutex != NULL && xSemaphoreTake(nau7802_mutex, portMAX_DELAY) == pdTRUE) {
            err = nau7802_calculate_zero_offset(nau7802, 10, 5000);
            if (err == ESP_OK) {
                float zero_offset = nau7802_get_zero_offset(nau7802);
                xSemaphoreGive(nau7802_mutex);
                system_nau7802_zero_offset_save(zero_offset);
                cJSON_AddStringToObject(response, "status", "ok");
                cJSON_AddStringToObject(response, "message", "Tare calibration completed");
                cJSON_AddNumberToObject(response, "zero_offset", zero_offset);
            } else {
                xSemaphoreGive(nau7802_mutex);
                ESP_LOGE(TAG, "Tare calibration failed: %s", esp_err_to_name(err));
                cJSON_AddStringToObject(response, "status", "error");
                cJSON_AddStringToObject(response, "message", "Tare calibration failed");
            }
        } else {
            ESP_LOGE(TAG, "Failed to acquire NAU7802 mutex for tare calibration");
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "message", "Failed to acquire device lock");
        }
    } else if (strcmp(action, "calibrate") == 0) {
        // Calibration with known weight
        item = cJSON_GetObjectItem(json, "known_weight");
        if (item == NULL || !cJSON_IsNumber(item)) {
            cJSON_Delete(json);
            cJSON_Delete(response);
            return send_json_error(req, "Missing or invalid 'known_weight' field", 400);
        }
        
        float known_weight_input = (float)cJSON_GetNumberValue(item);
        if (known_weight_input <= 0.0f) {
            cJSON_Delete(json);
            cJSON_Delete(response);
            return send_json_error(req, "Known weight must be greater than 0", 400);
        }
        
        // Get unit selection and convert to grams (NAU7802 calibration uses grams internally)
        // Use cached value if available, otherwise load from NVS
        if (!s_nau7802_unit_cached) {
            s_cached_nau7802_unit = system_nau7802_unit_load();
            s_nau7802_unit_cached = true;
        }
        uint8_t unit = s_cached_nau7802_unit;
        float known_weight_grams = known_weight_input;
        if (unit == 1) {
            // Convert lbs to grams: 1 lb = 453.592 grams
            known_weight_grams = known_weight_input * 453.592f;
        } else if (unit == 2) {
            // Convert kg to grams: 1 kg = 1000 grams
            known_weight_grams = known_weight_input * 1000.0f;
        }
        // unit == 0 means grams, no conversion needed
        
        // Use default of 10 samples for averaging during calibration
        SemaphoreHandle_t nau7802_mutex = fusion_core_get_nau7802_mutex();
        esp_err_t err = ESP_FAIL;
        
        if (nau7802_mutex != NULL && xSemaphoreTake(nau7802_mutex, portMAX_DELAY) == pdTRUE) {
            err = nau7802_calculate_calibration_factor(nau7802, known_weight_grams, 10, 5000);
            if (err == ESP_OK) {
                float cal_factor = nau7802_get_calibration_factor(nau7802);
                float zero_offset = nau7802_get_zero_offset(nau7802);
                xSemaphoreGive(nau7802_mutex);
                system_nau7802_calibration_factor_save(cal_factor);
                system_nau7802_zero_offset_save(zero_offset);
                cJSON_AddStringToObject(response, "status", "ok");
                cJSON_AddStringToObject(response, "message", "Calibration completed");
                cJSON_AddNumberToObject(response, "calibration_factor", cal_factor);
                cJSON_AddNumberToObject(response, "zero_offset", zero_offset);
            } else {
                xSemaphoreGive(nau7802_mutex);
                ESP_LOGE(TAG, "Known-weight calibration failed: %s", esp_err_to_name(err));
                cJSON_AddStringToObject(response, "status", "error");
                cJSON_AddStringToObject(response, "message", "Calibration failed");
            }
        } else {
            ESP_LOGE(TAG, "Failed to acquire NAU7802 mutex for calibration");
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "message", "Failed to acquire device lock");
        }
    } else if (strcmp(action, "afe") == 0) {
        // AFE (Analog Front End) calibration
        ESP_LOGI(TAG, "Performing AFE calibration");
        SemaphoreHandle_t nau7802_mutex = fusion_core_get_nau7802_mutex();
        esp_err_t err = ESP_FAIL;
        
        if (nau7802_mutex != NULL && xSemaphoreTake(nau7802_mutex, portMAX_DELAY) == pdTRUE) {
            err = nau7802_calibrate_af(nau7802);
            xSemaphoreGive(nau7802_mutex);
            
            if (err == ESP_OK) {
                cJSON_AddStringToObject(response, "status", "ok");
                cJSON_AddStringToObject(response, "message", "AFE calibration completed successfully");
            } else {
                ESP_LOGE(TAG, "AFE calibration failed: %s", esp_err_to_name(err));
                cJSON_AddStringToObject(response, "status", "error");
                cJSON_AddStringToObject(response, "message", "AFE calibration failed");
            }
        } else {
            ESP_LOGE(TAG, "Failed to acquire NAU7802 mutex for AFE calibration");
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "message", "Failed to acquire device lock");
        }
    } else {
        cJSON_Delete(json);
        cJSON_Delete(response);
        return send_json_error(req, "Invalid action (must be 'tare', 'calibrate', or 'afe')", 400);
    }
    
    cJSON_Delete(json);
    return send_json_response(req, response, ESP_OK);
}

// GET /api/vl53l1x - Get VL53L1X status and readings
static esp_err_t api_get_vl53l1x_handler(httpd_req_t *req)
{
    s_cached_vl53l1x_enabled = system_vl53l1x_enabled_load();
    s_vl53l1x_enabled_cached = true;
    bool enabled = s_cached_vl53l1x_enabled;
    bool initialized = vl53l1x_manager_is_initialized();
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", enabled);
    cJSON_AddBoolToObject(json, "initialized", initialized);
    
    SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
    bool data_valid = false;
    if (mutex != NULL && xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint16_t distance = 0;
        uint8_t status = 0;
        uint16_t ambient = 0;
        uint16_t sig_per_spad = 0;
        uint16_t num_spads = 0;
        
        memcpy(&distance, &g_assembly_data064[VL53L1X_BYTE_START], sizeof(uint16_t));
        memcpy(&status, &g_assembly_data064[VL53L1X_BYTE_START + 2], sizeof(uint8_t));
        memcpy(&ambient, &g_assembly_data064[VL53L1X_BYTE_START + 3], sizeof(uint16_t));
        memcpy(&sig_per_spad, &g_assembly_data064[VL53L1X_BYTE_START + 5], sizeof(uint16_t));
        memcpy(&num_spads, &g_assembly_data064[VL53L1X_BYTE_START + 7], sizeof(uint16_t));
        
        if (initialized) {
            cJSON_AddNumberToObject(json, "distance_mm", distance);
            cJSON_AddNumberToObject(json, "status", status);
            cJSON_AddNumberToObject(json, "ambient", ambient);
            cJSON_AddNumberToObject(json, "sig_per_spad", sig_per_spad);
            cJSON_AddNumberToObject(json, "num_spads", num_spads);
            data_valid = true;
        }
        
        xSemaphoreGive(mutex);
    }
    
    cJSON_AddBoolToObject(json, "data_valid", data_valid);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/vl53l1x - Set VL53L1X enabled state
static esp_err_t api_post_vl53l1x_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
    if (enabled_item == NULL || !cJSON_IsBool(enabled_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'enabled' field", 400);
    }
    
    bool enabled = cJSON_IsTrue(enabled_item);
    system_vl53l1x_enabled_save(enabled);
    s_cached_vl53l1x_enabled = enabled;
    s_vl53l1x_enabled_cached = true;
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddBoolToObject(response, "enabled", enabled);
    cJSON_AddStringToObject(response, "message", "VL53L1X configuration saved. Reboot required.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/vl53l1x/config - Get VL53L1X configuration
static esp_err_t api_get_vl53l1x_config_handler(httpd_req_t *req)
{
    system_vl53l1x_config_t config;
    if (!system_vl53l1x_config_load(&config)) {
        system_vl53l1x_config_get_defaults(&config);
    }
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "distance_mode", config.distance_mode);
    cJSON_AddNumberToObject(json, "timing_budget_ms", config.timing_budget_ms);
    cJSON_AddNumberToObject(json, "inter_measurement_ms", config.inter_measurement_ms);
    cJSON_AddNumberToObject(json, "roi_x_size", config.roi_x_size);
    cJSON_AddNumberToObject(json, "roi_y_size", config.roi_y_size);
    cJSON_AddNumberToObject(json, "roi_center_spad", config.roi_center_spad);
    cJSON_AddNumberToObject(json, "offset_mm", config.offset_mm);
    cJSON_AddNumberToObject(json, "xtalk_cps", config.xtalk_cps);
    cJSON_AddNumberToObject(json, "signal_threshold_kcps", config.signal_threshold_kcps);
    cJSON_AddNumberToObject(json, "sigma_threshold_mm", config.sigma_threshold_mm);
    cJSON_AddNumberToObject(json, "threshold_low_mm", config.threshold_low_mm);
    cJSON_AddNumberToObject(json, "threshold_high_mm", config.threshold_high_mm);
    cJSON_AddNumberToObject(json, "threshold_window", config.threshold_window);
    cJSON_AddNumberToObject(json, "interrupt_polarity", config.interrupt_polarity);
    cJSON_AddNumberToObject(json, "i2c_address", config.i2c_address);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/vl53l1x/config - Set VL53L1X configuration
static esp_err_t api_post_vl53l1x_config_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    system_vl53l1x_config_t config;
    esp_err_t err = vl53l1x_manager_get_config(&config);
    if (err != ESP_OK) {
        system_vl53l1x_config_get_defaults(&config);
    }
    
    cJSON *item;
    if ((item = cJSON_GetObjectItem(json, "distance_mode")) != NULL && cJSON_IsNumber(item)) {
        config.distance_mode = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "timing_budget_ms")) != NULL && cJSON_IsNumber(item)) {
        config.timing_budget_ms = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "inter_measurement_ms")) != NULL && cJSON_IsNumber(item)) {
        config.inter_measurement_ms = (uint32_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "roi_x_size")) != NULL && cJSON_IsNumber(item)) {
        config.roi_x_size = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "roi_y_size")) != NULL && cJSON_IsNumber(item)) {
        config.roi_y_size = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "roi_center_spad")) != NULL && cJSON_IsNumber(item)) {
        config.roi_center_spad = (uint8_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "offset_mm")) != NULL && cJSON_IsNumber(item)) {
        config.offset_mm = (int16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "xtalk_cps")) != NULL && cJSON_IsNumber(item)) {
        config.xtalk_cps = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "signal_threshold_kcps")) != NULL && cJSON_IsNumber(item)) {
        config.signal_threshold_kcps = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "sigma_threshold_mm")) != NULL && cJSON_IsNumber(item)) {
        config.sigma_threshold_mm = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "threshold_low_mm")) != NULL && cJSON_IsNumber(item)) {
        config.threshold_low_mm = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "threshold_high_mm")) != NULL && cJSON_IsNumber(item)) {
        config.threshold_high_mm = (uint16_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "threshold_window")) != NULL && cJSON_IsNumber(item)) {
        config.threshold_window = (uint8_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "interrupt_polarity")) != NULL && cJSON_IsNumber(item)) {
        config.interrupt_polarity = (uint8_t)cJSON_GetNumberValue(item);
    }
    if ((item = cJSON_GetObjectItem(json, "i2c_address")) != NULL && cJSON_IsNumber(item)) {
        config.i2c_address = (uint8_t)cJSON_GetNumberValue(item);
    }
    
    cJSON_Delete(json);
    
    err = vl53l1x_manager_set_config(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set VL53L1X configuration: %s", esp_err_to_name(err));
        return send_json_error(req, "Failed to set configuration", 500);
    }
    
    ESP_LOGI(TAG, "VL53L1X configuration saved successfully via API");
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "VL53L1X configuration saved successfully");
    
    return send_json_response(req, response, ESP_OK);
}

// POST /api/vl53l1x/calibrate/offset - Calibrate offset
static esp_err_t api_post_vl53l1x_calibrate_offset_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *target_item = cJSON_GetObjectItem(json, "target_distance_mm");
    if (target_item == NULL || !cJSON_IsNumber(target_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'target_distance_mm' field", 400);
    }
    
    int16_t target_distance = (int16_t)cJSON_GetNumberValue(target_item);
    cJSON_Delete(json);
    
    esp_err_t err = vl53l1x_manager_calibrate_offset(target_distance);
    if (err != ESP_OK) {
        return send_json_error(req, "Failed to calibrate offset", 500);
    }
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "Offset calibration complete");
    
    return send_json_response(req, response, ESP_OK);
}

// POST /api/vl53l1x/calibrate/xtalk - Calibrate xtalk
static esp_err_t api_post_vl53l1x_calibrate_xtalk_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *target_item = cJSON_GetObjectItem(json, "target_distance_mm");
    if (target_item == NULL || !cJSON_IsNumber(target_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'target_distance_mm' field", 400);
    }
    
    uint16_t target_distance = (uint16_t)cJSON_GetNumberValue(target_item);
    cJSON_Delete(json);
    
    esp_err_t err = vl53l1x_manager_calibrate_xtalk(target_distance);
    if (err != ESP_OK) {
        return send_json_error(req, "Failed to calibrate xtalk", 500);
    }
    
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "Xtalk calibration complete");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/lsm6ds3 - Get LSM6DS3 status and readings
static esp_err_t api_get_lsm6ds3_handler(httpd_req_t *req)
{
    if (!s_lsm6ds3_enabled_cached) {
        s_cached_lsm6ds3_enabled = system_lsm6ds3_enabled_load();
        s_lsm6ds3_enabled_cached = true;
    }
    bool enabled = s_cached_lsm6ds3_enabled;
    bool initialized = lsm6ds3_manager_is_initialized();
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", enabled);
    cJSON_AddBoolToObject(json, "initialized", initialized);
    
    SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
    if (mutex != NULL && xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Read scaled angles from assembly (scaled by 100, like NAU7802 weight)
        int16_t roll_scaled = 0, pitch_scaled = 0, ground_angle_scaled = 0;
        
        memcpy(&roll_scaled, &g_assembly_data064[LSM6DS3_BYTE_START], sizeof(int16_t));
        memcpy(&pitch_scaled, &g_assembly_data064[LSM6DS3_BYTE_START + 2], sizeof(int16_t));
        memcpy(&ground_angle_scaled, &g_assembly_data064[LSM6DS3_BYTE_START + 4], sizeof(int16_t));
        
        // Unscale: divide by 100 to get actual angle in degrees
        float roll = (float)roll_scaled / 100.0f;
        float pitch = (float)pitch_scaled / 100.0f;
        float ground_angle = (float)ground_angle_scaled / 100.0f;
        
        cJSON_AddNumberToObject(json, "roll", roll);
        cJSON_AddNumberToObject(json, "pitch", pitch);
        cJSON_AddNumberToObject(json, "ground_angle", ground_angle);
        
        xSemaphoreGive(mutex);
    }
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/lsm6ds3 - Set LSM6DS3 enabled state
static esp_err_t api_post_lsm6ds3_handler(httpd_req_t *req)
{
    char content[128];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
    if (enabled_item == NULL || !cJSON_IsBool(enabled_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'enabled' field", 400);
    }
    
    bool enabled = cJSON_IsTrue(enabled_item);
    system_lsm6ds3_enabled_save(enabled);
    s_cached_lsm6ds3_enabled = enabled;
    s_lsm6ds3_enabled_cached = true;
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddBoolToObject(response, "enabled", enabled);
    cJSON_AddStringToObject(response, "message", "LSM6DS3 configuration saved. Reboot required.");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/lsm6ds3/calibrate - Get LSM6DS3 calibration status and offsets
static esp_err_t api_get_lsm6ds3_calibrate_handler(httpd_req_t *req)
{
    float roll_zero = 0, pitch_zero = 0, yaw_zero = 0;
    bool roll_set = false, pitch_set = false, yaw_set = false;
    
    lsm6ds3_manager_get_angle_zero(&roll_zero, &pitch_zero, &yaw_zero, &roll_set, &pitch_set, &yaw_set);
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "roll_zero", roll_zero);
    cJSON_AddNumberToObject(json, "pitch_zero", pitch_zero);
    cJSON_AddNumberToObject(json, "yaw_zero", yaw_zero);
    cJSON_AddBoolToObject(json, "roll_zero_set", roll_set);
    cJSON_AddBoolToObject(json, "pitch_zero_set", pitch_set);
    cJSON_AddBoolToObject(json, "yaw_zero_set", yaw_set);
    
    return send_json_response(req, json, ESP_OK);
}

// GET /api/lsm6ds3/nvs-calibration - Get LSM6DS3 calibration values from NVS (read-only)
static esp_err_t api_get_lsm6ds3_nvs_calibration_handler(httpd_req_t *req)
{
    float accel_offset_mg[3] = {0};
    float gyro_offset_mdps[3] = {0};
    bool accel_calibrated = false;
    bool gyro_calibrated = false;
    
    esp_err_t err = lsm6ds3_manager_get_nvs_calibration(accel_offset_mg, gyro_offset_mdps, &accel_calibrated, &gyro_calibrated);
    
    cJSON *json = cJSON_CreateObject();
    
    if (err == ESP_OK) {
        cJSON_AddBoolToObject(json, "found", true);
        cJSON_AddBoolToObject(json, "accel_calibrated", accel_calibrated);
        cJSON_AddBoolToObject(json, "gyro_calibrated", gyro_calibrated);
        
        cJSON *accel_offsets_json = cJSON_CreateArray();
        cJSON_AddItemToArray(accel_offsets_json, cJSON_CreateNumber(accel_offset_mg[0]));
        cJSON_AddItemToArray(accel_offsets_json, cJSON_CreateNumber(accel_offset_mg[1]));
        cJSON_AddItemToArray(accel_offsets_json, cJSON_CreateNumber(accel_offset_mg[2]));
        cJSON_AddItemToObject(json, "accel_offsets_mg", accel_offsets_json);
        
        cJSON *gyro_offsets_json = cJSON_CreateArray();
        cJSON_AddItemToArray(gyro_offsets_json, cJSON_CreateNumber(gyro_offset_mdps[0]));
        cJSON_AddItemToArray(gyro_offsets_json, cJSON_CreateNumber(gyro_offset_mdps[1]));
        cJSON_AddItemToArray(gyro_offsets_json, cJSON_CreateNumber(gyro_offset_mdps[2]));
        cJSON_AddItemToObject(json, "gyro_offsets_mdps", gyro_offsets_json);
    } else {
        cJSON_AddBoolToObject(json, "found", false);
    }
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/lsm6ds3/calibrate - Trigger LSM6DS3 calibration
static esp_err_t api_post_lsm6ds3_calibrate_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *type_item = cJSON_GetObjectItem(json, "type");
    if (type_item == NULL || !cJSON_IsString(type_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'type' field (must be 'accel' or 'gyro')", 400);
    }
    
    const char *type = cJSON_GetStringValue(type_item);
    uint32_t samples = 100;
    uint32_t sample_delay_ms = 10;
    
    cJSON *samples_item = cJSON_GetObjectItem(json, "samples");
    if (samples_item != NULL && cJSON_IsNumber(samples_item)) {
        samples = (uint32_t)cJSON_GetNumberValue(samples_item);
        if (samples < 1) samples = 1;
        if (samples > 10000) samples = 10000;
    }
    
    cJSON *delay_item = cJSON_GetObjectItem(json, "sample_delay_ms");
    if (delay_item != NULL && cJSON_IsNumber(delay_item)) {
        sample_delay_ms = (uint32_t)cJSON_GetNumberValue(delay_item);
        if (sample_delay_ms < 1) sample_delay_ms = 1;
        if (sample_delay_ms > 1000) sample_delay_ms = 1000;
    }
    
    esp_err_t err = ESP_ERR_INVALID_ARG;
    const char *calibration_type = "";
    
    if (strcmp(type, "gyro") == 0) {
        err = lsm6ds3_manager_calibrate_gyro(samples, sample_delay_ms);
        calibration_type = "gyroscope";
    } else {
        cJSON_Delete(json);
        return send_json_error(req, "Invalid 'type' field (must be 'gyro')", 400);
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    
    if (err == ESP_OK) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", calibration_type);
        cJSON_AddStringToObject(response, "type", calibration_type);
        cJSON_AddNumberToObject(response, "samples", samples);
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", esp_err_to_name(err));
    }
    
    return send_json_response(req, response, err == ESP_OK ? ESP_OK : ESP_FAIL);
}

// POST /api/lsm6ds3/calibrate/preview - Preview LSM6DS3 calibration without saving
static esp_err_t api_post_lsm6ds3_calibrate_preview_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *type_item = cJSON_GetObjectItem(json, "type");
    if (type_item == NULL || !cJSON_IsString(type_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'type' field (must be 'gyro')", 400);
    }
    
    const char *type_ptr = cJSON_GetStringValue(type_item);
    if (strcmp(type_ptr, "gyro") != 0) {
        cJSON_Delete(json);
        return send_json_error(req, "Invalid 'type' field (must be 'gyro')", 400);
    }
    
    uint32_t samples = 100;
    uint32_t sample_delay_ms = 10;
    
    cJSON *samples_item = cJSON_GetObjectItem(json, "samples");
    if (samples_item != NULL && cJSON_IsNumber(samples_item)) {
        samples = (uint32_t)cJSON_GetNumberValue(samples_item);
        if (samples < 1) samples = 1;
        if (samples > 10000) samples = 10000;
    }
    
    cJSON *delay_item = cJSON_GetObjectItem(json, "sample_delay_ms");
    if (delay_item != NULL && cJSON_IsNumber(delay_item)) {
        sample_delay_ms = (uint32_t)cJSON_GetNumberValue(delay_item);
        if (sample_delay_ms < 1) sample_delay_ms = 1;
        if (sample_delay_ms > 1000) sample_delay_ms = 1000;
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    
    float offset_mdps[3] = {0};
    esp_err_t err = lsm6ds3_manager_calibrate_gyro_preview(samples, sample_delay_ms, offset_mdps);
    
    if (err == ESP_OK) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", "Calibration preview completed");
        cJSON_AddStringToObject(response, "type", "gyroscope");
        cJSON_AddNumberToObject(response, "samples", samples);
        
        cJSON *offsets_json = cJSON_CreateArray();
        cJSON_AddItemToArray(offsets_json, cJSON_CreateNumber(offset_mdps[0]));
        cJSON_AddItemToArray(offsets_json, cJSON_CreateNumber(offset_mdps[1]));
        cJSON_AddItemToArray(offsets_json, cJSON_CreateNumber(offset_mdps[2]));
        cJSON_AddItemToObject(response, "offset_mdps", offsets_json);
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", esp_err_to_name(err));
    }
    
    return send_json_response(req, response, err == ESP_OK ? ESP_OK : ESP_FAIL);
}

// POST /api/lsm6ds3/zero - Set LSM6DS3 angle zero offsets
static esp_err_t api_post_lsm6ds3_zero_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    float roll = 0, pitch = 0, yaw = 0;
    bool roll_provided = false, pitch_provided = false, yaw_provided = false;
    
    cJSON *roll_item = cJSON_GetObjectItem(json, "roll");
    if (roll_item != NULL && cJSON_IsNumber(roll_item)) {
        roll = (float)cJSON_GetNumberValue(roll_item);
        roll_provided = true;
    }
    
    cJSON *pitch_item = cJSON_GetObjectItem(json, "pitch");
    if (pitch_item != NULL && cJSON_IsNumber(pitch_item)) {
        pitch = (float)cJSON_GetNumberValue(pitch_item);
        pitch_provided = true;
    }
    
    cJSON *yaw_item = cJSON_GetObjectItem(json, "yaw");
    if (yaw_item != NULL && cJSON_IsNumber(yaw_item)) {
        yaw = (float)cJSON_GetNumberValue(yaw_item);
        yaw_provided = true;
    }
    
    cJSON *clear_item = cJSON_GetObjectItem(json, "clear");
    bool clear = false;
    if (clear_item != NULL && cJSON_IsBool(clear_item)) {
        clear = cJSON_IsTrue(clear_item);
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    esp_err_t err = ESP_OK;
    
    if (clear) {
        err = lsm6ds3_manager_clear_angle_zero();
        if (err == ESP_OK) {
            cJSON_AddStringToObject(response, "status", "ok");
            cJSON_AddStringToObject(response, "message", "Angle zero offsets cleared successfully");
        } else {
            cJSON_AddStringToObject(response, "status", "error");
            cJSON_AddStringToObject(response, "message", esp_err_to_name(err));
        }
        return send_json_response(req, response, err == ESP_OK ? ESP_OK : ESP_FAIL);
    }
    
    if (!roll_provided && !pitch_provided && !yaw_provided) {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", "At least one angle (roll, pitch, or yaw) must be provided, or set 'clear' to true");
        return send_json_response(req, response, ESP_ERR_INVALID_ARG);
    }
    
    err = lsm6ds3_manager_set_angle_zero(roll, pitch, yaw);
    
    if (err == ESP_OK) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", "Angle zero offsets saved successfully");
        cJSON_AddNumberToObject(response, "roll", roll);
        cJSON_AddNumberToObject(response, "pitch", pitch);
        cJSON_AddNumberToObject(response, "yaw", yaw);
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", esp_err_to_name(err));
    }
    
    return send_json_response(req, response, err == ESP_OK ? ESP_OK : ESP_FAIL);
}

// GET /api/gp8403 - Get GP8403 DAC status and values
static esp_err_t api_get_gp8403_handler(httpd_req_t *req)
{
    if (!s_gp8403_enabled_cached) {
        s_cached_gp8403_enabled = system_gp8403_dac_enabled_load();
        s_gp8403_enabled_cached = true;
    }
    bool enabled = s_cached_gp8403_enabled;
    bool initialized = gp8403_dac_manager_is_initialized();
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", enabled);
    cJSON_AddBoolToObject(json, "initialized", initialized);
    
    i2c_device_counts_t device_counts;
    i2c_bus_manager_get_device_counts(&device_counts);
    cJSON_AddNumberToObject(json, "device_count", device_counts.gp8403_count);
    
    SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
    if (mutex != NULL && xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        cJSON *devices = cJSON_CreateArray();
        for (int i = 0; i < 4; i++) {
            uint16_t ch0 = 0, ch1 = 0;
            int offset = GP8403_DAC_BYTE_START + (i * 4);
            if (offset + 3 < sizeof(g_assembly_data096)) {
                memcpy(&ch0, &g_assembly_data096[offset], sizeof(uint16_t));
                memcpy(&ch1, &g_assembly_data096[offset + 2], sizeof(uint16_t));
            }
            
            cJSON *device = cJSON_CreateObject();
            cJSON_AddNumberToObject(device, "address", 0x58 + i);
            cJSON_AddNumberToObject(device, "channel_0", ch0);
            cJSON_AddNumberToObject(device, "channel_1", ch1);
            cJSON_AddItemToArray(devices, device);
        }
        cJSON_AddItemToObject(json, "devices", devices);
        xSemaphoreGive(mutex);
    }
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/gp8403 - Set GP8403 DAC enabled state or values
static esp_err_t api_post_gp8403_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
    bool enabled = false;
    
    if (enabled_item != NULL) {
        if (cJSON_IsBool(enabled_item)) {
            enabled = cJSON_IsTrue(enabled_item);
        } else if (cJSON_IsNumber(enabled_item)) {
            enabled = (cJSON_GetNumberValue(enabled_item) != 0);
        } else {
            cJSON_Delete(json);
            return send_json_error(req, "Invalid 'enabled' field (must be boolean or number)", 400);
        }
        
        system_gp8403_dac_enabled_save(enabled);
        s_cached_gp8403_enabled = enabled;
        s_gp8403_enabled_cached = true;
        cJSON_Delete(json);
        cJSON *response = cJSON_CreateObject();
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddBoolToObject(response, "enabled", enabled);
        cJSON_AddStringToObject(response, "message", "GP8403 configuration saved. Reboot required.");
        return send_json_response(req, response, ESP_OK);
    }
    
    cJSON_Delete(json);
    return send_json_error(req, "Missing 'enabled' field", 400);
}

// GET /api/mcp230xx - Get MCP230XX status and GPIO states
static esp_err_t api_get_mcp230xx_handler(httpd_req_t *req)
{
    bool initialized = mcp230xx_manager_is_initialized();
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "initialized", initialized);
    
    SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
    if (mutex != NULL && xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        cJSON *output = cJSON_CreateArray();
        cJSON *feedback = cJSON_CreateArray();
        
        for (int i = 0; i < 16; i++) {
            cJSON_AddItemToArray(output, cJSON_CreateNumber(g_assembly_data096[MCP230XX_OUTPUT_BYTE_START + i]));
            cJSON_AddItemToArray(feedback, cJSON_CreateNumber(g_assembly_data064[MCP230XX_FEEDBACK_BYTE_START + i]));
        }
        
        cJSON_AddItemToObject(json, "output", output);
        cJSON_AddItemToObject(json, "feedback", feedback);
        xSemaphoreGive(mutex);
    }
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/mcp230xx - Set MCP230XX GPIO output states
static esp_err_t api_post_mcp230xx_handler(httpd_req_t *req)
{
    char content[512];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *output_item = cJSON_GetObjectItem(json, "output");
    if (output_item != NULL && cJSON_IsArray(output_item)) {
        SemaphoreHandle_t mutex = fusion_core_get_assembly_mutex();
        if (mutex != NULL && xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            int size = cJSON_GetArraySize(output_item);
            int max = (size < 16) ? size : 16;
            
            for (int i = 0; i < max; i++) {
                cJSON *item = cJSON_GetArrayItem(output_item, i);
                if (item != NULL && cJSON_IsNumber(item)) {
                    double num_value = cJSON_GetNumberValue(item);
                    if (num_value < 0) num_value = 0;
                    if (num_value > 255) num_value = 255;
                    uint8_t value = (uint8_t)num_value;
                    g_assembly_data096[MCP230XX_OUTPUT_BYTE_START + i] = value;
                }
            }
            xSemaphoreGive(mutex);
        }
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    cJSON_AddStringToObject(response, "status", "ok");
    cJSON_AddStringToObject(response, "message", "MCP230XX GPIO states updated");
    
    return send_json_response(req, response, ESP_OK);
}

// GET /api/mcp230xx/enabled - Get MCP230XX enabled state
static esp_err_t api_get_mcp230xx_enabled_handler(httpd_req_t *req)
{
    if (!s_mcp_enabled_cached) {
        s_cached_mcp_enabled = system_mcp_enabled_load();
        s_mcp_enabled_cached = true;
    }
    bool enabled = s_cached_mcp_enabled;
    
    cJSON *json = cJSON_CreateObject();
    cJSON_AddBoolToObject(json, "enabled", enabled);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/mcp230xx/enabled - Set MCP230XX enabled state
static esp_err_t api_post_mcp230xx_enabled_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
    if (enabled_item == NULL || !cJSON_IsBool(enabled_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'enabled' field", 400);
    }
    
    bool enabled = cJSON_IsTrue(enabled_item);
    bool saved = system_mcp_enabled_save(enabled);
    
    if (saved) {
        s_cached_mcp_enabled = enabled;
        s_mcp_enabled_cached = true;
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    
    if (saved) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddBoolToObject(response, "enabled", enabled);
        cJSON_AddStringToObject(response, "message", enabled ? "MCP230XX enabled. Reboot required." : "MCP230XX disabled. Reboot required.");
        return send_json_response(req, response, ESP_OK);
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", "Failed to save MCP230XX enabled state");
        return send_json_response(req, response, ESP_FAIL);
    }
}

// GET /api/mcp230xx/devices - Get detected and configured MCP230XX devices
static esp_err_t api_get_mcp230xx_devices_handler(httpd_req_t *req)
{
    cJSON *json = cJSON_CreateObject();
    cJSON *devices = cJSON_CreateArray();
    
    if (!s_mcp_config_cached) {
        if (mcp_config_load_all(&s_cached_mcp_config)) {
            s_mcp_config_cached = true;
        }
    }
    
    mcp_detected_devices_t *detected = mcp_config_get_detected_devices();
    
    for (uint8_t addr = 0x20; addr <= 0x27; addr++) {
        bool detected_at_boot = false;
        uint8_t detected_device_type = 0xFF;
        
        for (int i = 0; i < detected->device_count; i++) {
            if (detected->devices[i].i2c_address == addr && detected->devices[i].detected) {
                detected_at_boot = true;
                detected_device_type = detected->devices[i].device_type;
                break;
            }
        }
        
        mcp_device_config_t *device_config = NULL;
        if (s_mcp_config_cached) {
            device_config = mcp_config_find_device(&s_cached_mcp_config, addr);
        }
        
        cJSON *device = cJSON_CreateObject();
        char addr_str[8];
        snprintf(addr_str, sizeof(addr_str), "0x%02X", addr);
        cJSON_AddStringToObject(device, "address", addr_str);
        cJSON_AddBoolToObject(device, "detected", detected_at_boot);
        
        if (device_config != NULL) {
            cJSON_AddBoolToObject(device, "configured", true);
            cJSON_AddBoolToObject(device, "enabled", device_config->enabled);
            cJSON_AddNumberToObject(device, "device_type", device_config->device_type);
            const char *type_str = (device_config->device_type == 0) ? "MCP23017" : "MCP23008";
            cJSON_AddStringToObject(device, "device_type_name", type_str);
        } else {
            cJSON_AddBoolToObject(device, "configured", false);
            cJSON_AddBoolToObject(device, "enabled", false);
            if (detected_at_boot && detected_device_type != 0xFF) {
                cJSON_AddNumberToObject(device, "device_type", detected_device_type);
                const char *type_str = (detected_device_type == 0) ? "MCP23017" : "MCP23008";
                cJSON_AddStringToObject(device, "device_type_name", type_str);
            } else {
                cJSON_AddNullToObject(device, "device_type");
                cJSON_AddStringToObject(device, "device_type_name", "Not configured");
            }
        }
        
        cJSON_AddItemToArray(devices, device);
    }
    
    cJSON_AddItemToObject(json, "devices", devices);
    if (!s_mcp_enabled_cached) {
        s_cached_mcp_enabled = system_mcp_enabled_load();
        s_mcp_enabled_cached = true;
    }
    cJSON_AddBoolToObject(json, "global_enabled", s_cached_mcp_enabled);
    
    return send_json_response(req, json, ESP_OK);
}

// POST /api/mcp230xx/config - Configure MCP230XX device type
static esp_err_t api_post_mcp230xx_config_handler(httpd_req_t *req)
{
    char content[256];
    int ret = httpd_req_recv(req, content, sizeof(content) - 1);
    if (ret <= 0) {
        return send_json_error(req, "Failed to receive request", 400);
    }
    content[ret] = '\0';
    
    cJSON *json = cJSON_Parse(content);
    if (json == NULL) {
        return send_json_error(req, "Invalid JSON", 400);
    }
    
    cJSON *address_item = cJSON_GetObjectItem(json, "address");
    cJSON *device_type_item = cJSON_GetObjectItem(json, "device_type");
    cJSON *enabled_item = cJSON_GetObjectItem(json, "enabled");
    
    if (address_item == NULL || !cJSON_IsString(address_item)) {
        cJSON_Delete(json);
        return send_json_error(req, "Missing or invalid 'address' field (must be string like '0x20')", 400);
    }
    
    uint8_t address = 0;
    if (sscanf(cJSON_GetStringValue(address_item), "0x%hhx", &address) != 1) {
        cJSON_Delete(json);
        return send_json_error(req, "Invalid address format (must be like '0x20')", 400);
    }
    
    if (address < 0x20 || address > 0x27) {
        cJSON_Delete(json);
        return send_json_error(req, "Address must be between 0x20 and 0x27", 400);
    }
    
    if (!s_mcp_config_cached) {
        if (mcp_config_load_all(&s_cached_mcp_config)) {
            s_mcp_config_cached = true;
        }
    }
    
    mcp_config_t config = s_cached_mcp_config;
    
    mcp_device_config_t *device_config = mcp_config_find_device(&config, address);
    
    if (device_config == NULL) {
        if (config.device_count >= MCP_MAX_DEVICES) {
            cJSON_Delete(json);
            return send_json_error(req, "Maximum number of devices reached", 400);
        }
        
        uint8_t default_device_type = 1;
        mcp_detected_devices_t *detected = mcp_config_get_detected_devices();
        for (int i = 0; i < detected->device_count; i++) {
            if (detected->devices[i].i2c_address == address && detected->devices[i].detected && detected->devices[i].device_type != 0xFF) {
                default_device_type = detected->devices[i].device_type;
                break;
            }
        }
        
        device_config = &config.devices[config.device_count];
        mcp_config_get_defaults(device_config, address, default_device_type);
        config.device_count++;
    }
    
    if (device_type_item != NULL && cJSON_IsNumber(device_type_item)) {
        int device_type = (int)cJSON_GetNumberValue(device_type_item);
        if (device_type != 0 && device_type != 1) {
            cJSON_Delete(json);
            return send_json_error(req, "device_type must be 0 (MCP23017) or 1 (MCP23008)", 400);
        }
        device_config->device_type = (uint8_t)device_type;
    }
    
    if (enabled_item != NULL && cJSON_IsBool(enabled_item)) {
        device_config->enabled = cJSON_IsTrue(enabled_item);
    }
    
    bool saved = mcp_config_save_all(&config);
    
    if (saved) {
        s_cached_mcp_config = config;
        s_mcp_config_cached = true;
    }
    
    cJSON_Delete(json);
    cJSON *response = cJSON_CreateObject();
    
    if (saved) {
        cJSON_AddStringToObject(response, "status", "ok");
        cJSON_AddStringToObject(response, "message", "Device configuration saved. Reboot required.");
        return send_json_response(req, response, ESP_OK);
    } else {
        cJSON_AddStringToObject(response, "status", "error");
        cJSON_AddStringToObject(response, "message", "Failed to save device configuration");
        return send_json_response(req, response, ESP_FAIL);
    }
}

void webui_register_api_handlers(httpd_handle_t server)
{
    if (server == NULL) {
        ESP_LOGE(TAG, "Cannot register API handlers: server handle is NULL!");
        return;
    }
    
    ESP_LOGI(TAG, "Registering API handlers...");
    
    // POST /api/ota/update
    httpd_uri_t ota_update_uri = {
        .uri       = "/api/ota/update",
        .method    = HTTP_POST,
        .handler   = api_ota_update_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &ota_update_uri);
    
    // GET /api/ota/status
    httpd_uri_t ota_status_uri = {
        .uri       = "/api/ota/status",
        .method    = HTTP_GET,
        .handler   = api_ota_status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &ota_status_uri);
    
    // POST /api/reboot
    httpd_uri_t reboot_uri = {
        .uri       = "/api/reboot",
        .method    = HTTP_POST,
        .handler   = api_reboot_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &reboot_uri);
    
    // GET /api/assemblies/sizes
    httpd_uri_t get_assemblies_sizes_uri = {
        .uri       = "/api/assemblies/sizes",
        .method    = HTTP_GET,
        .handler   = api_get_assemblies_sizes_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_assemblies_sizes_uri);
    
    // GET /api/status - Get assembly data for status pages
    httpd_uri_t get_status_uri = {
        .uri       = "/api/status",
        .method    = HTTP_GET,
        .handler   = api_get_status_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_status_uri);
    
    // GET /api/i2c/pullup
    httpd_uri_t get_i2c_pullup_uri = {
        .uri       = "/api/i2c/pullup",
        .method    = HTTP_GET,
        .handler   = api_get_i2c_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_i2c_pullup_uri);
    
    // POST /api/i2c/pullup
    httpd_uri_t post_i2c_pullup_uri = {
        .uri       = "/api/i2c/pullup",
        .method    = HTTP_POST,
        .handler   = api_post_i2c_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_i2c_pullup_uri);
    
    // GET /api/i2c/pullup/primary
    httpd_uri_t get_i2c_primary_pullup_uri = {
        .uri       = "/api/i2c/pullup/primary",
        .method    = HTTP_GET,
        .handler   = api_get_i2c_primary_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_i2c_primary_pullup_uri);
    
    // POST /api/i2c/pullup/primary
    httpd_uri_t post_i2c_primary_pullup_uri = {
        .uri       = "/api/i2c/pullup/primary",
        .method    = HTTP_POST,
        .handler   = api_post_i2c_primary_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_i2c_primary_pullup_uri);
    
    // GET /api/i2c/pullup/secondary
    httpd_uri_t get_i2c_secondary_pullup_uri = {
        .uri       = "/api/i2c/pullup/secondary",
        .method    = HTTP_GET,
        .handler   = api_get_i2c_secondary_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_i2c_secondary_pullup_uri);
    
    // POST /api/i2c/pullup/secondary
    httpd_uri_t post_i2c_secondary_pullup_uri = {
        .uri       = "/api/i2c/pullup/secondary",
        .method    = HTTP_POST,
        .handler   = api_post_i2c_secondary_pullup_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_i2c_secondary_pullup_uri);
    
    // GET /api/logs - Get system logs
    httpd_uri_t get_logs_uri = {
        .uri       = "/api/logs",
        .method    = HTTP_GET,
        .handler   = api_get_logs_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_logs_uri);
    
    // GET /api/ipconfig
    httpd_uri_t get_ipconfig_uri = {
        .uri       = "/api/ipconfig",
        .method    = HTTP_GET,
        .handler   = api_get_ipconfig_handler,
        .user_ctx  = NULL
    };
    esp_err_t ret = httpd_register_uri_handler(server, &get_ipconfig_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /api/ipconfig: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Registered GET /api/ipconfig handler");
    }
    
    // POST /api/ipconfig
    httpd_uri_t post_ipconfig_uri = {
        .uri       = "/api/ipconfig",
        .method    = HTTP_POST,
        .handler   = api_post_ipconfig_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(server, &post_ipconfig_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register POST /api/ipconfig: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Registered POST /api/ipconfig handler");
    }
    
    // GET /api/nau7802
    httpd_uri_t get_nau7802_uri = {
        .uri       = "/api/nau7802",
        .method    = HTTP_GET,
        .handler   = api_get_nau7802_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_nau7802_uri);
    
    // POST /api/nau7802
    httpd_uri_t post_nau7802_uri = {
        .uri       = "/api/nau7802",
        .method    = HTTP_POST,
        .handler   = api_post_nau7802_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_nau7802_uri);
    
    // POST /api/nau7802/calibrate
    httpd_uri_t post_nau7802_calibrate_uri = {
        .uri       = "/api/nau7802/calibrate",
        .method    = HTTP_POST,
        .handler   = api_post_nau7802_calibrate_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_nau7802_calibrate_uri);
    
    // GET /api/vl53l1x
    httpd_uri_t get_vl53l1x_uri = {
        .uri       = "/api/vl53l1x",
        .method    = HTTP_GET,
        .handler   = api_get_vl53l1x_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_vl53l1x_uri);
    
    // POST /api/vl53l1x
    httpd_uri_t post_vl53l1x_uri = {
        .uri       = "/api/vl53l1x",
        .method    = HTTP_POST,
        .handler   = api_post_vl53l1x_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_vl53l1x_uri);
    
    // GET /api/vl53l1x/config
    httpd_uri_t get_vl53l1x_config_uri = {
        .uri       = "/api/vl53l1x/config",
        .method    = HTTP_GET,
        .handler   = api_get_vl53l1x_config_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_vl53l1x_config_uri);
    
    // POST /api/vl53l1x/config
    httpd_uri_t post_vl53l1x_config_uri = {
        .uri       = "/api/vl53l1x/config",
        .method    = HTTP_POST,
        .handler   = api_post_vl53l1x_config_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_vl53l1x_config_uri);
    
    // POST /api/vl53l1x/calibrate/offset
    httpd_uri_t post_vl53l1x_calibrate_offset_uri = {
        .uri       = "/api/vl53l1x/calibrate/offset",
        .method    = HTTP_POST,
        .handler   = api_post_vl53l1x_calibrate_offset_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_vl53l1x_calibrate_offset_uri);
    
    // POST /api/vl53l1x/calibrate/xtalk
    httpd_uri_t post_vl53l1x_calibrate_xtalk_uri = {
        .uri       = "/api/vl53l1x/calibrate/xtalk",
        .method    = HTTP_POST,
        .handler   = api_post_vl53l1x_calibrate_xtalk_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_vl53l1x_calibrate_xtalk_uri);
    
    // GET /api/lsm6ds3
    httpd_uri_t get_lsm6ds3_uri = {
        .uri       = "/api/lsm6ds3",
        .method    = HTTP_GET,
        .handler   = api_get_lsm6ds3_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_lsm6ds3_uri);
    
    // POST /api/lsm6ds3
    httpd_uri_t post_lsm6ds3_uri = {
        .uri       = "/api/lsm6ds3",
        .method    = HTTP_POST,
        .handler   = api_post_lsm6ds3_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_lsm6ds3_uri);
    
    // GET /api/lsm6ds3/calibrate
    httpd_uri_t get_lsm6ds3_calibrate_uri = {
        .uri       = "/api/lsm6ds3/calibrate",
        .method    = HTTP_GET,
        .handler   = api_get_lsm6ds3_calibrate_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_lsm6ds3_calibrate_uri);
    
    // POST /api/lsm6ds3/calibrate
    httpd_uri_t post_lsm6ds3_calibrate_uri = {
        .uri       = "/api/lsm6ds3/calibrate",
        .method    = HTTP_POST,
        .handler   = api_post_lsm6ds3_calibrate_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_lsm6ds3_calibrate_uri);
    
    // POST /api/lsm6ds3/calibrate/preview
    httpd_uri_t post_lsm6ds3_calibrate_preview_uri = {
        .uri       = "/api/lsm6ds3/calibrate/preview",
        .method    = HTTP_POST,
        .handler   = api_post_lsm6ds3_calibrate_preview_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_lsm6ds3_calibrate_preview_uri);
    
    // POST /api/lsm6ds3/zero
    httpd_uri_t post_lsm6ds3_zero_uri = {
        .uri       = "/api/lsm6ds3/zero",
        .method    = HTTP_POST,
        .handler   = api_post_lsm6ds3_zero_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_lsm6ds3_zero_uri);
    
    // GET /api/lsm6ds3/nvs-calibration
    httpd_uri_t get_lsm6ds3_nvs_calibration_uri = {
        .uri       = "/api/lsm6ds3/nvs-calibration",
        .method    = HTTP_GET,
        .handler   = api_get_lsm6ds3_nvs_calibration_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_lsm6ds3_nvs_calibration_uri);
    
    // GET /api/gp8403
    httpd_uri_t get_gp8403_uri = {
        .uri       = "/api/gp8403",
        .method    = HTTP_GET,
        .handler   = api_get_gp8403_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_gp8403_uri);
    
    // POST /api/gp8403
    httpd_uri_t post_gp8403_uri = {
        .uri       = "/api/gp8403",
        .method    = HTTP_POST,
        .handler   = api_post_gp8403_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_gp8403_uri);
    
    // GET /api/mcp230xx
    httpd_uri_t get_mcp230xx_uri = {
        .uri       = "/api/mcp230xx",
        .method    = HTTP_GET,
        .handler   = api_get_mcp230xx_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_mcp230xx_uri);
    
    // POST /api/mcp230xx
    httpd_uri_t post_mcp230xx_uri = {
        .uri       = "/api/mcp230xx",
        .method    = HTTP_POST,
        .handler   = api_post_mcp230xx_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_mcp230xx_uri);
    
    // GET /api/mcp230xx/enabled
    httpd_uri_t get_mcp230xx_enabled_uri = {
        .uri       = "/api/mcp230xx/enabled",
        .method    = HTTP_GET,
        .handler   = api_get_mcp230xx_enabled_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &get_mcp230xx_enabled_uri);
    
    // POST /api/mcp230xx/enabled
    httpd_uri_t post_mcp230xx_enabled_uri = {
        .uri       = "/api/mcp230xx/enabled",
        .method    = HTTP_POST,
        .handler   = api_post_mcp230xx_enabled_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_mcp230xx_enabled_uri);
    
    // GET /api/mcp230xx/devices
    httpd_uri_t get_mcp230xx_devices_uri = {
        .uri       = "/api/mcp230xx/devices",
        .method    = HTTP_GET,
        .handler   = api_get_mcp230xx_devices_handler,
        .user_ctx  = NULL
    };
    ret = httpd_register_uri_handler(server, &get_mcp230xx_devices_uri);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GET /api/mcp230xx/devices: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Registered GET /api/mcp230xx/devices handler");
    }
    
    // POST /api/mcp230xx/config
    httpd_uri_t post_mcp230xx_config_uri = {
        .uri       = "/api/mcp230xx/config",
        .method    = HTTP_POST,
        .handler   = api_post_mcp230xx_config_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &post_mcp230xx_config_uri);
    
    ESP_LOGI(TAG, "API handler registration complete");
}

