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

/**
 * @file main.c
 * @brief Main application entry point for ESP32-P4 EtherNet/IP device
 *
 * ADDRESS CONFLICT DETECTION (ACD) IMPLEMENTATION
 * ===============================================
 *
 * This file implements RFC 5227 compliant Address Conflict Detection (ACD) for
 * static IP addresses. ACD ensures that IP addresses are not assigned until
 * confirmed safe to use, preventing network conflicts.
 *
 * Architecture:
 * ------------
 * - Static IP: RFC 5227 compliant behavior (implemented in application layer)
 *   * Probe phase: 3 ARP probes from 0.0.0.0 with configurable intervals (default: 200ms)
 *   * Announce phase: 4 ARP announcements after successful probe (default: 2000ms intervals)
 *   * Ongoing defense: Periodic ARP probes every ~90 seconds (configurable)
 *   * Total time: ~6-10 seconds for initial IP assignment
 *   * ACD probe sequence runs BEFORE IP assignment
 *   * IP assigned only after ACD confirms no conflict (ACD_IP_OK callback)
 *
 * - DHCP: Simplified ACD (not fully RFC 5227 compliant)
 *   * ACD check performed by lwIP DHCP client before accepting IP
 *   * Handled internally by lwIP DHCP client
 *
 * Implementation:
 * --------------
 * The ACD implementation is in the application layer (this file) and coordinates
 * with the lwIP ACD module. The implementation follows RFC 5227 behavior:
 * - ACD probe sequence completes before IP assignment
 * - Uses tcpip_perform_acd() to coordinate probe sequence
 * - IP assignment deferred until ACD_IP_OK callback received
 * - Natural state machine flow: PROBE_WAIT → PROBING → ANNOUNCE_WAIT → ANNOUNCING → ONGOING
 *
 * Features:
 * --------
 * 1. Retry Logic (CONFIG_OPENER_ACD_RETRY_ENABLED):
 *    - On conflict, removes IP and schedules retry after delay
 *    - Configurable max attempts and retry delay
 *    - Prevents infinite retry loops
 *
 * 2. User LED Indication:
 *    - GPIO27 blinks during normal operation
 *    - Goes solid on ACD conflict detection
 *    - Visual feedback for network issues
 *
 * 3. Callback Tracking:
 *    - Distinguishes between callback events and timeout conditions
 *    - Prevents false positive conflict detection when probe sequence is still running
 *    - IP assignment occurs in callback when ACD_IP_OK fires
 *
 * Thread Safety:
 * -------------
 * - ACD operations use tcpip_callback_with_block() to ensure execution on tcpip thread
 * - Context structures allocated on heap to prevent stack corruption
 * - Semaphores coordinate async callback execution
 *
 * Configuration:
 * --------------
 * - CONFIG_OPENER_ACD_PROBE_NUM: Number of probes (default: 3)
 * - CONFIG_OPENER_ACD_PROBE_WAIT_MS: Initial delay before probing (default: 200ms)
 * - CONFIG_OPENER_ACD_PROBE_MIN_MS: Minimum delay between probes (default: 200ms)
 * - CONFIG_OPENER_ACD_PROBE_MAX_MS: Maximum delay between probes (default: 200ms)
 * - CONFIG_OPENER_ACD_ANNOUNCE_NUM: Number of announcements (default: 4)
 * - CONFIG_OPENER_ACD_ANNOUNCE_INTERVAL_MS: Time between announcements (default: 2000ms)
 * - CONFIG_OPENER_ACD_ANNOUNCE_WAIT_MS: Delay before announcing (default: 2000ms)
 * - CONFIG_OPENER_ACD_PERIODIC_DEFEND_INTERVAL_MS: Defensive ARP interval (default: 90000ms)
 * - CONFIG_OPENER_ACD_RETRY_ENABLED: Enable retry on conflict
 * - CONFIG_OPENER_ACD_RETRY_DELAY_MS: Delay before retry (default: 10000ms)
 * - CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS: Max retry attempts (default: 5)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lwip/netif.h"
#include "lwip/err.h"
#include "lwip/tcpip.h"
#include "lwip/ip4_addr.h"
#include "lwip/netifapi.h"
#include "lwip/timeouts.h"
#include "lwip/etharp.h"
#include "nvs_flash.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "opener.h"
#include "nvtcpip.h"
#include "ciptcpipinterface.h"
#include "sdkconfig.h"
#include "esp_netif_net_stack.h"
#include "webui.h"
#include "ota_manager.h"
#include "system_config.h"
#include "log_buffer.h"
#include "eth_media_counters.h"
#include "i2c_bus_manager.h"
#include "vl53l1x_manager.h"
#include "lsm6ds3_manager.h"
#include "nau7802_manager.h"
#include "gp8403_dac_manager.h"
#include "mcp230xx_manager.h"
#include "fusion_core_assembly.h"
#include "acd_manager.h"
#if OPENER_LLDP_ENABLED
#include "esp_vfs_l2tap.h"
#endif

SemaphoreHandle_t fusion_core_get_assembly_mutex(void);
SemaphoreHandle_t scale_application_get_assembly_mutex(void);  // Backward compatibility

void FusionCoreSetActiveNetif(struct netif *netif);
void FusionCoreNotifyLinkUp(void);
void FusionCoreNotifyLinkDown(void);
void ScaleApplicationSetActiveNetif(struct netif *netif);  // Backward compatibility
void ScaleApplicationNotifyLinkUp(void);  // Backward compatibility
void ScaleApplicationNotifyLinkDown(void);  // Backward compatibility

// External assembly data arrays (defined in opener component)
extern uint8_t g_assembly_data064[72];  // Input Assembly 100
extern uint8_t g_assembly_data096[40];  // Output Assembly 150

static const char *TAG = "opener_main";
static struct netif *s_netif = NULL;
static SemaphoreHandle_t s_netif_mutex = NULL;
static bool s_services_initialized = false;
static esp_eth_mac_t *s_eth_mac = NULL;  // MAC pointer for media counter access



// User LED state (GPIO27)
#define USER_LED_GPIO 27
static bool s_user_led_initialized = false;
static bool s_user_led_flash_enabled = false;
static TaskHandle_t s_user_led_task_handle = NULL;

static bool s_opener_initialized;

static bool tcpip_config_uses_dhcp(void);
static void configure_hostname(esp_netif_t *netif);
static void opener_configure_dns(esp_netif_t *netif);

static bool ip_info_has_static_address(const esp_netif_ip_info_t *ip_info) {
    if (ip_info == NULL) {
        return false;
    }
    if (ip_info->ip.addr == 0 || ip_info->netmask.addr == 0) {
        return false;
    }
    return true;
}

static bool tcpip_config_uses_dhcp(void) {
    return (g_tcpip.config_control & kTcpipCfgCtrlMethodMask) == kTcpipCfgCtrlDhcp;
}

static bool tcpip_static_config_valid(void) {
    if ((g_tcpip.config_control & kTcpipCfgCtrlMethodMask) != kTcpipCfgCtrlStaticIp) {
        return true;
    }
    return CipTcpIpIsValidNetworkConfig(&g_tcpip.interface_configuration);
}

static void configure_hostname(esp_netif_t *netif) {
    if (g_tcpip.hostname.length > 0 && g_tcpip.hostname.string != NULL) {
        size_t length = g_tcpip.hostname.length;
        if (length > 63) {
            length = 63;
        }
        char host[64];
        memcpy(host, g_tcpip.hostname.string, length);
        host[length] = '\0';
        esp_netif_set_hostname(netif, host);
    }
}

static void opener_configure_dns(esp_netif_t *netif) {
    esp_netif_dns_info_t dns_info = {
        .ip.type = IPADDR_TYPE_V4,
        .ip.u_addr.ip4.addr = g_tcpip.interface_configuration.name_server
    };
    if (dns_info.ip.u_addr.ip4.addr != 0) {
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns_info));
    }

    dns_info.ip.u_addr.ip4.addr = g_tcpip.interface_configuration.name_server_2;
    if (dns_info.ip.u_addr.ip4.addr != 0) {
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, ESP_NETIF_DNS_BACKUP, &dns_info));
    }
}


// User LED control functions
static void user_led_init(void);
static void user_led_set(bool on);
static void user_led_flash_task(void *pvParameters);
static void user_led_start_flash(void);
static void user_led_stop_flash(void);

static void configure_netif_from_tcpip(esp_netif_t *netif) {
    if (netif == NULL) {
        return;
    }

    struct netif *lwip_netif = (struct netif *)esp_netif_get_netif_impl(netif);

    if (tcpip_config_uses_dhcp()) {
        esp_netif_dhcpc_stop(netif);
        esp_netif_dhcpc_start(netif);
    } else {
        esp_netif_ip_info_t ip_info = {0};
        ip_info.ip.addr = g_tcpip.interface_configuration.ip_address;
        ip_info.netmask.addr = g_tcpip.interface_configuration.network_mask;
        ip_info.gw.addr = g_tcpip.interface_configuration.gateway;
        esp_netif_dhcpc_stop(netif);

        if (ip_info_has_static_address(&ip_info)) {
#if LWIP_IPV4 && LWIP_ACD
            if (g_tcpip.select_acd) {
                ESP_LOGI(TAG, "ACD enabled - IP assignment deferred until ACD completes");
                acd_manager_set_ip_config(&ip_info);
                if (lwip_netif != NULL) {
                    acd_manager_start_probe(netif, lwip_netif);
                }
            } else {
                ESP_LOGI(TAG, "ACD disabled - setting static IP immediately");
                CipTcpIpSetLastAcdActivity(0);
                ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));
                opener_configure_dns(netif);
            }
#else
            ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));
            opener_configure_dns(netif);
#endif
        } else {
            ESP_LOGW(TAG, "Static configuration missing IP/mask; attempting AutoIP fallback");
#if CONFIG_LWIP_AUTOIP
            if (lwip_netif != NULL && netifapi_autoip_start(lwip_netif) == ERR_OK) {
                ESP_LOGI(TAG, "AutoIP started successfully");
                g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
                g_tcpip.config_control |= kTcpipCfgCtrlDhcp;
                g_tcpip.interface_configuration.ip_address = 0;
                g_tcpip.interface_configuration.network_mask = 0;
                g_tcpip.interface_configuration.gateway = 0;
                g_tcpip.interface_configuration.name_server = 0;
                g_tcpip.interface_configuration.name_server_2 = 0;
                NvTcpipStore(&g_tcpip);
                return;
            }
            ESP_LOGE(TAG, "AutoIP start failed; falling back to DHCP");
#endif
            ESP_LOGW(TAG, "Switching interface to DHCP due to invalid static configuration");
            g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
            g_tcpip.config_control |= kTcpipCfgCtrlDhcp;
            NvTcpipStore(&g_tcpip);
            ESP_ERROR_CHECK(esp_netif_dhcpc_start(netif));
            return;
        }

    }

    configure_hostname(netif);
    g_tcpip.status |= 0x01;
    g_tcpip.status &= ~kTcpipStatusIfaceCfgPend;
}

static void ethernet_event_handler(void *arg, esp_event_base_t event_base,
                                   int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;
    esp_netif_t *eth_netif = (esp_netif_t *)arg;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
               mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
        ESP_ERROR_CHECK(esp_netif_set_mac(eth_netif, mac_addr));
        
#if OPENER_LLDP_ENABLED
        uint8_t lldp_multicast_mac[6] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x0E};
        esp_err_t mcast_ret = esp_eth_ioctl(eth_handle, ETH_CMD_ADD_MAC_FILTER, lldp_multicast_mac);
        if (mcast_ret == ESP_OK) {
            ESP_LOGI(TAG, "LLDP multicast address (01:80:c2:00:00:0e) added to MAC filter");
        } else {
            ESP_LOGW(TAG, "Failed to add LLDP multicast to MAC filter: %s (LLDP reception may not work)", 
                     esp_err_to_name(mcast_ret));
        }
#endif
        
        // Detect PHY and initialize media counters
        if (s_eth_mac != NULL) {
            bool ip101_detected = EthMediaCountersInit(s_eth_mac, CONFIG_OPENER_ETH_PHY_ADDR);
            if (ip101_detected) {
                ESP_LOGI(TAG, "IP101 PHY detected - media counters enabled");
            } else {
                ESP_LOGI(TAG, "Non-IP101 PHY detected - media counters disabled");
            }
        }
        
        #if LWIP_IPV4 && LWIP_ACD
        if (!tcpip_config_uses_dhcp() && acd_manager_is_probe_pending()) {
            struct netif *lwip_netif = (struct netif *)esp_netif_get_netif_impl(eth_netif);
            acd_manager_start_probe(eth_netif, lwip_netif);
        }
        #endif
        ScaleApplicationNotifyLinkUp();
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        #if LWIP_IPV4 && LWIP_ACD
        acd_manager_stop();
        #endif
        s_opener_initialized = false;
        s_services_initialized = false;  // Allow re-initialization when link comes back up
        ScaleApplicationNotifyLinkDown();
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;
    
    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    
    // Create mutex on first call if needed
    if (s_netif_mutex == NULL) {
        s_netif_mutex = xSemaphoreCreateMutex();
        if (s_netif_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create netif mutex");
            return;
        }
    }
    
    // Take mutex to protect s_netif access
    if (xSemaphoreTake(s_netif_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take netif mutex");
        return;
    }
    
    if (s_netif == NULL) {
        for (struct netif *netif = netif_list; netif != NULL; netif = netif->next) {
            if (netif_is_up(netif) && netif_is_link_up(netif)) {
                s_netif = netif;
                break;
            }
        }
    }
    
    struct netif *netif_to_use = s_netif;
    xSemaphoreGive(s_netif_mutex);
    
    if (netif_to_use != NULL) {
        ScaleApplicationSetActiveNetif(netif_to_use);
        
        // Initialize services only once (IP_EVENT_ETH_GOT_IP can fire multiple times)
        if (!s_services_initialized) {
            opener_init(netif_to_use);
            s_opener_initialized = true;
            ScaleApplicationNotifyLinkUp();
            
            // Initialize OTA manager
            if (!ota_manager_init()) {
                ESP_LOGW(TAG, "Failed to initialize OTA manager");
            }
            
            // Initialize and start Web UI
            if (!webui_init()) {
                ESP_LOGW(TAG, "Failed to initialize Web UI");
            }
            
            // Modbus TCP removed from build (component files remain)
            
            // Note: NAU7802 scale reading task is now created after NAU7802 initialization
            // (in init_services() after NAU7802 begin() succeeds)
            
            s_services_initialized = true;
            ESP_LOGI(TAG, "All services initialized");
        } else {
            ESP_LOGD(TAG, "Services already initialized, skipping re-initialization");
        }
    } else {
        ESP_LOGE(TAG, "Failed to find netif");
    }
}

void app_main(void)
{
    // Initialize user LED early at boot
    user_led_init();
    
    // Initialize log buffer early to capture boot logs
    // Use 32KB buffer to capture boot sequence and recent runtime logs
    if (!log_buffer_init(32 * 1024)) {
        ESP_LOGW(TAG, "Failed to initialize log buffer");
    }
    
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);
    
    // Mark the current running app as valid to allow OTA updates
    // This must be done after NVS init and before any OTA operations
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (running != NULL) {
        esp_ota_img_states_t ota_state;
        if (esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
            if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
                ESP_LOGI(TAG, "Marking OTA image as valid");
                esp_err_t ret = esp_ota_mark_app_valid_cancel_rollback();
                if (ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to mark app as valid: %s", esp_err_to_name(ret));
                }
            }
        }
    }
    
    (void)NvTcpipLoad(&g_tcpip);
    ESP_LOGI(TAG, "After NV load select_acd=%d", g_tcpip.select_acd);
    
    // Note: ACD setting (select_acd) is now respected from NV storage.
    // Users can disable ACD via attribute 10, and it will persist across reboots.
    // Previously, this code would auto-enable ACD for static IP, overriding user preference.

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

#if LWIP_IPV4 && LWIP_ACD
    ESP_ERROR_CHECK(acd_manager_init());
    acd_manager_set_led_control_callback(user_led_start_flash, user_led_stop_flash, user_led_set);
    acd_manager_set_dns_config_callback(opener_configure_dns);
#endif

    // Register L2 TAP VFS early - BEFORE Ethernet driver starts
    // This ensures the filter callback is properly registered with the Ethernet netif glue
    #if OPENER_LLDP_ENABLED
    esp_err_t l2tap_ret = esp_vfs_l2tap_intf_register(NULL);
    if (l2tap_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register L2 TAP VFS early: %s (LLDP may not work)", esp_err_to_name(l2tap_ret));
    } else {
        ESP_LOGI(TAG, "L2 TAP VFS registered early (before Ethernet start)");
    }
    #endif

    /* Ensure default configuration uses DHCP when nothing stored */
    if ((g_tcpip.config_control & kTcpipCfgCtrlMethodMask) != kTcpipCfgCtrlStaticIp &&
        (g_tcpip.config_control & kTcpipCfgCtrlMethodMask) != kTcpipCfgCtrlDhcp) {
        g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
        g_tcpip.config_control |= kTcpipCfgCtrlDhcp;
    }
    if (!tcpip_static_config_valid()) {
        ESP_LOGW(TAG, "Invalid static configuration detected, switching to DHCP");
        g_tcpip.config_control &= ~kTcpipCfgCtrlMethodMask;
        g_tcpip.config_control |= kTcpipCfgCtrlDhcp;
        g_tcpip.interface_configuration.ip_address = 0;
        g_tcpip.interface_configuration.network_mask = 0;
        g_tcpip.interface_configuration.gateway = 0;
        g_tcpip.interface_configuration.name_server = 0;
        g_tcpip.interface_configuration.name_server_2 = 0;
        g_tcpip.status &= ~(kTcpipStatusAcdStatus | kTcpipStatusAcdFault);
        NvTcpipStore(&g_tcpip);
    }
    if (tcpip_config_uses_dhcp()) {
        g_tcpip.interface_configuration.ip_address = 0;
        g_tcpip.interface_configuration.network_mask = 0;
        g_tcpip.interface_configuration.gateway = 0;
        g_tcpip.interface_configuration.name_server = 0;
        g_tcpip.interface_configuration.name_server_2 = 0;
    }

    g_tcpip.status |= 0x01;
    g_tcpip.status &= ~kTcpipStatusIfaceCfgPend;

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    ESP_ERROR_CHECK(esp_netif_set_default_netif(eth_netif));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, 
                                               &ethernet_event_handler, eth_netif));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, 
                                               &got_ip_event_handler, eth_netif));

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    
    phy_config.phy_addr = CONFIG_OPENER_ETH_PHY_ADDR;
    phy_config.reset_gpio_num = CONFIG_OPENER_ETH_PHY_RST_GPIO;

    esp32_emac_config.smi_gpio.mdc_num = CONFIG_OPENER_ETH_MDC_GPIO;
    esp32_emac_config.smi_gpio.mdio_num = CONFIG_OPENER_ETH_MDIO_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_config);

    // Store MAC pointer for media counter access
    s_eth_mac = mac;

    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));

    configure_netif_from_tcpip(eth_netif);
    
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
    
    ESP_LOGI(TAG, "Initializing I2C Bus Manager...");
    esp_err_t i2c_err = i2c_bus_manager_init();
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C Bus Manager: %s", esp_err_to_name(i2c_err));
    }
    
    ESP_LOGI(TAG, "Initializing sensor managers...");
    vl53l1x_manager_init();
    lsm6ds3_manager_init();
    nau7802_manager_init();
    gp8403_dac_manager_init();
    mcp230xx_manager_init();
    
    i2c_device_counts_t counts;
    i2c_bus_manager_get_device_counts(&counts);
    
    uint8_t device_counts[5] = {0};
    device_counts[0] = vl53l1x_manager_is_initialized() ? 1 : 0;
    device_counts[1] = lsm6ds3_manager_is_initialized() ? 1 : 0;
    device_counts[2] = nau7802_manager_is_initialized() ? 1 : 0;
    device_counts[3] = counts.mcp230xx_count;
    device_counts[4] = counts.gp8403_count;
    
    SemaphoreHandle_t assembly_mutex = fusion_core_get_assembly_mutex();
    if (assembly_mutex != NULL && xSemaphoreTake(assembly_mutex, portMAX_DELAY) == pdTRUE) {
        INPUT_ASSEMBLY_100[56] = device_counts[0];
        INPUT_ASSEMBLY_100[57] = device_counts[1];
        INPUT_ASSEMBLY_100[58] = device_counts[2];
        INPUT_ASSEMBLY_100[59] = device_counts[3];
        INPUT_ASSEMBLY_100[60] = device_counts[4];
        xSemaphoreGive(assembly_mutex);
    }
    
    ESP_LOGI(TAG, "Device counts written to assembly: VL53L1X=%d, LSM6DS3=%d, NAU7802=%d, MCP230XX=%d, GP8403=%d",
             device_counts[0], device_counts[1], device_counts[2], device_counts[3], device_counts[4]);
}

// NAU7802 access functions moved to nau7802_manager component
// These functions are kept for backward compatibility with web API
void* scale_application_get_nau7802_handle(void)
{
    return NULL;
}

bool scale_application_is_nau7802_initialized(void)
{
    return nau7802_manager_is_initialized();
}

SemaphoreHandle_t scale_application_get_nau7802_mutex(void)
{
    return NULL;
}

/* New FusionCore function names - use manager components */
void* fusion_core_get_nau7802_handle(void)
{
    return nau7802_manager_get_device_handle();
}

bool fusion_core_is_nau7802_initialized(void)
{
    return nau7802_manager_is_initialized();
}

SemaphoreHandle_t fusion_core_get_nau7802_mutex(void)
{
    return nau7802_manager_get_mutex();
}



// User LED control functions
static void user_led_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << USER_LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        s_user_led_initialized = true;
        // Start blinking by default at boot
        user_led_start_flash();
        ESP_LOGI(TAG, "User LED initialized on GPIO%d (blinking by default)", USER_LED_GPIO);
    } else {
        ESP_LOGE(TAG, "Failed to initialize user LED on GPIO%d: %s", USER_LED_GPIO, esp_err_to_name(ret));
    }
}

static void user_led_set(bool on) {
    if (s_user_led_initialized) {
        gpio_set_level(USER_LED_GPIO, on ? 1 : 0);
    }
}

static void user_led_flash_task(void *pvParameters) {
    (void)pvParameters;
    const TickType_t flash_interval = pdMS_TO_TICKS(500);  // 500ms on/off
    
    while (1) {
        if (s_user_led_flash_enabled) {
            user_led_set(true);
            vTaskDelay(flash_interval);
            user_led_set(false);
            vTaskDelay(flash_interval);
        } else {
            // If flashing disabled, keep LED on and exit task
            user_led_set(true);
            vTaskDelete(NULL);
            return;
        }
    }
}

static void user_led_start_flash(void) {
    if (!s_user_led_initialized) {
        return;
    }
    
    if (s_user_led_task_handle == NULL) {
        s_user_led_flash_enabled = true;
        BaseType_t ret = xTaskCreate(
            user_led_flash_task,
            "user_led_flash",
            2048,
            NULL,
            1,  // Low priority
            &s_user_led_task_handle
        );
        if (ret == pdPASS) {
            ESP_LOGI(TAG, "User LED: Started blinking (normal operation)");
        } else {
            ESP_LOGE(TAG, "Failed to create user LED flash task");
            s_user_led_flash_enabled = false;
        }
    }
}

static void user_led_stop_flash(void) {
    if (s_user_led_task_handle != NULL) {
        s_user_led_flash_enabled = false;
        // Wait a bit for the task to exit cleanly
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_user_led_task_handle != NULL) {
            s_user_led_task_handle = NULL;
            ESP_LOGI(TAG, "User LED: Stopped blinking (going solid for ACD conflict)");
        }
    }
}


