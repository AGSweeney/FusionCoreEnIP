#include "acd_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "lwip/tcpip.h"
#include "lwip/netifapi.h"
#include "lwip/timeouts.h"
#include "lwip/etharp.h"
#include "lwip/inet.h"
#include "esp_netif_ip_addr.h"
#include "sdkconfig.h"
#include "esp_netif_net_stack.h"
#include "ciptcpipinterface.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "acd_manager";

#if LWIP_IPV4 && LWIP_ACD

static struct acd s_static_ip_acd;
static bool s_acd_registered = false;
static SemaphoreHandle_t s_acd_sem = NULL;
static SemaphoreHandle_t s_acd_registration_sem = NULL;
static acd_callback_enum_t s_acd_last_state = ACD_IP_OK;
static bool s_acd_callback_received = false;
static bool s_acd_probe_pending = false;
static esp_netif_ip_info_t s_pending_static_ip_cfg = {0};
static acd_manager_status_t s_acd_status = ACD_MANAGER_STATUS_IDLE;
static acd_manager_ip_assignment_cb_t s_ip_assignment_cb = NULL;
static void (*s_led_start_flash)(void) = NULL;
static void (*s_led_stop_flash)(void) = NULL;
static void (*s_led_set)(bool on) = NULL;
static void (*s_configure_dns)(esp_netif_t *netif) = NULL;

#if CONFIG_OPENER_ACD_RETRY_ENABLED
static TimerHandle_t s_acd_retry_timer = NULL;
static int s_acd_retry_count = 0;
static esp_netif_t *s_acd_retry_netif = NULL;
static struct netif *s_acd_retry_lwip_netif = NULL;
static void tcpip_acd_retry_timer_callback(TimerHandle_t xTimer);
#endif

typedef struct {
    struct netif *netif;
    ip4_addr_t ip;
    err_t err;
} AcdStartContext;

typedef struct {
    struct netif *netif;
    ip4_addr_t ip;
    err_t err;
} AcdStartProbeContext;

static bool netif_has_valid_hwaddr(struct netif *netif) {
    if (netif == NULL) {
        return false;
    }
    if (netif->hwaddr_len != ETH_HWADDR_LEN) {
        return false;
    }
    for (int i = 0; i < ETH_HWADDR_LEN; ++i) {
        if (netif->hwaddr[i] != 0) {
            return true;
        }
    }
    return false;
}

static void acd_start_probe_cb(void *arg) {
    AcdStartProbeContext *ctx = (AcdStartProbeContext *)arg;
    if (ctx == NULL || ctx->netif == NULL) {
        ESP_LOGE(TAG, "acd_start_probe_cb: Invalid context");
        if (ctx) free(ctx);
        return;
    }
    ESP_LOGI(TAG, "acd_start_probe_cb: Calling acd_start() for IP " IPSTR " on netif %p", 
             IP2STR(&ctx->ip), ctx->netif);
    ctx->err = acd_start(ctx->netif, &s_static_ip_acd, ctx->ip);
    ESP_LOGI(TAG, "acd_start_probe_cb: acd_start() returned err=%d", (int)ctx->err);
    free(ctx);
}

#if CONFIG_OPENER_ACD_RETRY_ENABLED
static void retry_callback(void *arg) {
    (void)arg;
    if (s_acd_retry_netif != NULL && s_acd_retry_lwip_netif != NULL) {
        ESP_LOGI(TAG, "ACD retry timer expired - restarting ACD probe sequence (attempt %d)",
                 s_acd_retry_count + 1);
        acd_manager_start_probe(s_acd_retry_netif, s_acd_retry_lwip_netif);
    }
}
#endif

static void tcpip_acd_conflict_callback(struct netif *netif, acd_callback_enum_t state) {
    ESP_LOGI(TAG, "ACD callback received: state=%d (0=IP_OK, 1=RESTART_CLIENT, 2=DECLINE)", (int)state);
    s_acd_last_state = state;
    s_acd_callback_received = true;
    
    extern CipTcpIpObject g_tcpip;
    
    switch (state) {
        case ACD_IP_OK:
            g_tcpip.status &= ~(kTcpipStatusAcdStatus | kTcpipStatusAcdFault);
            CipTcpIpSetLastAcdActivity(1);
            s_acd_status = ACD_MANAGER_STATUS_IP_OK;
            if (s_led_start_flash) {
                s_led_start_flash();
            }
            ESP_LOGI(TAG, "ACD: IP OK - no conflict detected, entering ongoing defense phase");
#if CONFIG_OPENER_ACD_RETRY_ENABLED
            s_acd_retry_count = 0;
            if (s_acd_retry_timer != NULL) {
                xTimerStop(s_acd_retry_timer, portMAX_DELAY);
            }
#endif
            if (s_acd_probe_pending && netif != NULL) {
                esp_netif_t *esp_netif = esp_netif_get_handle_from_netif_impl(netif);
                if (esp_netif != NULL && s_pending_static_ip_cfg.ip.addr != 0) {
                    ESP_LOGI(TAG, "Legacy ACD: Assigning IP " IPSTR " after callback confirmation", IP2STR(&s_pending_static_ip_cfg.ip));
                    esp_netif_set_ip_info(esp_netif, &s_pending_static_ip_cfg);
                    if (s_configure_dns) {
                        s_configure_dns(esp_netif);
                    }
                    if (s_ip_assignment_cb) {
                        s_ip_assignment_cb(esp_netif, &s_pending_static_ip_cfg);
                    }
                    s_acd_probe_pending = false;
                }
            }
            break;
        case ACD_DECLINE:
        case ACD_RESTART_CLIENT:
            g_tcpip.status |= kTcpipStatusAcdStatus;
            g_tcpip.status |= kTcpipStatusAcdFault;
            CipTcpIpSetLastAcdActivity(3);
            s_acd_status = ACD_MANAGER_STATUS_CONFLICT;
            if (s_led_stop_flash) {
                s_led_stop_flash();
            }
            if (s_led_set) {
                s_led_set(true);
            }
            ESP_LOGW(TAG, "ACD: Conflict detected (state=%d) - LED set to solid", (int)state);
#if CONFIG_OPENER_ACD_RETRY_ENABLED
            if (netif != NULL) {
                esp_netif_t *esp_netif = esp_netif_get_handle_from_netif_impl(netif);
                if (esp_netif != NULL) {
                    if (CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS == 0 || 
                        s_acd_retry_count < CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS) {
                        ESP_LOGW(TAG, "ACD: Scheduling retry (attempt %d/%d) after %dms",
                                 s_acd_retry_count + 1,
                                 CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS == 0 ? 999 : CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS,
                                 CONFIG_OPENER_ACD_RETRY_DELAY_MS);
                        s_acd_retry_count++;
                        s_acd_retry_netif = esp_netif;
                        s_acd_retry_lwip_netif = netif;
                        
                        esp_netif_ip_info_t zero_ip = {0};
                        esp_netif_set_ip_info(esp_netif, &zero_ip);
                        
                        if (s_acd_registered) {
                            acd_stop(&s_static_ip_acd);
                            s_acd_registered = false;
                        }
                        
                        if (s_acd_retry_timer == NULL) {
                            s_acd_retry_timer = xTimerCreate(
                                "acd_retry",
                                pdMS_TO_TICKS(CONFIG_OPENER_ACD_RETRY_DELAY_MS),
                                pdFALSE,
                                NULL,
                                tcpip_acd_retry_timer_callback
                            );
                        }
                        
                        if (s_acd_retry_timer != NULL) {
                            xTimerChangePeriod(s_acd_retry_timer, 
                                             pdMS_TO_TICKS(CONFIG_OPENER_ACD_RETRY_DELAY_MS),
                                             portMAX_DELAY);
                            xTimerStart(s_acd_retry_timer, portMAX_DELAY);
                        }
                    } else {
                        ESP_LOGE(TAG, "ACD: Max retry attempts (%d) reached - giving up",
                                 CONFIG_OPENER_ACD_RETRY_MAX_ATTEMPTS);
                    }
                }
            }
#endif
            break;
        default:
            g_tcpip.status |= kTcpipStatusAcdStatus;
            g_tcpip.status |= kTcpipStatusAcdFault;
            break;
    }
    if (s_acd_sem != NULL) {
        xSemaphoreGive(s_acd_sem);
    }
}

static void tcpip_acd_start_cb(void *arg) {
    ESP_LOGI(TAG, "tcpip_acd_start_cb: CALLBACK EXECUTING - arg=%p", arg);
    AcdStartContext *ctx = (AcdStartContext *)arg;
    if (ctx == NULL) {
        ESP_LOGE(TAG, "tcpip_acd_start_cb: NULL context");
        if (s_acd_registration_sem != NULL) {
            xSemaphoreGive(s_acd_registration_sem);
        }
        return;
    }
    ESP_LOGI(TAG, "tcpip_acd_start_cb: Context valid - netif=%p, ip=" IPSTR, 
             ctx->netif, IP2STR(&ctx->ip));
    ctx->err = ERR_OK;
    
    if (ctx->netif == NULL) {
        ESP_LOGD(TAG, "tcpip_acd_start_cb: NULL netif - ACD probe cancelled");
        ctx->err = ERR_IF;
        free(ctx);
        return;
    }
    
    bool probe_was_pending = s_acd_probe_pending;
    
    if (!s_acd_registered) {
        ctx->netif->acd_list = NULL;
        memset(&s_static_ip_acd, 0, sizeof(s_static_ip_acd));
        err_t add_err = acd_add(ctx->netif, &s_static_ip_acd, tcpip_acd_conflict_callback);
        if (add_err == ERR_OK) {
            s_acd_registered = true;
            ESP_LOGD(TAG, "tcpip_acd_start_cb: ACD client registered");
        } else {
            ESP_LOGE(TAG, "tcpip_acd_start_cb: acd_add() failed with err=%d", (int)add_err);
            ctx->err = ERR_IF;
            if (s_acd_registration_sem != NULL) {
                xSemaphoreGive(s_acd_registration_sem);
            }
            free(ctx);
            return;
        }
    }
    
    if (s_acd_registration_sem != NULL) {
        xSemaphoreGive(s_acd_registration_sem);
    }
    
    if (!probe_was_pending) {
        acd_stop(&s_static_ip_acd);
        s_static_ip_acd.state = ACD_STATE_ONGOING;
        s_static_ip_acd.ipaddr = ctx->ip;
        s_static_ip_acd.sent_num = 0;
        s_static_ip_acd.lastconflict = 0;
        s_static_ip_acd.num_conflicts = 0;
        
        acd_add(ctx->netif, &s_static_ip_acd, tcpip_acd_conflict_callback);
        
        CipTcpIpSetLastAcdActivity(1);
        
#ifdef CONFIG_OPENER_ACD_PERIODIC_DEFEND_INTERVAL_MS
        if (CONFIG_OPENER_ACD_PERIODIC_DEFEND_INTERVAL_MS > 0) {
            const uint16_t timer_interval_ms = 100;
            s_static_ip_acd.ttw = (uint16_t)((CONFIG_OPENER_ACD_PERIODIC_DEFEND_INTERVAL_MS + timer_interval_ms - 1) / timer_interval_ms);
        } else {
            s_static_ip_acd.ttw = 0;
        }
#else
        s_static_ip_acd.ttw = 100;
#endif
    }
    ctx->err = ERR_OK;
    free(ctx);
}

static void tcpip_acd_stop_cb(void *arg) {
    (void)arg;
    acd_stop(&s_static_ip_acd);
}

static bool tcpip_perform_acd(struct netif *netif, const ip4_addr_t *ip) {
    extern CipTcpIpObject g_tcpip;
    
    if (!g_tcpip.select_acd) {
        g_tcpip.status &= ~(kTcpipStatusAcdStatus | kTcpipStatusAcdFault);
        CipTcpIpSetLastAcdActivity(0);
        return true;
    }

    if (netif == NULL) {
        ESP_LOGW(TAG, "ACD requested but no netif available");
        g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
        CipTcpIpSetLastAcdActivity(3);
        return false;
    }

    if (s_acd_sem == NULL) {
        s_acd_sem = xSemaphoreCreateBinary();
        if (s_acd_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create ACD semaphore");
            g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
            CipTcpIpSetLastAcdActivity(3);
            return false;
        }
    }

    while (xSemaphoreTake(s_acd_sem, 0) == pdTRUE) {
    }

    if (!s_acd_probe_pending) {
        ESP_LOGD(TAG, "tcpip_perform_acd: ACD probe no longer pending - skipping");
        return true;
    }

    s_acd_callback_received = false;
    s_acd_last_state = ACD_IP_OK;
    CipTcpIpSetLastAcdActivity(2);
    s_acd_status = ACD_MANAGER_STATUS_PROBING;

    if (netif == NULL) {
        ESP_LOGW(TAG, "tcpip_perform_acd: netif became NULL - ACD cancelled");
        return true;
    }

    AcdStartContext *ctx = (AcdStartContext *)malloc(sizeof(AcdStartContext));
    if (ctx == NULL) {
        ESP_LOGE(TAG, "tcpip_perform_acd: Failed to allocate ACD context");
        g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
        CipTcpIpSetLastAcdActivity(3);
        return false;
    }
    
    ctx->netif = netif;
    ctx->ip = *ip;
    ctx->err = ERR_OK;

    ESP_LOGD(TAG, "tcpip_perform_acd: Registering ACD client for IP " IPSTR, IP2STR(ip));
    
    if (s_acd_registration_sem == NULL) {
        s_acd_registration_sem = xSemaphoreCreateBinary();
        if (s_acd_registration_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create ACD registration semaphore");
            free(ctx);
            g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
            CipTcpIpSetLastAcdActivity(3);
            return false;
        }
    }
    
    while (xSemaphoreTake(s_acd_registration_sem, 0) == pdTRUE) {
    }
    
    if (!s_acd_registered) {
        ESP_LOGD(TAG, "tcpip_perform_acd: Attempting direct ACD registration");
        netif->acd_list = NULL;
        memset(&s_static_ip_acd, 0, sizeof(s_static_ip_acd));
        err_t add_err = acd_add(netif, &s_static_ip_acd, tcpip_acd_conflict_callback);
        if (add_err == ERR_OK) {
            s_acd_registered = true;
            ESP_LOGD(TAG, "tcpip_perform_acd: Direct ACD registration succeeded");
            free(ctx);
        } else {
            ESP_LOGW(TAG, "tcpip_perform_acd: Direct registration failed (err=%d), trying callback", (int)add_err);
        }
    }
    
    if (!s_acd_registered) {
        ESP_LOGD(TAG, "tcpip_perform_acd: Registering ACD client via callback");
        err_t callback_err = tcpip_callback_with_block(tcpip_acd_start_cb, ctx, 1);
        
        if (callback_err != ERR_OK) {
            ESP_LOGE(TAG, "Failed to register ACD client (callback_err=%d)", (int)callback_err);
            g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
            CipTcpIpSetLastAcdActivity(3);
            return false;
        }
        
        TickType_t registration_timeout = pdMS_TO_TICKS(500);
        if (xSemaphoreTake(s_acd_registration_sem, registration_timeout) != pdTRUE) {
            ESP_LOGW(TAG, "ACD registration callback timed out - trying direct registration as fallback");
            if (!s_acd_registered) {
                netif->acd_list = NULL;
                memset(&s_static_ip_acd, 0, sizeof(s_static_ip_acd));
                err_t add_err = acd_add(netif, &s_static_ip_acd, tcpip_acd_conflict_callback);
                if (add_err == ERR_OK) {
                    s_acd_registered = true;
                    ESP_LOGI(TAG, "tcpip_perform_acd: Fallback direct registration succeeded");
                } else {
                    ESP_LOGE(TAG, "ACD registration failed via both callback and direct methods");
                    g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
                    CipTcpIpSetLastAcdActivity(3);
                    return false;
                }
            }
        }
        
        if (!s_acd_registered) {
            ESP_LOGE(TAG, "ACD registration callback completed but registration failed");
            g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
            CipTcpIpSetLastAcdActivity(3);
            return false;
        }
    }
    
    if (s_acd_probe_pending && s_acd_registered) {
        ESP_LOGD(TAG, "tcpip_perform_acd: Starting ACD probe for IP " IPSTR, IP2STR(ip));
        err_t acd_start_err = acd_start(netif, &s_static_ip_acd, *ip);
        if (acd_start_err == ERR_OK) {
            ESP_LOGD(TAG, "tcpip_perform_acd: ACD probe started");
        } else {
            ESP_LOGE(TAG, "tcpip_perform_acd: acd_start() failed with err=%d", (int)acd_start_err);
            AcdStartProbeContext *probe_ctx = (AcdStartProbeContext *)malloc(sizeof(AcdStartProbeContext));
            if (probe_ctx == NULL) {
                ESP_LOGE(TAG, "Failed to allocate probe context");
                g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
                CipTcpIpSetLastAcdActivity(3);
                return false;
            }
            
            probe_ctx->netif = netif;
            probe_ctx->ip = *ip;
            probe_ctx->err = ERR_OK;
            
            err_t callback_err = tcpip_callback_with_block(acd_start_probe_cb, probe_ctx, 1);
            if (callback_err != ERR_OK) {
                ESP_LOGE(TAG, "tcpip_perform_acd: acd_start() callback failed (callback_err=%d)", 
                         (int)callback_err);
                free(probe_ctx);
                g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
                CipTcpIpSetLastAcdActivity(3);
                return false;
            }
            ESP_LOGI(TAG, "tcpip_perform_acd: ACD probe started via callback");
        }
    } else {
        ESP_LOGW(TAG, "tcpip_perform_acd: Cannot start ACD probe - probe_pending=%d, registered=%d", 
                 s_acd_probe_pending, s_acd_registered);
    }

    TickType_t wait_ticks = pdMS_TO_TICKS(2000);

    ESP_LOGD(TAG, "Waiting for ACD probe sequence to complete (timeout: 2000ms)...");
    if (xSemaphoreTake(s_acd_sem, wait_ticks) == pdTRUE) {
        ESP_LOGI(TAG, "ACD completed with state=%d", (int)s_acd_last_state);
        if (s_acd_last_state == ACD_IP_OK) {
            CipTcpIpSetLastAcdActivity(0);
            return true;
        }
        if (s_acd_last_state == ACD_DECLINE || s_acd_last_state == ACD_RESTART_CLIENT) {
            ESP_LOGE(TAG, "ACD detected conflict (state=%d) - IP should not be assigned", (int)s_acd_last_state);
            CipTcpIpSetLastAcdActivity(3);
            return false;
        }
    } else if (s_acd_callback_received && s_acd_last_state == ACD_IP_OK) {
        ESP_LOGI(TAG, "ACD callback received (state=IP_OK) - semaphore timeout was harmless, continuing with IP assignment");
        CipTcpIpSetLastAcdActivity(0);
        return true;
    }

    if (s_acd_last_state == ACD_RESTART_CLIENT || s_acd_last_state == ACD_DECLINE) {
        ESP_LOGE(TAG, "ACD conflict detected during probe phase (state=%d) - IP should not be assigned", (int)s_acd_last_state);
        CipTcpIpSetLastAcdActivity(3);
        tcpip_callback_with_block(tcpip_acd_stop_cb, NULL, 1);
        return false;
    }
    
    ESP_LOGI(TAG, "ACD probe wait timed out (state=%d) - callback not received yet (probe sequence still running)", (int)s_acd_last_state);
    ESP_LOGI(TAG, "Note: ACD probe sequence can take 6-10 seconds (probes + announcements). Waiting for callback...");
    ESP_LOGI(TAG, "IP assignment will occur when ACD_IP_OK callback is received.");
    return true;
}

#if CONFIG_OPENER_ACD_RETRY_ENABLED
static void tcpip_acd_retry_timer_callback(TimerHandle_t xTimer) {
    (void)xTimer;
    
    if (s_acd_retry_netif == NULL || s_acd_retry_lwip_netif == NULL) {
        return;
    }
    
    s_acd_probe_pending = true;
    
    err_t err = tcpip_callback_with_block(retry_callback, NULL, 0);
    if (err != ERR_OK) {
        acd_manager_start_probe(s_acd_retry_netif, s_acd_retry_lwip_netif);
    }
}
#endif

static void tcpip_retry_acd_deferred(void *arg) {
    esp_netif_t *netif = (esp_netif_t *)arg;
    if (netif == NULL) {
        ESP_LOGW(TAG, "tcpip_retry_acd_deferred: NULL netif - retry timer fired after cleanup");
        return;
    }
    
    if (!s_acd_probe_pending) {
        ESP_LOGD(TAG, "tcpip_retry_acd_deferred: ACD probe no longer pending (IP likely assigned) - skipping retry");
        return;
    }
    
    if (s_acd_registered) {
        ESP_LOGD(TAG, "tcpip_retry_acd_deferred: ACD already running (registered=%d) - skipping retry", s_acd_registered);
        return;
    }
    
    struct netif *lwip_netif = (struct netif *)esp_netif_get_netif_impl(netif);
    if (lwip_netif != NULL) {
        ESP_LOGI(TAG, "tcpip_retry_acd_deferred: Retrying ACD start");
        acd_manager_start_probe(netif, lwip_netif);
    } else {
        ESP_LOGW(TAG, "tcpip_retry_acd_deferred: NULL lwip_netif - netif may not be fully initialized yet");
    }
}

esp_err_t acd_manager_init(void) {
    if (s_acd_sem == NULL) {
        s_acd_sem = xSemaphoreCreateBinary();
        if (s_acd_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create ACD semaphore");
            return ESP_ERR_NO_MEM;
        }
    }
    
    if (s_acd_registration_sem == NULL) {
        s_acd_registration_sem = xSemaphoreCreateBinary();
        if (s_acd_registration_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create ACD registration semaphore");
            return ESP_ERR_NO_MEM;
        }
    }
    
    s_acd_registered = false;
    s_acd_probe_pending = false;
    s_acd_last_state = ACD_IP_OK;
    s_acd_callback_received = false;
    s_acd_status = ACD_MANAGER_STATUS_IDLE;
    memset(&s_pending_static_ip_cfg, 0, sizeof(s_pending_static_ip_cfg));
    
    ESP_LOGI(TAG, "ACD Manager initialized");
    return ESP_OK;
}

bool acd_manager_is_probe_pending(void) {
    return s_acd_probe_pending;
}

void acd_manager_set_ip_config(const esp_netif_ip_info_t *ip_info) {
    if (ip_info != NULL) {
        s_pending_static_ip_cfg = *ip_info;
        s_acd_probe_pending = true;
    }
}

bool acd_manager_start_probe(esp_netif_t *netif, struct netif *lwip_netif) {
    extern CipTcpIpObject g_tcpip;
    
    ESP_LOGI(TAG, "acd_manager_start_probe: called - probe_pending=%d, select_acd=%d, netif=%p, lwip_netif=%p", 
             s_acd_probe_pending, g_tcpip.select_acd ? 1 : 0, netif, lwip_netif);
    
    if (!g_tcpip.select_acd) {
        ESP_LOGI(TAG, "acd_manager_start_probe: ACD disabled (select_acd=0) - skipping ACD probe");
        s_acd_probe_pending = false;
        return true;
    }
    
    if (!s_acd_probe_pending || netif == NULL || lwip_netif == NULL) {
        ESP_LOGW(TAG, "acd_manager_start_probe: Skipping - probe_pending=%d, netif=%p, lwip_netif=%p", 
                 s_acd_probe_pending, netif, lwip_netif);
        return false;
    }
    
    if (!netif_has_valid_hwaddr(lwip_netif)) {
        ESP_LOGI(TAG, "ACD deferred until MAC address is available");
        return false;
    }
    
    if (!netif_is_link_up(lwip_netif)) {
        ESP_LOGI(TAG, "ACD deferred until link is up (link status: %d) - will retry", netif_is_link_up(lwip_netif));
        sys_timeout(100, tcpip_retry_acd_deferred, netif);
        return false;
    }
    
    ESP_LOGI(TAG, "acd_manager_start_probe: All conditions met, starting ACD...");

    ESP_LOGI(TAG, "Using legacy ACD mode - ACD runs before IP assignment");
    ip4_addr_t desired_ip = { .addr = s_pending_static_ip_cfg.ip.addr };
    CipTcpIpSetLastAcdActivity(2);
    ESP_LOGD(TAG, "Legacy ACD: Starting probe sequence for IP " IPSTR " BEFORE IP assignment", IP2STR(&desired_ip));
    
    (void)tcpip_perform_acd(lwip_netif, &desired_ip);
    
    if (s_acd_callback_received && (s_acd_last_state == ACD_DECLINE || s_acd_last_state == ACD_RESTART_CLIENT)) {
        ESP_LOGE(TAG, "ACD conflict detected for " IPSTR " - NOT assigning IP", IP2STR(&desired_ip));
        ESP_LOGW(TAG, "IP assignment cancelled due to ACD conflict");
        g_tcpip.status |= kTcpipStatusAcdStatus | kTcpipStatusAcdFault;
        CipTcpIpSetLastAcdActivity(3);
        s_acd_probe_pending = false;
        tcpip_callback_with_block(tcpip_acd_stop_cb, NULL, 1);
        return false;
    }
    
    if (s_acd_callback_received && s_acd_last_state == ACD_IP_OK) {
        ESP_LOGI(TAG, "Legacy ACD: No conflict detected - assigning IP " IPSTR, IP2STR(&desired_ip));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &s_pending_static_ip_cfg));
        if (s_configure_dns) {
            s_configure_dns(netif);
        }
        if (s_ip_assignment_cb) {
            s_ip_assignment_cb(netif, &s_pending_static_ip_cfg);
        }
        s_acd_probe_pending = false;
    } else {
        ESP_LOGI(TAG, "Legacy ACD: Probe sequence in progress - IP will be assigned when callback fires");
    }
    
    CipTcpIpSetLastAcdActivity(1);
    ESP_LOGD(TAG, "Legacy ACD: ACD is in ONGOING state (callback fired after announce phase), periodic defense active");
    
    return true;
}

acd_manager_status_t acd_manager_get_status(void) {
    return s_acd_status;
}

void acd_manager_register_ip_assignment_callback(acd_manager_ip_assignment_cb_t callback) {
    s_ip_assignment_cb = callback;
}

void acd_manager_set_led_control_callback(void (*led_start_flash)(void), void (*led_stop_flash)(void), void (*led_set)(bool on)) {
    s_led_start_flash = led_start_flash;
    s_led_stop_flash = led_stop_flash;
    s_led_set = led_set;
}

void acd_manager_set_dns_config_callback(void (*configure_dns)(esp_netif_t *netif)) {
    s_configure_dns = configure_dns;
}

void acd_manager_stop(void) {
    tcpip_callback_with_block(tcpip_acd_stop_cb, NULL, 1);
}

#else

esp_err_t acd_manager_init(void) {
    ESP_LOGI(TAG, "ACD Manager: ACD not available (LWIP_IPV4 or LWIP_ACD disabled)");
    return ESP_OK;
}

bool acd_manager_is_probe_pending(void) {
    return false;
}

void acd_manager_set_ip_config(const esp_netif_ip_info_t *ip_info) {
    (void)ip_info;
}

bool acd_manager_start_probe(esp_netif_t *netif, struct netif *lwip_netif) {
    (void)netif;
    (void)lwip_netif;
    extern CipTcpIpObject g_tcpip;
    if (g_tcpip.select_acd) {
        ESP_LOGW(TAG, "ACD requested but not supported by lwIP configuration");
    }
    g_tcpip.status &= ~(kTcpipStatusAcdStatus | kTcpipStatusAcdFault);
    return true;
}

acd_manager_status_t acd_manager_get_status(void) {
    return ACD_MANAGER_STATUS_IDLE;
}

void acd_manager_register_ip_assignment_callback(acd_manager_ip_assignment_cb_t callback) {
    (void)callback;
}

void acd_manager_set_led_control_callback(void (*led_start_flash)(void), void (*led_stop_flash)(void), void (*led_set)(bool on)) {
    (void)led_start_flash;
    (void)led_stop_flash;
    (void)led_set;
}

void acd_manager_set_dns_config_callback(void (*configure_dns)(esp_netif_t *netif)) {
    (void)configure_dns;
}

void acd_manager_stop(void) {
}

#endif

