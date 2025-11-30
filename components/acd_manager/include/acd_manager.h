#ifndef ACD_MANAGER_H
#define ACD_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_netif.h"
#include "lwip/netif.h"
#include "lwip/ip4_addr.h"
#include "lwip/acd.h"
#include "ciptcpipinterface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ACD_MANAGER_STATUS_IDLE = 0,
    ACD_MANAGER_STATUS_PROBING = 1,
    ACD_MANAGER_STATUS_IP_OK = 2,
    ACD_MANAGER_STATUS_CONFLICT = 3,
} acd_manager_status_t;

typedef void (*acd_manager_ip_assignment_cb_t)(esp_netif_t *netif, const esp_netif_ip_info_t *ip_info);

esp_err_t acd_manager_init(void);

bool acd_manager_is_probe_pending(void);

void acd_manager_set_ip_config(const esp_netif_ip_info_t *ip_info);

bool acd_manager_start_probe(esp_netif_t *netif, struct netif *lwip_netif);

acd_manager_status_t acd_manager_get_status(void);

void acd_manager_register_ip_assignment_callback(acd_manager_ip_assignment_cb_t callback);

void acd_manager_set_led_control_callback(void (*led_start_flash)(void), void (*led_stop_flash)(void), void (*led_set)(bool on));

void acd_manager_set_dns_config_callback(void (*configure_dns)(esp_netif_t *netif));

void acd_manager_stop(void);

#ifdef __cplusplus
}
#endif

#endif

