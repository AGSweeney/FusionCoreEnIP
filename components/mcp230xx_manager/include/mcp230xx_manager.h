#ifndef MCP230XX_MANAGER_H
#define MCP230XX_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t mcp230xx_manager_init(void);

bool mcp230xx_manager_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif

