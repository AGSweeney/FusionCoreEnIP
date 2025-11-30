#ifndef GP8403_DAC_MANAGER_H
#define GP8403_DAC_MANAGER_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t gp8403_dac_manager_init(void);

bool gp8403_dac_manager_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif

