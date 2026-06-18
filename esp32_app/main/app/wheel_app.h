#ifndef WHEEL_APP_H
#define WHEEL_APP_H

#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t wheel_app_init(void);
void wheel_app_tick(void);
esp_err_t wheel_app_status_json(char *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* WHEEL_APP_H */
