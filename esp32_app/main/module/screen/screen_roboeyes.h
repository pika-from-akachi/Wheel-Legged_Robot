#ifndef SCREEN_ROBOEYES_H
#define SCREEN_ROBOEYES_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t screen_roboeyes_begin_expression(const char *expression);
esp_err_t screen_roboeyes_update(void);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_ROBOEYES_H */
