// ssr_pwm.h
#ifndef __SSR_PWM_H
#define __SSR_PWM_H
#include <stdint.h>

void ssr_pwm_init(uint8_t pin, uint32_t cycle_ticks);
void ssr_pwm_set(uint16_t value);
uint16_t ssr_pwm_get(void);
void ssr_pwm_setup_commands(void);

#endif // __SSR_PWM_H
