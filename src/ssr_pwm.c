// ssr_pwm.c
// PWM control for SSR on Klipper MCU (RP2040)
#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "board/gpio.h"

static uint8_t ssr_pwm_pin;
static uint32_t ssr_pwm_cycle_ticks = 0;
static uint16_t ssr_pwm_value = 0;

void
ssr_pwm_init(uint8_t pin, uint32_t cycle_ticks)
{
    ssr_pwm_pin = pin;
    ssr_pwm_cycle_ticks = cycle_ticks;
    ssr_pwm_value = 0;
    // Configure the pin for PWM
    gpio_pwm_setup(pin, cycle_ticks, 0);
}

void
ssr_pwm_set(uint16_t value)
{
    ssr_pwm_value = value;
    gpio_pwm_write(ssr_pwm_pin, value);
}

uint16_t
ssr_pwm_get(void)
{
    return ssr_pwm_value;
}

void
command_ssr_pwm_set(uint32_t *args)
{
    // args[0]: value
    ssr_pwm_set(args[0]);
    sendf("ssr_pwm_set value=%u\n", args[0]);
}

void
command_ssr_pwm_get(uint32_t *args)
{
    sendf("ssr_pwm_get value=%u\n", ssr_pwm_value);
}

DECL_COMMAND(command_ssr_pwm_set, "ssr_pwm_set value=%hu");
DECL_COMMAND(command_ssr_pwm_get, "ssr_pwm_get");
