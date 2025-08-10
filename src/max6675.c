// max6675.c
// Driver for MAX6675 thermocouple on Klipper MCU (RP2040)
#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "spicmds.h"
#include "max6675.h"

struct max6675 {
    struct spi_config spi;
    uint8_t cs_pin;
    uint16_t last_temp;
};

static struct max6675 max6675_inst;

void
max6675_init(uint8_t cs_pin, struct spi_config spi)
{
    max6675_inst.cs_pin = cs_pin;
    max6675_inst.spi = spi;
}

uint16_t
max6675_read_temp(void)
{
    uint8_t tx[2] = {0, 0};
    uint8_t rx[2];
    spi_transfer(&max6675_inst.spi, max6675_inst.cs_pin, tx, rx, 2);
    uint16_t value = (rx[0] << 8) | rx[1];
    if (value & 0x4) return 0xFFFF; // Open thermocouple
    return value >> 3;
}

void
command_max6675_read(uint32_t *args)
{
    uint16_t temp = max6675_read_temp();
    sendf("max6675_temp temp=%u\n", temp);
}

void
max6675_setup_commands(void)
{
    register_command("max6675_read", command_max6675_read, "");
}
