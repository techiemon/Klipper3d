// max6675.c
// Driver for MAX6675 thermocouple on Klipper MCU
#include "basecmd.h"    // oid_alloc
#include "command.h"    // DECL_COMMAND, sendf
#include "sched.h"
#include "spicmds.h"    // spidev_transfer
#include "max6675.h"

struct max6675 {
    struct spidev_s *spi;
    uint16_t last_temp;
};

static struct max6675 max6675_inst;

void
max6675_init(struct spidev_s *spi)
{
    max6675_inst.spi = spi;
}

uint16_t
max6675_read_temp(void)
{
    uint8_t msg[2] = {0x00, 0x00};
    spidev_transfer(max6675_inst.spi, 1, sizeof(msg), msg);
    uint16_t value = ((uint16_t)msg[0] << 8) | msg[1];
    if (value & 0x04)
        return 0xFFFF; // Open thermocouple or fault
    return value >> 3; // 12-bit value (0.25C units)
}

void
command_max6675_read(uint32_t *args)
{
    uint16_t temp = max6675_read_temp();
    sendf("max6675_temp temp=%u\n", temp);
}

DECL_COMMAND(command_max6675_read, "max6675_read");

void
command_config_max6675(uint32_t *args)
{
    // args[0]: spi_oid
    struct spidev_s *spi = spidev_oid_lookup(args[0]);
    max6675_init(spi);
}

DECL_COMMAND(command_config_max6675, "config_max6675 spi_oid=%c");
