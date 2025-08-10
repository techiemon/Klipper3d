// max6675.h
#ifndef __MAX6675_H
#define __MAX6675_H
#include <stdint.h>
#include "spicmds.h"

void max6675_init(uint8_t cs_pin, struct spi_config spi);
uint16_t max6675_read_temp(void);
void max6675_setup_commands(void);

#endif // __MAX6675_H
