#ifndef __DS1820__
#define __DS1820__

#include	<stdint.h>

#define MAX_OWI_DEVICES 3

void ds1820_discover(void);
void ds1820_set_resolution(uint8_t d, uint8_t bits);
void ds1820_start_measuring(uint8_t d);
uint8_t ds1820_get_conversion_time_10ms(uint8_t bits);
int16_t ds1820_read_temperature(uint8_t d) ;

#endif