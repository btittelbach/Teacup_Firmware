#ifndef __DS1820__
#define __DS1820__

#include	<stdint.h>

#define DS1820_FAMILY_ID                0x10
#define DS1820_START_CONVERSION         0x44
#define DS1820_READ_SCRATCHPAD          0xbe
#define DS1820_WRITE_SCRATCHPAD          0x4e
#define DS1820_ERROR                    -1000   // Return code. Outside temperature range.
#define DS1820_RESOLUTION_9BITS  0x00
#define DS1820_RESOLUTION_10BITS  0x20
#define DS1820_RESOLUTION_11BITS  0x40
#define DS1820_RESOLUTION_12BITS  0x60
// Rounded Up Conversion times + extra time, so we don't talk on onewire bus too often since that means disabling interrupts for a few microseconds
#define DS1820_TCONV_MS_9BITS  (100 + 200)
#define DS1820_TCONV_MS_10BITS  (190 + 310)
#define DS1820_TCONV_MS_11BITS  ( 390 + 310)
#define DS1820_TCONV_MS_12BITS  (800 + 100)


#define DS2890_FAMILY_ID                0x2c
#define DS2890_WRITE_CONTROL_REGISTER   0X55
#define DS2890_RELEASE_CODE             0x96
#define DS2890_WRITE_POSITION           0x0f


#define MAX_OWI_DEVICES 3

void ds1820_discover(void);
void ds1820_set_resolution(uint8_t d, uint8_t bits);
void ds1820_start_measuring(uint8_t d);
uint8_t ds1820_get_conversion_time_10ms(uint8_t bits);
int16_t ds1820_read_temperature(uint8_t d) ;

#endif