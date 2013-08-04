
#include "ds1820.h"
#include "onewire.h"
#include "delay.h"

uint8_t owi_addr_[MAX_OWI_DEVICES][8];
uint8_t num_owi_dev_found_;

void ds1820_discover(void)
{
    uint8_t d=0;
    num_owi_dev_found_ = 0;
    if (! owi_reset()) //if no devices on bus, return now
        return;
    owi_reset_search();
    delay_ms(250);

    //Search only for DS1820 temp sensors
    owi_target_search(DS1820_FAMILY_ID);

    while ( owi_search(owi_addr_[ d ]))
    {
        d++;
        if ( d >= MAX_OWI_DEVICES)
            break;
    }
    num_owi_dev_found_ = d;
}

void ds1820_set_resolution(uint8_t d, uint8_t bits)
{
    uint8_t resolution;
    if (d >= num_owi_dev_found_ || bits < 9 || bits > 12)
        return;

    owi_reset();
    owi_select(owi_addr_[d]);

    owi_write(DS1820_WRITE_SCRATCHPAD, 0);
    owi_write(0xFF, 0);
    owi_write(0x7F, 0);

    resolution = (bits - 9) << 5;
    owi_write(resolution | 0x1F, 0);
}


void ds1820_start_measuring(uint8_t d)
{
    owi_reset();
    owi_select(owi_addr_[d]);
    owi_write(DS1820_START_CONVERSION, 1);
}

uint8_t ds1820_get_conversion_time_10ms(uint8_t bits)
{
    switch (bits)
    {
        case 9: return DS1820_TCONV_MS_9BITS / 10;
        case 10: return DS1820_TCONV_MS_10BITS / 10;
        case 11: return DS1820_TCONV_MS_11BITS / 10;
        default: return DS1820_TCONV_MS_12BITS / 10;
    }
}

int16_t ds1820_read_temperature(uint8_t d)
{
    uint8_t data[9];
    int16_t raw = 0;
    uint8_t type_s=0;
    uint8_t cfg, i;

    owi_reset();
    owi_select(owi_addr_[d]);
    owi_write(DS1820_READ_SCRATCHPAD, 0);         // Read Scratchpad
    for (i = 0; i < 9; i++) {          // we need 9 bytes
        data[i] = owi_read();
    }
    #if (ONEWIRE_CRC == 1)
    uint8_t crc_result = owi_crc8(data,8);
    if (crc_result != data[8])
        return -1;
    #endif

    // the first ROM byte indicates which chip
    switch (owi_addr_[d][0])
    {
        case 0x10:
            type_s = 1;
            break;
        case 0x28:
        case 0x22:
            type_s = 0;
            break;
        default:
            break;
    }

    // Convert the data to actual temperature
    // because the result is a 16 bit signed integer, it should
    // be stored to an "int16_t" type, which is always 16 bits
    // even when compiled on a 32 bit processor.
    raw = (data[1] << 8) | data[0];
    if (type_s)
    {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
            // "count remain" gives full 12 bit resolution
            raw = (raw & 0xFFF0) + 12 - data[6];
        }
    } else {
        cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == DS1820_RESOLUTION_9BITS)
            raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == DS1820_RESOLUTION_10BITS)
            raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == DS1820_RESOLUTION_11BITS)
            raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    }

    //~ celsius = raw / 16;
    return raw / 4;
}
