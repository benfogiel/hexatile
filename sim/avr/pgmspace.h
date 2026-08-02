#pragma once
#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t*)(addr))
