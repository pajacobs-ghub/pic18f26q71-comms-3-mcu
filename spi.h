// spi.h
// PJ, 2026-03-11

#ifndef MY_SPI
#define MY_SPI
#include <stdint.h>

void spi1_init(uint8_t upin, uint8_t ckp, uint8_t cke, uint8_t smp);
void spi1_close(void);
uint8_t spi1_exchange(uint8_t n, uint8_t* buf);

#endif
