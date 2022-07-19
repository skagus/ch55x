#include <stdint.h>

uint8_t usb_get_cdc1(uint8_t* pDst, uint8_t nMaxSize);
uint8_t usb_get_cdc0(uint8_t* pDst, uint8_t nMaxSize);
uint8_t usb_send_cdc1(uint8_t* pSrc, uint8_t nSize);
uint8_t usb_send_cdc0(uint8_t* pSrc, uint8_t nSize);

void logic_init();
void logic_run();
