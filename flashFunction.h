#include "stm32f4xx_hal.h"
#define FLASH_MEMORY_BEGIN 0x08060000


void coin_init();
uint8_t read_data(uint32_t Address);
void save_data(uint32_t Address,uint8_t data);
void Write_coin(uint8_t *data_p, int broj);
void Read_coin(uint8_t *data_p, int broj);

int sizestruct = 27;