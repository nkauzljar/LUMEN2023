#include "flashFunction.h"
// inicijalizirati sa  HAL_Init();
void coin_init(){
	
HAL_Init();
uint32_t Address = FLASH_MEMORY_BEGIN;
uint8_t data = 8;
save_data(Address,data);
}

uint8_t read_data(uint32_t Address){

	__IO uint8_t read_data = *(__IO uint32_t *)Address;
	return (uint8_t)read_data;
}

void save_data(uint32_t Address,uint8_t data){

    HAL_FLASH_Unlock();

    // FLASH_Erase_Sector(FLASH_SECTOR_7,VOLTAGE_RANGE_1);
	//HAL_Delay(50);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,Address,(uint8_t)data);
	//HAL_Delay(50);
	HAL_FLASH_Lock();


}

void Write_coin(uint8_t *data_p, int broj){
int i;
uint32_t flash_address = FLASH_MEMORY_BEGIN + 1 + (broj * sizestruct);

  for ( i = 0; i < sizestruct; i++, data_p++, flash_address++ )
	 save_data(flash_address, *data_p);

}

void Read_coin(uint8_t *data_p, int broj){
	
int i;
uint32_t flash_address = FLASH_MEMORY_BEGIN + 1 + (broj * sizestruct);

 for ( i = 0; i < sizestruct ; i++, data_p++, flash_address++ )
      *data_p = read_data(flash_address);

}

 
