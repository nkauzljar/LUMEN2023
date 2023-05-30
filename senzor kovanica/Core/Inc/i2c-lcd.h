#include "stm32f4xx_hal.h"

void lcd_init (void);   // initialize lcd

void lcd_send_cmd (char cmd);  // send command to the lcd

void lcd_send_data (char data);  // send data to the lcd

void lcd_send_string (char *str);  // send string to the lcd

void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);

void lcd_clear (void);	//obrisi ekran

void display_total_init(void);	//napisi "Total"

void display_total_update(float total);	//napisi vrijednost uz total

void display_bottom(char *str);		//ispisi string na donji redak

void clear_bottom(void);		//obrisi samo donji redak

void display_loading(void);		//loading animacija

