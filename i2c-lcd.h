#ifndef __I2C_LCD_H
#define __I2C_LCD_H

#include "stm32f1xx_hal.h"

void lcd_init (void);   // Kh?i t?o LCD
void lcd_send_cmd (char cmd);  // G?i l?nh
void lcd_send_data (char data);  // G?i d? li?u
void lcd_send_string (char *str);  // G?i chu?i
void lcd_put_cur(int row, int col); // Ð?t con tr?
void lcd_clear (void); // Xóa màn hình

#endif