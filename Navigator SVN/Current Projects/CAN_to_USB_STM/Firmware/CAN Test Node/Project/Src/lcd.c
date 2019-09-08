/*
 * lcd.c
 *
 *  Created on: Mar 11, 2019
 *      Author: rosem
 */

#include "main.h"
#include <stdio.h>


extern I2C_HandleTypeDef hi2c1;

/*Function to convert an 8-bit LCD command to the proper 4-bit command format*/
void LCD_command(uint8_t command)
{
    uint8_t high_nibble_enable_high = ((command & 0xF0)|0x0C);
    uint8_t high_nibble_enable_low =  ((command & 0xF0)|0x08);
    uint8_t low_nibble_enable_high =  (((command<<4) & 0xF0)|0x0C);
    uint8_t low_nibble_enable_low =   (((command<<4) & 0xF0)|0x08);
    uint8_t lcd_send_data[] = {high_nibble_enable_high, high_nibble_enable_low, low_nibble_enable_high, low_nibble_enable_low};
    HAL_I2C_Master_Transmit(&hi2c1,0x3F<<1,&lcd_send_data,4,100);
    HAL_Delay(10);
}

/*Function to convert a number to ascii*/
uint8_t hex_to_ascii(uint8_t hex)
{
    //function to convert hex to ascii
    uint8_t ascii;
    if (hex <= 9)
    {
        ascii = hex + 0x30;
    }
    else
    {
        ascii = hex + 0x37;
    }
    return ascii;
}

/*Function to convert an 8-bit LCD ASCII character to the proper 4-bit format*/
void LCD_char(uint8_t character)
{
    uint8_t high_nibble_enable_high = ((character & 0xF0)|0x0D);
    uint8_t high_nibble_enable_low =  ((character & 0xF0)|0x09);
    uint8_t low_nibble_enable_high =  (((character<<4) & 0xF0)|0x0D);
    uint8_t low_nibble_enable_low =   (((character<<4) & 0xF0)|0x09);
    uint8_t lcd_send_data[] = {high_nibble_enable_high, high_nibble_enable_low, low_nibble_enable_high, low_nibble_enable_low};
    HAL_I2C_Master_Transmit(&hi2c1,0x3F<<1,&lcd_send_data,4,100);
    HAL_Delay(10);
}

/*Function to clear LCD*/
void LCD_clear()
{
    LCD_command(0x01); //Clear Screen, Cursor Home
}

/*Function to move cursor to second lcd line*/
void LCD_line2()
{
    LCD_command(0xC0);
}

/*Function to move cursor home*/
void LCD_cursor_home()
{
    LCD_command(0x02);
}

/*Function to output a string to the LCD*/
void LCD_string(uint8_t *string)
{
    for (unsigned int i = 0; string[i] != 0x00; i++)
    {
        LCD_char(string[i]);
    }
}

void LCD_print_float(float vin, uint8_t voltage)
{
    LCD_line2();

    char testString[16];
    gcvt(vin, 6, testString);
    LCD_string(testString);

//    int digit1 = vin;
//    LCD_char(hex_to_ascii(digit1));
//    LCD_char('.');
//
//    float num2 = 10*(vin - digit1);
//    int digit2 = num2;
//    LCD_char(hex_to_ascii(digit2));
//
//    float num3 = 10*(num2 - digit2);
//    int digit3 = num3;
//    LCD_char(hex_to_ascii(digit3));

    if (voltage) {
    	LCD_string("V");
    }
    else {
    	LCD_string("A");
    }
}

/*Function to initialize LCD*/
void LCD_init(void)
{
    uint8_t lcd_init_data[] = {0x33,0x32,0x28,0x0F,0x01};
    for (unsigned int i = 0; i < sizeof(lcd_init_data); i++)
    {
        LCD_command(lcd_init_data[i]);
    }
    //uint16_t lcd_init_data[] = {0x3C,0x38, 0x3C,0x38, 0x3C,0x38, 0x2C,0x28, 0x2C,0x28, 0x8C,0x88, 0x0C,0x08, 0xFC,0xF8, 0x0C,0x08, 0x1C,0x18};
    //I2C_O2O_SendBytes(lcd_init_data, sizeof(lcd_init_data));
}

/*Function to find percent difference between two numbers*/
float percent_difference(float num1, float num2)
{
    return (float)( (  ((float)abs(num1-num2)) / ((float)(num1+num2))  ) *200);
}
