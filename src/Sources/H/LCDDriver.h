#ifndef _LCD_H_
#define _LCD_H_

#include"common.h"

#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1
#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
#define byte uint8
#define word uint16

void LCD_IO_Init(void);
void LCD_Init(void);
void LCD_CLS(void);
void LCD_CLS_ROW(byte x,byte y );
void LCD_P6x8Cha(byte x,byte y,byte ch);
void LCD_P6x8Str(byte x,byte y,byte ch[]);
void LCD_P6x8Num(unsigned char x,unsigned char y, float number);
void LCD_P8x16Cha(byte x,byte y,byte ch);
void LCD_P8x16Str(byte x,byte y,byte ch[]);
void LCD_P8x16Num(unsigned char x,unsigned char y, float number);
void LCD_PutPixel(byte x,byte y);
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
void LCD_Fill(byte dat);
void LCD_DLY_ms(word ms);
void LCD_Set_Pos(byte x, byte y);
void LCD_WrCmd(byte cmd);
void LCD_WrDat(byte data); 

#endif