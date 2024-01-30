#ifndef __OLED_H
#define __OLED_H			

#include "main.h"


//=============   软件spi   ===========
#define OLED_RST_Clr() PBout(4)=0   //RST
#define OLED_RST_Set() PBout(4)=1   //RST

#define OLED_DC_Clr() PBout(3)=0    //DC
#define OLED_DC_Set() PBout(3)=1    //DC

#define OLED_SCL_Clr()  PBout(13)=0  //SCL
#define OLED_SCL_Set()  PBout(13)=1   //SCL

#define OLED_SDA_Clr()  PBout(15)=0   //SDA
#define OLED_SDA_Set()  PBout(15)=1   //SDA


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


extern void OLED_Refresh_Gram(void);	
extern void OLED_Display_On(void);
extern void OLED_Display_Off(void);	   				   		    
extern void OLED_Init(void);
extern void OLED_Clear(void);
extern void OLED_Clear2(void);
extern void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
extern uint8_t OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size,uint8_t mode);
extern void OLED_ShowNumber(uint8_t x,uint8_t y,int16_t num,uint8_t len,uint8_t size);
extern uint8_t OLED_ShowNumber2(uint8_t x, uint8_t y, int16_t num, uint8_t size);
extern uint8_t OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p, uint8_t size);	 
#endif  

