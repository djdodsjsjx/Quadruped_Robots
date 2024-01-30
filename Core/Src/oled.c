#include "oled.h"
#include "oledfont.h"  
#include <math.h>

//向OLED写入一个字节。
//dat:要写入的数据/命令
//cmd:数据/命令标志 0,表示命令;1,表示数据;
static void OLED_WR_Byte(uint8_t dat,uint8_t cmd) {	
	uint8_t i;			  
	if(cmd) OLED_DC_Set();
	else OLED_DC_Clr();		  
	for (i = 0; i < 8; i++) {			  
		OLED_SCL_Clr();
		if (dat & 0x80) OLED_SDA_Set();
		else OLED_SDA_Clr();
		OLED_SCL_Set();
		dat <<= 1;   
	}				 		  
	OLED_DC_Set();   	  
} 


//初始化OLED					    
extern void OLED_Init(void) { 	
	
	OLED_RST_Clr();
	HAL_Delay(100);
	OLED_RST_Set(); 

	OLED_WR_Byte(0xAE, OLED_CMD); //关闭显示
	OLED_WR_Byte(0xD5, OLED_CMD); //设置时钟分频因子,震荡频率
	OLED_WR_Byte(80, OLED_CMD);   //[3:0],分频因子;[7:4],震荡频率
	OLED_WR_Byte(0xA8, OLED_CMD); //设置驱动路数
	OLED_WR_Byte(0X3F, OLED_CMD); //默认0X3F(1/64) 
	OLED_WR_Byte(0xD3, OLED_CMD); //设置显示偏移
	OLED_WR_Byte(0X00, OLED_CMD); //默认为0

	OLED_WR_Byte(0x40, OLED_CMD); //设置显示开始行 [5:0],行数.
													    
	OLED_WR_Byte(0x8D, OLED_CMD); //电荷泵设置
	OLED_WR_Byte(0x14, OLED_CMD); //bit2，开启/关闭
	OLED_WR_Byte(0x20, OLED_CMD); //设置内存地址模式
	OLED_WR_Byte(0x02, OLED_CMD); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
	OLED_WR_Byte(0xA1, OLED_CMD); //段重定义设置,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0, OLED_CMD); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WR_Byte(0xDA, OLED_CMD); //设置COM硬件引脚配置
	OLED_WR_Byte(0x12, OLED_CMD); //[5:4]配置
		 
	OLED_WR_Byte(0x81, OLED_CMD); //对比度设置
	OLED_WR_Byte(0xEF, OLED_CMD); //1~255;默认0X7F (亮度设置,越大越亮)
	OLED_WR_Byte(0xD9, OLED_CMD); //设置预充电周期
	OLED_WR_Byte(0xf1, OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB, OLED_CMD); //设置VCOMH 电压倍率
	OLED_WR_Byte(0x30, OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4, OLED_CMD); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WR_Byte(0xA6, OLED_CMD); //设置显示方式;bit0:1,反相显示;0,正常显示	    						   
	OLED_WR_Byte(0xAF, OLED_CMD); //开启显示	
	
	
	OLED_Clear();

	
}


uint8_t OLED_GRAM[128][8];	 
extern void OLED_Refresh_Gram(void) {
	uint8_t i,n;		    
	for (i = 0; i < 8; i++) {  
		OLED_WR_Byte(0xb0 + i, OLED_CMD);    //设置页地址（0~7）
		OLED_WR_Byte(0x00, OLED_CMD);      //设置显示位置—列低地址
		OLED_WR_Byte(0x10, OLED_CMD);      //设置显示位置—列高地址   
		for (n = 0; n < 128; n++) OLED_WR_Byte(OLED_GRAM[n][i], OLED_DATA); 
	}   
}

//开启OLED显示    
extern void OLED_Display_On(void) {
	OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X14, OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF, OLED_CMD);  //DISPLAY ON
}
//关闭OLED显示     
extern void OLED_Display_Off(void) {
	OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC命令
	OLED_WR_Byte(0X10, OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE, OLED_CMD);  //DISPLAY OFF
}		   			 
//清屏函数,清完屏,整个屏幕是黑色的!和没点亮一样!!!	  
extern void OLED_Clear(void) {  
	uint8_t i,n;  
	for (i = 0; i < 8; i++) {
		for(n = 0; n < 128; n++) {
			OLED_GRAM[n][i] = 0X00; 
		}
	}  
	OLED_Refresh_Gram();//更新显示
}

//清屏不显示
extern void OLED_Clear2(void) {
	uint8_t i,n;  
	for (i = 0; i < 8; i++) {
		for(n = 0; n < 128; n++) {
			OLED_GRAM[n][i] = 0X00; 
		}
	}  
}
//画点 
//x:0~127
//y:0~63
//t:1 填充 0,清空				   
extern void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t) {
	uint8_t pos, bx, temp = 0;
	if (x > 127 || y > 63) return;//超出范围了.
	pos = 7 - y / 8;
	bx = y % 8;
	temp = 1 << (7 - bx);
	if (t) OLED_GRAM[x][pos] |= temp;
	else OLED_GRAM[x][pos] &= ~temp;	    
}

//在指定位置显示一个字符,包括部分字符
//x:0~127
//y:0~63	
//chr:显示的字符		 
//size:选择字体 16/12 
//mode:0,反白显示;1,正常显示	
extern uint8_t OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode) {      			    
	uint8_t temp, t, t1;
	uint8_t y0 = y;
	chr = chr - ' ';//得到偏移后的值				   
    for (t = 0; t < size; t++) {   
		if (size == 12) temp = oled_asc2_1206[chr][t];  //调用1206字体
		else temp = oled_asc2_1608[chr][t];		 //调用1608字体 	                          
        for (t1 = 0; t1 < 8; t1++) {
			if (temp & 0x80) OLED_DrawPoint(x, y, mode);
			else OLED_DrawPoint(x, y, !mode);
			temp <<= 1;
			y++;
			if((y - y0) == size) {
				y = y0;
				x++;
				break;
			}
		}  	 
    } 
	return size / 2;         
}

//显示数值
//x,y:起点坐标  
//num为显示的数值
//len为数值的长度
//size字体大小，16或12
extern void OLED_ShowNumber(uint8_t x, uint8_t y, int16_t num, uint8_t len, uint8_t size) {
	uint8_t t, temp;
	uint8_t enshow = 0;
	for (t = 0; t < len; t++) {
		temp = (int16_t)(num / pow(10, len - t - 1)) % 10;
		if (enshow == 0 && t < len-1) {
			if (temp == 0) {
				OLED_ShowChar(x + size / 2 * t, y, ' ', size, 1);
				continue;
			}else enshow = 1; 
		 	 
		}
	 	OLED_ShowChar(x + size / 2 * t, y, temp + '0', size, 1); 
	}
} 

//显示数值
//x,y:起点坐标  
//num为显示的数值
//len为数值的长度
//size字体大小，16或12
extern uint8_t OLED_ShowNumber2(uint8_t x, uint8_t y, int16_t num, uint8_t size) {         	
	uint8_t len = 0;
	int16_t num2 = num;
	while (num2 != 0) {
		++len;
		num2 /= 10;
	}
	for (uint8_t i = 0; i < len; ++i) {
		uint8_t temp = (int16_t)(num / pow(10, len - i - 1)) % 10;    //从高位开始，取出第i位的数值
	 	OLED_ShowChar(x + size / 2 * i, y, temp + '0', size, 1); 
	}
	return (size / 2) * len; 
} 

//显示字符串
//x,y:起点坐标  
//*p:字符串起始地址
//size字体大小，16或12
extern uint8_t OLED_ShowString(uint8_t x,uint8_t y,const uint8_t *p, uint8_t size) {
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58 
	uint8_t len = 0;
    while (*p != '\0') {       
        if (x > MAX_CHAR_POSX ) {
			x = 0;
			y += size;
		}
        if (y > MAX_CHAR_POSY) {
			y = x = 0;
			OLED_Clear();
		}
        OLED_ShowChar(x, y, *p, size, 1);	 
        x += size / 2;
        ++p;
		++len;
    }  
	return (size / 2) * len;
}	






