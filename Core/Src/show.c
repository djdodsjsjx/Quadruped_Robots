#include "show.h"
#include "oled.h"
#include "openmv.h"
extern int16_t ROBOT_ROL, ROBOT_PIT, ROBOT_YAW;

uint8_t x = 0, y = 0;

//计算新添加的数值需要占用多少个长度
static uint8_t Check_Len(const char* p, int16_t num, uint8_t size) {
	uint8_t len = 0;
	while(*p != '\0') {
		++p;
		++len;
	}
	while(num != 0) {
		num /= 10;
		++len;
	}
	return (len + 1) * (size / 2);   //len加一有符号位
}

/*
 * p：该数值对应的字符串
 * num：该数值
 * size：选择字体大小，16或12
 * */
static void Show_Val(const char* p, int16_t num, uint8_t size) {
	if (x > 0) x += size / 2;      //若当前数值之前已经插入过数值，空一格
	if (x + Check_Len(p, num, size) > 128) {  //若插入当前字符后超过当前行，转至下一行插入
		x = 0;
		y += size + 3;             
	}
	if (y > 64 - size) return ;
	x += OLED_ShowString(x, y, p, size);
	if (num >= 0) {
		x += OLED_ShowChar(x, y, '+', size, 1);
		x += OLED_ShowNumber2(x, y, num, size);
	} else {
		x += OLED_ShowChar(x, y, '-', size, 1);
		x += OLED_ShowNumber2(x, y, -num, size);
	}
}
extern int16_t yaw_target;
extern uint8_t PA_PO_FLAG, Turn_Left_FLAG, Turn_Right_FLAG;
extern uint8_t esp_cmd, esp_data;
//size: 字体大小 可选12或16
extern void oled_show(uint8_t size)
{
	OLED_Clear2();      //清楚上一次OLED缓存
	x = 0;
	y = 0;

	//姿态角显示
	Show_Val("yaw:", ROBOT_YAW, size);
	Show_Val("pit:", ROBOT_PIT, size);
	Show_Val("rol:", ROBOT_ROL, size);

	//循迹参数显示
	Show_Val("sta:", opmv.lt.sta, size);
	Show_Val("ang:", opmv.lt.angle, size);
	Show_Val("det:", opmv.lt.deviation, size);

	//上位机显示
	// Show_Val("cmd:", esp_cmd, size);
	// Show_Val("data:", esp_data, size);
	// Show_Val("pox:", opmv.lt.pos_x, size);
	// Show_Val("poy:", opmv.lt.pos_y, size);
	// Show_Val("tar:", yaw_target, size);
	// Show_Val("P:", PA_PO_FLAG, size);
	// Show_Val("L:", Turn_Left_FLAG, size);
	// Show_Val("R:", Turn_Right_FLAG, size);
	OLED_Refresh_Gram();	 //更新这次显示
}

