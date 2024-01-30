#include "DT.h"
#include "usart.h"

#define BYTE0(dwTemp)     (*(char*)(&dwTemp))
#define BYTE1(dwTemp)     (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp)     (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp)     (*((char*)(&dwTemp)+3))

uint8_t DataToSend[100];
void DT_Send(const int16_t* a, uint8_t len) {
	uint8_t cnt = 0, sc = 0, ac = 0;
	DataToSend[cnt++] = 0XAA;
	DataToSend[cnt++] = 0XFF;
	DataToSend[cnt++] = 0XF1;
	DataToSend[cnt++] = len * 2;
	for (uint8_t i = 0; i < len; ++i) {
		DataToSend[cnt++] = BYTE0(a[i]);
		DataToSend[cnt++] = BYTE1(a[i]);
		sc += DataToSend[cnt - 1] + DataToSend[cnt - 2];
		ac += sc;
	}	
	DataToSend[cnt++] = sc;
	DataToSend[cnt++] = ac;
	HAL_UART_Transmit_IT(&huart1, &DataToSend, cnt);
}

//×Ô¶¨Òå
void DT_SendF1(int16_t a_1, int16_t a_2,int16_t a_3,int16_t a_4,int16_t a_5,int16_t a_6) {
	uint8_t _cnt = 0;
	
	DataToSend[_cnt++] = 0XAA;
	DataToSend[_cnt++] = 0XFF;
	DataToSend[_cnt++] = 0XF1;
	DataToSend[_cnt++] = 0;
	
	DataToSend[_cnt++] = BYTE0(a_1);
	DataToSend[_cnt++] = BYTE1(a_1);
	
	DataToSend[_cnt++] = BYTE0(a_2);
	DataToSend[_cnt++] = BYTE1(a_2);
	
	DataToSend[_cnt++] = BYTE0(a_3);
	DataToSend[_cnt++] = BYTE1(a_3);
	
	DataToSend[_cnt++] = BYTE0(a_4);
	DataToSend[_cnt++] = BYTE1(a_4);
	
	DataToSend[_cnt++] = BYTE0(a_5);
	DataToSend[_cnt++] = BYTE1(a_5);
	
	DataToSend[_cnt++] = BYTE0(a_6);
	DataToSend[_cnt++] = BYTE1(a_6);

	DataToSend[3] = _cnt-4;
	
	uint8_t sc = 0;
	uint8_t ac = 0;
	for(uint8_t i=0; i<DataToSend[3]+4; i++) {
		sc += DataToSend[i];
		ac += sc;
	}
	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;

	HAL_UART_Transmit_IT(&huart1, &DataToSend, _cnt);
}

void DT_SendF1_08(uint8_t a_1, uint8_t a_2,uint8_t a_3,uint8_t a_4,uint8_t a_5,uint8_t a_6) {
	uint8_t _cnt = 0;
	
	DataToSend[_cnt++] = 0XAA;
	DataToSend[_cnt++] = 0XFF;
	DataToSend[_cnt++] = 0XF1;
	DataToSend[_cnt++] = 0;
	
	DataToSend[_cnt++] = a_1;
	
	DataToSend[_cnt++] = a_2;
	DataToSend[_cnt++] = a_3;
	
	DataToSend[_cnt++] = BYTE0(a_3);
	DataToSend[_cnt++] = BYTE1(a_3);
	
	DataToSend[_cnt++] = BYTE0(a_4);

	DataToSend[3] = _cnt-4;
	
	uint8_t sc = 0;
	uint8_t ac = 0;
	for(uint8_t i=0; i<DataToSend[3]+4; i++) {
		sc += DataToSend[i];
		ac += sc;
	}
	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;

	HAL_UART_Transmit_IT(&huart1, &DataToSend, _cnt);
}

//mpu6050

void DT_Send01(int16_t acc_x, int16_t acc_y,int16_t acc_z,int16_t gyrox,int16_t gyroy,int16_t gyroz, uint8_t sta) {
	uint8_t _cnt = 0;
	
	DataToSend[_cnt++] = 0XAA;
	DataToSend[_cnt++] = 0XFF;
	DataToSend[_cnt++] = 0X01;
	DataToSend[_cnt++] = 0;
	
	DataToSend[_cnt++] = BYTE0(acc_x);
	DataToSend[_cnt++] = BYTE1(acc_x);
	
	DataToSend[_cnt++] = BYTE0(acc_y);
	DataToSend[_cnt++] = BYTE1(acc_y);
	
	DataToSend[_cnt++] = BYTE0(acc_z);
	DataToSend[_cnt++] = BYTE1(acc_z);
	
	DataToSend[_cnt++] = BYTE0(gyrox);
	DataToSend[_cnt++] = BYTE1(gyrox);
	
	DataToSend[_cnt++] = BYTE0(gyroy);
	DataToSend[_cnt++] = BYTE1(gyroy);
	
	DataToSend[_cnt++] = BYTE0(gyroz);
	DataToSend[_cnt++] = BYTE1(gyroz);
	
	DataToSend[_cnt++] = sta;

	DataToSend[3] = _cnt-4;
	
	uint8_t sc = 0;
	uint8_t ac = 0;
	for(uint8_t i=0; i<DataToSend[3]+4; i++) {
		sc += DataToSend[i];
		ac += sc;
	}
	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;

	// Uart1_Put_Buf(DataToSend,_cnt);
}


void DT_Send03(int16_t rol, int16_t pit,int16_t yaw, uint8_t sta) {
	rol=rol*100;
	pit=pit*100;
	yaw=yaw*100;
	uint8_t _cnt = 0;
	
	DataToSend[_cnt++] = 0XAA;
	DataToSend[_cnt++] = 0XFF;
	DataToSend[_cnt++] = 0X03;
	DataToSend[_cnt++] = 0;
	
	DataToSend[_cnt++] = BYTE0(rol);
	DataToSend[_cnt++] = BYTE1(rol);
	
	DataToSend[_cnt++] = BYTE0(pit);
	DataToSend[_cnt++] = BYTE1(pit);
	
	DataToSend[_cnt++] = BYTE0(yaw);
	DataToSend[_cnt++] = BYTE1(yaw);
	
	DataToSend[_cnt++] = sta;

	DataToSend[3] = _cnt-4;
	
	uint8_t sc = 0;
	uint8_t ac = 0;
	for(uint8_t i=0; i<DataToSend[3]+4; i++) {
		sc += DataToSend[i];
		ac += sc;
	}
	
	DataToSend[_cnt++] = sc;
	DataToSend[_cnt++] = ac;

	// HAL_UART_Transmit(&huart1, (uint8_t* )&DataToSend, _cnt, 0xffff);
	HAL_UART_Transmit_IT(&huart1, &DataToSend, _cnt);
	// Uart1_Put_Buf(DataToSend,_cnt);
}
