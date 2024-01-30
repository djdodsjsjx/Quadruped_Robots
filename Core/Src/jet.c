#include "jet.h"
#include "DT.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
uint8_t jet_cmd_count[40];    //接收的指令次数
uint8_t jet_cmd = 4; //控制转换指令，默认为4
// uint8_t jet_data = 0;

// extern void Jet_Byte_Get(uint8_t bytedata) {	       //从Jetson处获取数据
// 	static uint8_t rec_sta = 0;                        //通信数据状态位
// 	static uint8_t fun = 0;                            //接收的功能字
// 	static uint8_t temp_cmd = 0, temp_data = 0;        //接收临时控制指令与数据
// 	if (bytedata == 0xF0) rec_sta = 1;                 //检测到帧头0xF0，开始读取数据，状态位置1
// 	else if (rec_sta == 1) fun = bytedata, ++rec_sta;  //状态位为1时，读取当前功能字，状态位前移
// 	else if (rec_sta == 2) {                           //状态位为2时，读取指令和数据内容
// 		if (fun == 0x0C) temp_cmd = bytedata, ++rec_sta;   //功能位为0x0C时，只控制指令
// 		else if (fun == 0X0B) {							   //功能位为0x0B时，取出控制指令和数据内容
// 			if (temp_cmd == 0) temp_cmd = bytedata;
// 			else {
// 				temp_data = bytedata;
// 				++rec_sta;
// 			}
// 		}
// 	}
// 	else if (rec_sta == 3 && bytedata == 0x0D) rec_sta++; //0x0D和0x0A为结束标志位
// 	else if (rec_sta == 4 && bytedata == 0x0A) {
// 		jet_cmd = temp_cmd;                               //结束时，将临时变量赋给控制变量
// 		jet_data = temp_data;
// 		++jet_cmd_count[jet_cmd];
// 		temp_cmd = 0;                                     //清除临时变量
// 		temp_data = 0;
// 	}
// }

int16_t jet_data = 0;
extern void Jet_Byte_Get(uint8_t bytedata) {	       //从Jetson处获取数据

	static uint8_t rec_sta = 0;                        //通信数据状态位
	static uint8_t fun = 0;                            //接收的功能字
	static int16_t tmp_data = 0;
	if (rec_sta == 0 && bytedata == 0xF0) rec_sta = 1;                 //检测到帧头0xF0，开始读取数据，状态位置1
	else if (rec_sta == 1) fun = bytedata, ++rec_sta;  //状态位为1时，读取当前功能字，状态位前移
	else if (rec_sta >= 2 && rec_sta <= 3) {                           //状态位为2时，读取指令和数据内容
		if (fun == 0x0C) {
			tmp_data = tmp_data << 8 | bytedata;
			rec_sta ++;
		} 
	}
	else if (rec_sta == 4 && bytedata == 0x0D) rec_sta++; //0x0D和0x0A为结束标志位
	else if (rec_sta == 5 && bytedata == 0x0A) {
		DT_SendF1(tmp_data, 0, 0, 0, 0, 0);
		jet_data = tmp_data;
		tmp_data = 0;
		rec_sta = 0;
		// DT_SendF1(jet_data, 0, 0, 0, 0, 0);
	}
    // jet_data = bytedata;
    // HAL_UART_Transmit(&huart1, &jet_data, 1, 1);
}

//串口发送的指令处理 F0 0C CMD（1byte） data（N byte） 0d 0a
// void Send_To_Comptuer(int16_t data) {
// 	uint8_t cnt = 0;
//     uint8_t send_to_jet[7] = {0};
// 	send_to_jet[cnt++] = 0XF0;
// 	send_to_jet[cnt++] = 0X0C;
// 	send_to_jet[cnt++] = 0X01;
// 	send_to_jet[cnt++] = data;             //小端模式
// 	send_to_jet[cnt++] = data >> 8;
// 	send_to_jet[cnt++] = 0X0D;
// 	send_to_jet[cnt++] = 0X0A;
// 	HAL_UART_Transmit(&huart3, send_to_jet, cnt, 1);
// }

// extern void Receive_Test(uint8_t bytedata) {	   
// 	static uint8_t rec_sta = 0;
// 	static int16_t temp = 0;
// 	if (bytedata == 0xF0) rec_sta = 1;
// 	else if (rec_sta == 1 && bytedata == 0x0C) rec_sta++;
// 	else if (rec_sta == 2 && bytedata == 0x01) rec_sta++;
// 	else if (rec_sta == 3) temp = bytedata, rec_sta++;
// 	else if (rec_sta == 4) temp += bytedata << 8, rec_sta++;
// 	else if (rec_sta == 5 && bytedata == 0x0D) rec_sta++;
// 	else if (rec_sta == 6 && bytedata == 0x0A) {
// 	DT_SendF1(temp, 0, 0, 0, 0, 0);
// 	}
// }

// extern void Receive_Test2(uint8_t* res) {	   
// 	if (res[0] == 0xF0 && res[1] == 0x0C && res[2] == 0x01 && res[5] == 0x0D && res[6] == 0x0A) {
// 		DT_SendF1(res[3] + res[4] << 8, 0, 0, 0, 0, 0);
// 	}
// }

void send_to_jet(const int16_t* nums, uint8_t len) {
	uint8_t cnt = 0, sc = 0, ac = 0;
    uint8_t send_to_jet[100] = {0};
	send_to_jet[cnt++] = 0XAA;
	send_to_jet[cnt++] = 0XFF;
	send_to_jet[cnt++] = 0XF1;
	send_to_jet[cnt++] = len * 2;
	for (uint8_t i = 0; i < len; ++i) {
		send_to_jet[cnt++] = *(char*)(&nums[i]);
		send_to_jet[cnt++] = *((char*)(&nums[i]+1));
		sc += send_to_jet[cnt - 1] + send_to_jet[cnt - 2];
		ac += sc;
	}	
	send_to_jet[cnt++] = sc;
	send_to_jet[cnt++] = ac;
	HAL_UART_Transmit_IT(&huart3, &send_to_jet, cnt);
}
