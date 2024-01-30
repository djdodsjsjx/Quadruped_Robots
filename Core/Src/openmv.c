 #include "openmv.h"

//设定
#define OPMV_OFFLINE_TIME_MS  1000  //毫秒

//全局变量
uint16_t offline_check_time;
uint8_t openmv_buf[20];
_openmv_data_st opmv;

int16_t sum_angle,sum_deviation;
uint16_t opmv_t;


/**********************************************************************************************************
*函 数 名: OpenMV_Check_Reset
*功能说明: OpenMV掉线检测复位，证明没有掉线
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void OpenMV_Check_Reset() {
	offline_check_time = 0;
	opmv.offline = 0;
}

static void OpenMV_Data_Analysis(uint8_t *buf_data, uint8_t len) {
	opmv.lt.sta = *(buf_data + 5);
	opmv.lt.angle = (int16_t)((*(buf_data + 6) << 8) | *(buf_data + 7));
	opmv.lt.deviation = (int16_t)((*(buf_data + 8) << 8) | *(buf_data + 9));
	opmv.lt.p_flag = *(buf_data + 10);
	opmv.lt.pos_x = (int16_t)((*(buf_data + 11) << 8) | *(buf_data + 12));
	opmv.lt.pos_y = (int16_t)((*(buf_data + 13) << 8) | *(buf_data + 14));
	opmv.lt.dT_ms = *(buf_data + 15);

	opmv.mode_sta = 2;
	// opmv_t++;
	// sum_angle += opmv.lt.angle;
	// sum_deviation += opmv.lt.deviation;
	OpenMV_Check_Reset();
}



/**********************************************************************************************************
*函 数 名: OpenMV_Byte_Get
*功能说明: OpenMV字节数据获取
*参    数: 字节数据
*返 回 值: 无
**********************************************************************************************************/
extern void OpenMV_Byte_Get(uint8_t bytedata) {	
	static uint8_t len = 0,rec_sta;
	uint8_t check_val = 0;	
	openmv_buf[rec_sta] = bytedata;
	if (rec_sta == 0) {
		if(bytedata == 0xaa) rec_sta++;
		else rec_sta = 0;
	}
	else if (rec_sta == 1) {
		//(bytedata==0x29)未确定
		if (1) rec_sta++;
		else rec_sta = 0;	
	}
	else if (rec_sta == 2) {
		if (bytedata == 0x05) rec_sta++;
		else rec_sta = 0;	
	}
	else if (rec_sta == 3) {
		if (bytedata == 0x41 || bytedata == 0x42) rec_sta++; 
		else rec_sta = 0;	
	}
	else if (rec_sta == 4) {
		len = bytedata;
		if(len < 20) rec_sta++;	
		else rec_sta = 0;
	}
	else if(rec_sta == (len+5)) {
		for (uint8_t i = 0; i < len + 5; i++) {   //加5表示前面有5个字节
			check_val += openmv_buf[i];
		}
		if(check_val == bytedata) {
			//解析成功
			OpenMV_Data_Analysis(openmv_buf, len + 6);
			rec_sta = 0;
		}
		else rec_sta = 0;	
	}
	else rec_sta++;
}


/**********************************************************************************************************
*函 数 名: OpenMV_Offline_Check
*功能说明: OpenMV掉线检测，用来检测硬件是否在线
*参    数: 时间（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void OpenMV_Offline_Check(uint8_t dT_ms) {
	if(offline_check_time < OPMV_OFFLINE_TIME_MS) {
		offline_check_time += dT_ms;
	}
	else {
		opmv.offline = 1;
		opmv.mode_sta = 0;
	}
	
}
