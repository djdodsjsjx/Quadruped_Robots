#ifndef __OPENMV_H
#define __OPENMV_H
#include "main.h"

typedef struct
{
	//
	uint8_t color_flag;
	uint8_t sta;
	int16_t pos_x;
	int16_t pos_y;
	uint8_t dT_ms;

}_openmv_color_block_st;

typedef struct
{
	//
	uint8_t sta;	
	int16_t angle;
	int16_t deviation;
	uint8_t p_flag;
	int16_t pos_x;
	int16_t pos_y;
	uint8_t dT_ms;

}_openmv_line_tracking_st;

typedef struct
{
	uint8_t offline;
	uint8_t mode_cmd;
	uint8_t mode_sta;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
}_openmv_data_st;
//==数据声明
extern _openmv_data_st opmv;

//==函数声明

//public
void OpenMV_Offline_Check(uint8_t dT_ms);
extern void OpenMV_Byte_Get(uint8_t bytedata);

#endif

