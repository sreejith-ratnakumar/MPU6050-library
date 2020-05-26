/*
file name: MPU6050.h
description: This library is written for the basic functionalities of
	     the IMU MPU6050. The library can be used to configure the 
	     sensor as well as read from the sensor. The feature of FSYNC 
	     can not be implemented using this library. The function names
	     which contain "Actual" can be used to read the data in 
	     respective units like deg per sec and g. The functions with names
	     which donot have "Actual" can be used to read the raw data output.
	     This header file contains the necessary structures, enumerators and
	     function declaration.
author: Sreejith Ratnakumar, Vishnu Vijay, Aswathi T
link: https://github.com/sreejith-ratnakumar
*****************************************************************************
		COPYRIGHT(c) 2020 SREEJITH RATNAKUMAR 
*****************************************************************************
ATTENTION
This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License 3 as published by
    the Free Software Foundation.
    
		This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
		
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
******************************************************************************
*/
#include "stm32f1xx_hal.h"
#include <string.h>
//register adresses defined
#define CONFIG 0X1A
#define GYRO_CONFIG 0X1B
#define ACCEL_CONFIG 0X1C
#define INT_ENABLE 0X38
#define INT_STATUS 0X3A
#define ACCEL_XOUT_H 0X3B
#define ACCEL_XOUT_L 0X3C
#define ACCEL_YOUT_H 0X3D
#define ACCEL_YOUT_L 0X3E
#define ACCEL_ZOUT_H 0X3F
#define ACCEL_ZOUT_L 0X40
#define TEMP_OUT_H 0X41
#define TEMP_OUT_L 0X42
#define GYRO_XOUT_H 0X43
#define GYRO_XOUT_L 0X44
#define GYRO_YOUT_H 0X45
#define GYRO_YOUT_L 0X46
#define GYRO_ZOUT_H 0X47
#define GYRO_ZOUT_L 0X48
#define PWR_MGMT_1 0X6B
#define PWR_MGMT_2 0X6C
#define SMPRT_DIV 0X19

//device adress
#define MPUADDRESS 0X68

typedef struct{
	uint16_t x;
	uint16_t y;
	uint16_t z;
}Data;

typedef struct{
	uint8_t SLEEP;
	uint8_t CLKSRC;
	uint8_t GYR_FS;
	uint8_t ACCEL_FS;
	uint8_t DLFP;
	uint8_t TEMP_DIS;
	uint8_t SMPRT;
}MPU_Config_Struct;

enum CLKSOURCE{
	INT_8MHz=0x0,
	PLL_X_Gyr_Ref=0x1,
	PLL_Y_Gyr_Ref=0x2,
	PLL_Z_Gyr_Ref=0x3,
	PLL_Ext32KHz_Ref=0x4,
	PLL_Ext19MHz_Ref=0x5
};
	
enum Accelerometer_FS{
	FS_2g=0x0,
	FS_4g=0x1,
	FS_8g=0x2,
	FS_16g=0x3
};

enum Gyroscope_FS{
	FS_250dps=0x0,
	FS_500dps=0x1,
	FS_1000dps=0x2,
	FS_2000dps=0x3
};

enum Sleep_bit{
	Low_Power_Mode=0x1,
	Full_Power_Mode=0x0
};

enum Temp_Sensor{
	Temp_Enable=0x0,
	Temp_Disable=0x1
};

enum DLPF_Configuration{
	DLPF_BW_260=0x0,
	DLPF_BW_180=0x1,
	DLPF_BW_90=0x2,
	DLPF_BW_40=0x3,
	DLPF_BW_20=0x4,
	DLPF_BW_10=0x5,
	DLPF_BW_5=0x6,
};

//function declarations
void Write(uint8_t Data,uint8_t Address);
void Read(uint8_t *Dataptr,uint8_t Address,uint16_t NoOfBytes);
void MPU_6050_Initialize(I2C_HandleTypeDef *i2c);
void MPU6050_Config(MPU_Config_Struct *Config);
void MPU6050_Get_Accel_Data(Data *accel);
void MPU6050_Get_Gyro_Data(Data *gyro);
void MPU6050_Get_Temp(uint16_t* ptr);
void Write_SampleRate(uint8_t samplrate);

/*
******************************************************************************
	  MPU6050 library Copyright (C) 2020  Sreejith Ratnakumar
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
*******************************************************************************
*/
