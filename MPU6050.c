/*
file name: MPU6050.c
description: This library is written for the basic functionalities of
	     the IMU MPU6050. The library can be used to configure the 
	     sensor as well as read from the sensor. The feature of FSYNC 
	     can not be implemented using this library. The function names
	     which contain "Actual" can be used to read the data in 
	     respective units like deg per sec and g. The functions with names
	     which donot have "Actual" can be used to read the raw data output.
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
#include "MPU6050.h"

I2C_HandleTypeDef i2chandle;
double accel_sensitvity_scale,gyro_sensitivity_scale;

/**
  * @brief  Write 8 bit to the sensor and intented to be called
						only within library
  * @param  Data: 8 bit data to be written to the device
  * @param  Address: Target device address
  * @retval No return
  */
void Write(uint8_t Data,uint8_t Address)
{
	uint8_t buff[2];
	buff[0]=Address;
	buff[1]=Data;
	uint16_t DevAddress=MPUADDRESS<<1;
	HAL_I2C_Master_Transmit(&i2chandle,DevAddress,buff,1,100);
}

/**
  * @brief  Read 8 bit from sensor register and intended to
						be called only within library, not for value reading
  * @param  Dataptr: 8 bit data array pointer to where data is written
  * @param  Address: Target device address
	* @param  NoOfBytes: Number of bytes to be read from sensor
	* @retval No return
  */
void Read(uint8_t *Dataptr,uint8_t Address,uint16_t NoOfBytes)
{
	uint8_t buff[2];
	buff[0]=Address;
	uint16_t DevAddress=MPUADDRESS<<1;
	HAL_I2C_Master_Transmit(&i2chandle,DevAddress,buff,1,10);
	HAL_I2C_Master_Receive(&i2chandle,DevAddress,Dataptr,NoOfBytes,100);
}

/**
  * @brief  Initialize the MPU and enable the interrupts
  * @param  i2c: I2C handler used in program copied to library
  * @retval No return
  */
void MPU_6050_Initialize(I2C_HandleTypeDef *i2c)
{
	memcpy(&i2chandle,i2c,sizeof(&i2c));
	Write(0X19,INT_ENABLE);
}

/**
  * @brief  Configure the MPU according to the user requirement
  * @param  Config: structure that hasthe configurations to be applied
  * @retval No return
  */
void MPU6050_Config(MPU_Config_Struct *Config)
{
	Write(0x80,PWR_MGMT_1);
	HAL_Delay(100);
	uint8_t buf=(Config->CLKSRC)&0x07;
	buf |= (Config->SLEEP<<6)&0x40;
	Write(buf,PWR_MGMT_1);
	HAL_Delay(100);
	buf=(Config->DLFP)&0x07;
	Write(buf,CONFIG);
	buf=(Config->GYR_FS)&0x03;
	buf=buf<<3;
	Write(buf,GYRO_CONFIG);
	buf=(Config->ACCEL_FS)&0x03;
	buf=buf<<3;
	Write(buf,ACCEL_CONFIG);
	if(Config->DLFP==0)
		Config->DLFP=0x04;
	Write_SampleRate(Config->DLFP);
	if(Config->ACCEL_FS==FS_2g)
		accel_sensitvity_scale=2/16384;
	else if(Config->ACCEL_FS==FS_4g)
		accel_sensitvity_scale=4/8192;
	else if(Config->ACCEL_FS==FS_8g)
		accel_sensitvity_scale=8/4096;
	if(Config->ACCEL_FS==FS_16g)
		accel_sensitvity_scale=16/2048;
	
	if(Config->GYR_FS==FS_250dps)
		gyro_sensitivity_scale=250/131;
	else if(Config->GYR_FS==FS_500dps)
		gyro_sensitivity_scale=500/65.5;
	else if(Config->GYR_FS==FS_1000dps)
		gyro_sensitivity_scale=1000/32.8;
	if(Config->GYR_FS==FS_2000dps)
		gyro_sensitivity_scale=2000/16.4;
}

/**
  * @brief  To read the accelerometer data from the sensor
  * @param  accel: structure to put the accelerometer data
  * @retval No return
  */
void MPU6050_Get_Accel_Data(Data *accel)
{
	uint8_t buf;
	uint8_t dat[6];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		uint8_t acc[6];
		Read(dat,ACCEL_XOUT_H,6);
		accel->x=(dat[0]<<8)| dat[1];
		accel->y=(dat[2]<<8)| dat[3];
		accel->z=(dat[4]<<8)| dat[5];
	}
}

/**
  * @brief  To read the gyroscope data from the sensor
  * @param  gyro: structure to put the gyroscope data
  * @retval No return
  */
void MPU6050_Get_Gyro_Data(Data *gyro)
{
	uint8_t buf;
	uint8_t dat[6];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		uint8_t acc[6];
		Read(dat,GYRO_XOUT_H,6);
		gyro->x=(dat[0]<<8)| dat[1];
		gyro->y=(dat[2]<<8)| dat[3];
		gyro->z=(dat[4]<<8)| dat[5];
	}
}

/**
  * @brief  To read the temperature data from the sensor
  * @param  temp: variable to store the temperature
  * @retval No return
  */
void MPU6050_Get_Temp(uint16_t* temp)
{
	uint8_t buf;
	uint8_t dat[2];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		Read(dat,TEMP_OUT_H,2);
		*temp=(dat[0]<<8)|dat[1];
	}
}

/**
  * @brief  To read the actual accelerometer data from the sensor
						The output will be in the unit of g(9.8m/s2)
  * @param  accel: structure to put the accelerometer data
  * @retval No return
  */
void MPU6050_Get_Accel_ActualData(Data *accel)
{
	uint8_t buf;
	uint8_t dat[6];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		uint8_t acc[6];
		Read(dat,ACCEL_XOUT_H,6);
		accel->x=(dat[0]<<8)| dat[1];
		accel->x*=accel_sensitvity_scale;
		accel->y=(dat[2]<<8)| dat[3];
		accel->y*=accel_sensitvity_scale;
		accel->z=(dat[4]<<8)| dat[5];
		accel->z*=accel_sensitvity_scale;
	}
}

/**
  * @brief  To read the actual gyroscope data from the sensor
						The output will be in the unit deg per second.
  * @param  gyro: structure to put the gyroscope data
  * @retval No return
  */
void MPU6050_Get_Gyr_ActualData(Data *gyro)
{
	uint8_t buf;
	uint8_t dat[6];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		uint8_t acc[6];
		Read(dat,GYRO_XOUT_H,6);
		gyro->x=(dat[0]<<8)| dat[1];
		gyro->x*=gyro_sensitivity_scale;
		gyro->y=(dat[2]<<8)| dat[3];
		gyro->y*=gyro_sensitivity_scale;
		gyro->z=(dat[4]<<8)| dat[5];
		gyro->z*=gyro_sensitivity_scale;
	}
}

/**
  * @brief  To read the actual temperature data from the sensor
						The unit of the output will be in degree celcius
  * @param  temp: variable to store the temperature
  * @retval No return
  */
void MPU6050_Get_ActualTemp(uint16_t *temp)
{
	uint8_t buf;
	uint8_t dat[2];
	Read(&buf,INT_STATUS,1);
	if((buf&0x01)==0x01)
	{
		Read(dat,TEMP_OUT_H,2);
		*temp=(dat[0]<<8)|dat[1];
	}
	*temp*=(125/340);
}
/**
  * @brief  To read the sample rate value
  * @param  samplrate: variable to store the sample rate
  * @retval No return
  */
void Read_SampleRate(uint8_t *samplrate)
{
	Read(samplrate,SMPRT_DIV,1);
}

/**
  * @brief  To write the sample rate value to register
  * @param  samplrate: variable to store the sample rate
  * @retval No return
  */
void Write_SampleRate(uint8_t samplrate)
{
	Write(samplrate,SMPRT_DIV);
}

/*
******************************************************************************
		MPU6050 library Copyright (C) 2020  Sreejith Ratnakumar
    This program comes with ABSOLUTELY NO WARRANTY; for details type `show w'.
    This is free software, and you are welcome to redistribute it
    under certain conditions; type `show c' for details.
*******************************************************************************
*/
