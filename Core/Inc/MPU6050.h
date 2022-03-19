/*
 * MPU6050.h
 *
 *  Created on: 18 mar. 2022
 *      Author: ismae
 */
#include "app_common_typedef.h"
//#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal.h"
#include "math.h"


//Define here
#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

#define A_R 16384.0
#define RAD_TO_DEG  57.295779

//Function Here
void MPU6050_Init (I2C_HandleTypeDef hi2c);
struct GyroFunction MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c);
struct AccelFunction MPU6050_Read_Accel (I2C_HandleTypeDef hi2c);


#endif /* INC_MPU6050_H_ */
