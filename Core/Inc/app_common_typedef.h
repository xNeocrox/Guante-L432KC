/*
 * app_common_typedef.h
 *
 *  Created on: 10 mar. 2022
 *      Author: ismae
 */

#ifndef INC_APP_COMMON_TYPEDEF_H_
#define INC_APP_COMMON_TYPEDEF_H_

#include "stdint.h"


struct AccelFunction{
	float Ax;
	float Ay;
};


struct GyroFunction{
	float Gx;
	float Gy;
	float Gz;
};

struct Datos_Analogicos
{
	uint8_t Analog_pulgar;
	uint8_t Analog_indice;
	uint8_t Analog_corazon;
	uint8_t Analog_anular;
	uint8_t Analog_menique;
};

struct Datos_Gyro
{
	uint8_t AnguloX;
	uint8_t AnguloY;

};

typedef enum{
	WAIT_TIMER_TO_BE_READY,
	SEARCHING_OLED_I2C_DEVICE,
	OLED_IS_AVAILABLE,
	OLED_IS_NOT_AVAILABLE,
	INIT_WITH_LED,
	RUNNING_O,
	RUNNING_L
}Init_State;

typedef enum{
	SEARCHING_GYRO_I2C_DEVICE,
	GYRO_IS_AVAILABLE,
	GYRO_IS_NOT_AVAILABLE,
	RUNNING
}Gyro_State;


#endif /* INC_APP_COMMON_TYPEDEF_H_ */
