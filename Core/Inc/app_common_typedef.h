/*
 * app_common_typedef.h
 *
 *  Created on: 10 mar. 2022
 *      Author: ismae
 */

#ifndef INC_APP_COMMON_TYPEDEF_H_
#define INC_APP_COMMON_TYPEDEF_H_

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



void MPU6050_Init (I2C_HandleTypeDef hi2c)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

struct AccelFunction MPU6050_Read_Accel (I2C_HandleTypeDef hi2c)
{
	uint8_t Rec_Data[6];

	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;
	struct AccelFunction datos;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/
	datos.Ax = atan(-1*(Accel_X_RAW/A_R)/sqrt(pow((Accel_Y_RAW/A_R),2) + pow((Accel_Z_RAW/A_R),2)))*RAD_TO_DEG;
	datos.Ay = atan((Accel_Y_RAW/A_R)/sqrt(pow((Accel_X_RAW/A_R),2) + pow((Accel_Z_RAW/A_R),2)))*RAD_TO_DEG;

	return datos;
}

struct GyroFunction MPU6050_Read_Gyro (I2C_HandleTypeDef hi2c)
{
	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;
	struct GyroFunction datos;

	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	datos.Gx = Gyro_X_RAW/131.0;
	datos.Gy = Gyro_Y_RAW/131.0;
	datos.Gz = Gyro_Z_RAW/131.0;

	return datos;
}
#endif /* INC_APP_COMMON_TYPEDEF_H_ */
