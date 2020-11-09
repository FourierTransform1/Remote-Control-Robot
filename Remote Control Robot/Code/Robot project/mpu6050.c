
#include <inttypes.h>
#include <stdint.h>
//#include "i2c.h"
#include "mpu6050_reg.h"
#include "mpu6050.h"
#define I2C_WRITE   0


//configure important settings in mpu6050
//subject to change app(ilcation) by app
void mpu6050_init(void){
	i2c_write_byte(MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00); //exit sleep mode
	i2c_write_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00); // scale: 2 g
	i2c_start(MPU6050_ADDRESS+I2C_WRITE);
	
}


// read accel X, Y, Z all at once, high- & low-8-bits combined
// return int16_t (signed) in buff
//buff must have at least 3 available places
//no error handling for too small buff
void mpu6050_read_accel_ALL(int16_t * buff){
	
	uint8_t tmp[2];

	mpu6050_read_accel_X(tmp);
	buff[0] = (tmp[0]<<8)|(tmp[1]);
	mpu6050_read_accel_Y(tmp);
	buff[1] = (tmp[0]<<8)|(tmp[1]);
	mpu6050_read_accel_Z(tmp);
	buff[2] = (tmp[0]<<8)|(tmp[1]);
}


//read accel X, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff
void mpu6050_read_accel_X(uint8_t * buff){
	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, buff);
	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, buff+1);
}

//read accel Y, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff
void mpu6050_read_accel_Y(uint8_t * buff){

	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, buff);
	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_L, buff+1);
}

//read accel Z, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff
void mpu6050_read_accel_Z(uint8_t * buff){

	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, buff);
	i2c_read_byte(MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, buff+1);
}






