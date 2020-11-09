/*
	useful functions to manipulate mpu6050
*/

#ifndef MPU6050
#define MPU6050

//configure important settings in mpu6050
//subject to change app(ilcation) by app
void mpu6050_init(void);


// read accel X, Y, Z all at once, high- & low-8-bits combined
// return int16_t (signed) in buff
// buff must have at least 3 available places
// data sequence: (buff)-->X, (buff+1)-->Y, (buff+2)-->Z
// no error handling for too small buff
void mpu6050_read_accel_ALL(int16_t * buff);


//read accel X, Y, Z, high- & low-8-bits separated, high first
//buff must have at least 2 available places
//no error handling for too small buff

void mpu6050_read_accel_X(uint8_t * buff);
void mpu6050_read_accel_Y(uint8_t * buff);
void mpu6050_read_accel_Z(uint8_t * buff);



#endif
