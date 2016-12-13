/*
 * I2C.h
 *
 * Created: 28/11/2016 9:07:13 AM
 *  Author: Me
 */ 


#ifndef I2C_H_
#define I2C_H_

#define I2C_WRITE 0
#define I2C_READ 1

float* getAcc(void);	
float* getGyro(void);
float* getMag(void);
void ndof_init(void);
void ndof_update(void);
void ai2c_write_data(int, char, char);
void i2c_read_count(int, char, int, char*);
void init_adxl345(void);
void read_adxl345(void);
void init_itg3200(void);
void read_itg3200(void);
void init_hmc5843(void);
void read_hmc5843(void);

#endif /* I2C_H_ */


//http://www.hobbytronics.co.uk/hi-tech-c-i2c-master