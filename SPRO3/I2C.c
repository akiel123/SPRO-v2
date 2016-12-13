/*
 * I2C.c
 *
 * Created: 21/11/2016 11:44:32 AM
 *  Author: Andreas Kielsgaard
 * Some functions taken from https://chionophilous.wordpress.com/2012/02/10/connecting-to-sparkfuns-9dof-sensor-stick-i2c-access-to-adxl345-itg-3200-and-hmc5843/
 */
#define  ADXL345_ADDRESS (0xA6)
#define ADXL345_REGISTER_XLSB (0x32)
//Need to set power control bit to wake up the adxl345
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)
#define ADXL_DATA_FORMAT 0x31
#define ADXL_FULLRES 3
#define ADXL_RANGE_LSB 0
#define ADXL_RANGE_MSB 1
//Values for a +-2g setting
#define ADXL_XMIN 50
#define ADXL_XMAX 540
#define ADXL_YMIN -540
#define ADXL_YMAX -50
#define ADXL_ZMIN 75
#define ADXL_ZMAX 875
#define ADXL_XRANGE 490
#define ADXL_YRANGE 490
#define ADXL_ZRANGE 800

#define ITG3200_ADDRESS (0xD0)
//request burst of 6 bytes from this address
#define ITG3200_REGISTER_XMSB (0x1D)
#define ITG3200_REGISTER_DLPF_FS (0x16)
#define ITG3200_FULLSCALE (0x03 << 3)
#define ITG3200_42HZ (0x03)

#define HMC5843_ADDRESS (0x3C)
//First data address of 6 is XMSB.  Also need to set a configuration register for
//continuous measurement
#define HMC5843_REGISTER_XMSB (0x03)
#define HMC5843_REGISTER_MEASMODE (0x02)
#define HMC5843_MEASMODE_CONT (0x00)

#include "i2cmaster.h"
#include "I2C.h"
#include <stdio.h>
#include "usart.h"

float accelerometer_data[3];
float gyro_data[3]; //First is up down, second is for-/backwards, third sideways 
float magnetometer_data[3];
float velocity[3];
float rotation[3]; 

float* getAcc(void){//returns as acceleration in m/s^2 (Allegedly)
	float* result[3];
	for(int i = 0; i < 3; i++){
		result[i] = 0;
	}
	*result[0] = ((accelerometer_data[0] - ADXL_XMIN) - ADXL_XRANGE/2) / (ADXL_XRANGE) * 9.82;
	*result[1] = ((accelerometer_data[1] - ADXL_YMIN) - ADXL_YRANGE/2) / (ADXL_YRANGE) * 9.82;
	*result[2] = ((accelerometer_data[1] - ADXL_ZMIN) - ADXL_ZRANGE/2) / (ADXL_ZRANGE) * 9.82;
	
	return accelerometer_data;
}
float* getGyro(void){//returns as rad/s
	return gyro_data;
}
float* getMag(void){
	return magnetometer_data;
}

void ndof_init(void){
	
	PORTC &= ~(1 << PORTC5) | ~(1 << PORTC4); //Set PC4 and PC5 (SDA and SDL) as input
	
	i2c_init();
	for(int i = 0; i < 3; ++i) {
		accelerometer_data[i] = 0;
		magnetometer_data[i] = 0;
		gyro_data[i] = 0;
	}
	init_adxl345();
	init_hmc5843();
	init_itg3200();
}

void ndof_update(){
	//printf("Updating Ndof");
	read_adxl345();
	read_itg3200();
	//read_hmc5843();
	//printf("Done Updating Ndof");
	
	
}

void ai2c_write_data(int address, char reg, char data) {
	// Send output register address
	//printf("Writing Data starting\n");
	i2c_start_wait(address + I2C_WRITE); //Select unit to be adressed. I2C specifies that data will be written
	//printf("Done starting\n");
	i2c_write(reg); //Select register to transfer to
	//printf("Wrote Register\n");
	i2c_write(data); //Transfer data
	//printf("Wrote data\n");
	i2c_stop();
	//printf("Done\n");
}

void i2c_read_count(int address, char reg, int count, char* data) {
	//printf("Reading Data starting\n");
	i2c_start_wait(address + I2C_WRITE);
	//printf("Done starting, specifying register\n");
	i2c_write(reg);
	//printf("Starting again\n");
	i2c_start_wait(address + I2C_READ);
	//printf("Reading data: ");
	for(int i = 0; i < count - 1; i++){
		data[i] = i2c_readAck();
		//printf("%d, ", (int)(data[i]));
	}
	data[count - 1] = i2c_readNak();
	//printf("%d \n", (int)(data[count - 1]));	
	//printf("\n");
	//printf("Done reading data");
	i2c_stop();
	//printf("Done reading\n");	
}

void init_adxl345(void) {
	printf("Initializing adx1345\n");
	//Set range
	char setting = 0b00000000; //Set range to +-4g
	ai2c_write_data(ADXL345_ADDRESS, ADXL_DATA_FORMAT, setting);
	ai2c_write_data(ADXL345_ADDRESS, ADXL_REGISTER_PWRCTL, ADXL_PWRCTL_MEASURE);
	printf("Done\n");
}

void read_adxl345(void) {
	char bytes[6];
	for(int i = 0; i < 6; i++){
		bytes[i] = 0;
	}

	//read 6 bytes from the ADXL345
	i2c_read_count(ADXL345_ADDRESS, ADXL345_REGISTER_XLSB, 6, bytes);

	//now unpack the bytes
	for (int i=0;i<3;++i) {
		accelerometer_data[i] = (int)bytes[2*i] + (((int)bytes[2*i + 1]) << 8);
	}
}

void init_itg3200(void) {
	printf("Initializing itg3200\n");

	//Set DLPF to 42 Hz (change it if you want) and
	//set the scale to "Full Scale"
	ai2c_write_data(ITG3200_ADDRESS, ITG3200_REGISTER_DLPF_FS, ITG3200_FULLSCALE | ITG3200_42HZ);
	printf("Done\n");
}

void read_itg3200(void) {
	char bytes[6];
	for(int i = 0; i < 6; i++){
		bytes[i] = 0;
	}

	//read 6 bytes from the ITG3200
	i2c_read_count(ITG3200_ADDRESS, ITG3200_REGISTER_XMSB, 6, bytes);  //now unpack the bytes
	for (int i=0;i<3;++i) {
		gyro_data [i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
	}
}

void init_hmc5843(void) {
	printf("Initializing hmc5843\n");
	//set up continuous measurement
	ai2c_write_data(HMC5843_ADDRESS, HMC5843_REGISTER_MEASMODE, HMC5843_MEASMODE_CONT);
	printf("Done\n");
}

void read_hmc5843(void) {
	char bytes[6];
	for(int i = 0; i < 6; i++){
		bytes[i] = 0;
	}

	//read 6 bytes from the HMC5843
	i2c_read_count(HMC5843_ADDRESS, HMC5843_REGISTER_XMSB, 6, bytes);

	//now unpack the bytes
	for (int i=0;i<3;++i) {
		magnetometer_data[i] = (int)bytes[2*i + 1] + (((int)bytes[2*i]) << 8);
	}
}