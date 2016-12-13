/*
 * SPRO3.c
 *
 * Created: 7/12/2016 11:34:37 PM
 * Author : Me
 */ 
#define F_CPU 1600000UL

#include <avr/io.h>
#include <stdio.h>
#include "usart.h"
#include <util/delay.h>
#include "I2C.h"

void testI2C(void);

int main(void)
{
    
	testI2C();
	while (1) 
    {
    }
}

void testI2C(void){
	printf("Starting Initializations\n");
	uart_init();
	io_redirect();
	ndof_init();
	uart_init();
	io_redirect();
	printf("Done Initializing\n");
	int count = 0;
	while(1){
		ndof_update();
		float* acc = getAcc	();
		float* gyr = getGyro();
		float* mag = getMag	();
		//printf("Update %d: \n", count);
		printf("Accelerometer: %f, %f, %f\n", acc[0], acc[1], acc[2]);
		//printf("Gyroscope    : %f, %f, %f\n", gyr[0], gyr[1], gyr[2]);
		//printf("Magnetometer : %f, %f, %f\n", mag[0], mag[1], mag[2]);
		//printf("\n");	
		//_delay_ms(1000);--
		count++;
	}	
}

void testMotor(){
	//Set pwm values in HardwareControl.c
	init_interrupt();
}