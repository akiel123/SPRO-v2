/*
 * HardwareControl.c
 *
 * Created: 11/10/2016 15:33:09
 *  Author: manke
 */ 

#define F_CPU 16E6
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include "i2cmaster.h" //Enable communication via i2c
#include "usart.h" //Enable connection between PC and microcontroller
#include <avr/interrupt.h>
#include <util/delay.h>
#include "MathExtra.h"
#include "HardwareControl.h"
#include "I2C.h"



double targetVelocity = 0; // < 0: backwards, > 0: forwards, == 0: freedrive
int direction = 0; //1 = force towards right, 0 = do nothing, -1 = force towards left 
int breaking = 0; //Break if true
int currentTurnDirection = 0;

volatile double currentVelocity;
volatile double vx = 0, vy = 0, vz = 0;
volatile double distanceDriven = 0;
volatile double angularVelocity = 0;
volatile double angle = 0;
volatile int j = 0;
volatile int i = 0;

volatile int time1 = 0;
volatile int t = 0;
volatile int pwmTime = 0;
volatile int pwmMotor;
volatile int pwmSteering;
volatile int pwmMax;

enum side {left, right, none};
volatile enum side ussTargetedSide = none;
volatile enum side ussMeasuredSide = none;
volatile int isMeasuring = 0;
volatile int ussSensorIDNextIndex = 0;
static int ussSensorIDs[4] = {USS1, USS2, USS3, USS4};



int getCurrentTurnDirection(){
	return currentTurnDirection;	
}
void SetCommand(double fspeed, int fdirection, int fbreaking){
	pwmMotor = fspeed * pwmMax ;	//REMEMBER TO CHANGE THIS - PUTS MOTOR AT FULL SPEED A LOT OF PLACES
	direction = fdirection;
	breaking = fbreaking;
	//If breaking, pwm will be overriden, and be 0 after breaking is done
}
void resetDistance(){
	distanceDriven = 0;
}
double getDistance(){
	return distanceDriven;
}
void resetBodyAngle(){
	angle = 0;
}
double GetBodyAngleDeg(){ //Returns between 0 and 360
	return angle;
} 
double GetBodyAngleRad(){ //Returns between 0 and 2*PI
	return angle;
}
double GetWheelPosition(){
	return 0;
}


double GetVelocity(){return 1;}

double GetUSSData(int id){
	PORTD |= 0x40; //turn on PD6 (Trig Pin) for 10 ?
	_delay_us(10);
	PORTD &= ~0x40; //turn off PD6 (Trig Pin)
	i = 0; //reset timer
	int meassurementTime = 2;
	_delay_ms(meassurementTime);
	int res = (int)i;
	if(res < 150) return 0;
	else if(res > 6000) return -1;
	return 1;
}


double GetIRData(int id){
	ADMUX &= (0xf0) | id; //set the desired channel and internal reference
	ADCSRA |= (1<<ADSC); //start the conversion
	while ( (ADCSRA & (1<<ADSC)) ); //wait for the conversion to complete
	int resRaw = ADC; //return the result //Should be uint16_t
	
	switch(id){
	case IR1:
		break;
	case IR2:
		break;
	case IR3:
		break;
	case IR4: 
		if(resRaw < 1000) return 100; //Distance too long to meas
		if(resRaw > 2600) return -1; //Distance too short ot meas
		//Return measured distance, calculated by a formula derived
		//by linear regression on test results
		return 1/692 * (46101 - sqrt(692000 * 2700- 683252799));
	default:
		return -2;
	}
	return 0;
}




double * GetObstacleOrientationR(){
	static double result[6];
	double sens1 = GetIRData(IR1);
	double sens2 = GetIRData(IR2);
	double sens3 = GetIRData(IR3);
	double dsens1 = sens2 - sens1;
	double dsens2 = sens3 - sens2;
	double dsens3 = sens3 - sens1;
	
	
	double angle1 = acos(wSideSens1/dsens1);
	double angle2 = acos(wSideSens2/dsens2);
	double angle3 = acos(wSideSens3/dsens1);
	
	if(dsens1 < 0) angle1 = 180 * Deg2Rad - angle1;
	if(dsens2 < 0) angle3 = 180 * Deg2Rad - angle2;
	if(dsens3 < 0) angle2 = 180 * Deg2Rad - angle3;
	
	result[0] = dsens1;
	result[1] = dsens2;
	result[2] = dsens3;
	result[3] = angle1;
	result[4] = angle2;
	result[5] = angle3;
	
	return result;
}
double * GetObstacleOrientationD(){
	double * rad = GetObstacleOrientationR();
	double location;
	double * deg = &location;
	for(int i = 0; i < 6; i++){
		*(deg + i) = rad[i] * Rad2Deg;
	}
	return deg;
}

ISR (INT0_vect){ //Read registered on left ultra sound sensor
	t = time1; //Stores exact time
	ussMeasuredSide = left;
	USSHandleRead ();
}

ISR (INT1_vect){ //Read registered on right USS
	t = time1;
	ussMeasuredSide = right;
	USSHandleRead();
}

ISR (TIMER0_COMPA_vect){ //Triggers every 1us
	//Uss section
	//Call uss sensor every reasonable interval. At least 2ms between,
	//to make sure previous signal has faded
	time1++; //counts in microseconds
	if(time1 > 6000){
		time1 = -2; //make sure that time doesnt start incrementing before read is ready
		USSReadNext();
	}
	else if(time1 < 0){
		time1 = -2;
	}
	
	//Pwm section
	if(breaking){
		if(currentVelocity > breakSpeed){
			pwmMotor = -(pwmMax * breakingPower);
		}
		else if(currentVelocity < -breakSpeed){
			pwmMotor = pwmMax * breakingPower;
		}
		else breaking = 0;
	}
	
	if(pwmMotor > 0){ //If target velocity is positive
		if(pwmTime == pwmMotor){ //turn off output
			MOTOR_PORT = MOTOR_PORT ^ MOTOR_FORWARD;	
		}
		else if(pwmTime == pwmMax){//turn on output
			MOTOR_PORT |= MOTOR_FORWARD; 
		}
	} else { //If target velocity is negative
		if(pwmTime == -pwmMotor){//turn off output
			MOTOR_PORT = MOTOR_PORT ^ MOTOR_BACKWARD;
		}
		else if(pwmTime == pwmMax){//turn on output
			MOTOR_PORT |= MOTOR_BACKWARD;
		}
	}
	
	if(pwmTime == pwmSteering){//turn off output
		STEERING_PORT = STEERING_PORT ^ STEERING_PIN;
	}
	if(pwmTime == pwmMax){//turn on output
		STEERING_PORT |= STEERING_PIN;
		pwmTime = 0;
	}
	pwmTime++;
}

ISR (TIMER1_COMPA_vect){ //Called every 10.24ms
	ndof_update();
	float* accelration = getAcc();
	vx += accelration[0] * 0.01024;
	vy += accelration[1] * 0.01024;
	vz += accelration[2] * 0.01024;
}

void USSReadNext(){
	USSInitMeas(ussSensorIDs[ussSensorIDNextIndex]);
	ussSensorIDNextIndex++;
	if(ussSensorIDNextIndex >= 4) ussSensorIDNextIndex = 0;
}

void init_interrupt(){

	//timer 1
	TCCR0A |= (1 << WGM01); // Set the Timer Mode to CTC
	OCR0A = 0x01; // Set the value to count to = 1+1 //cycle = 1 us, with prescaler 68
	TIMSK0 = (1 << OCIE0A); //set the ISR COMPA vect
	
	//timer 2
	TCCR1A |= (1 << WGM01); // Set the Timer Mode to CTC
	OCR1A = 0xF9; // Set the value to count to = 159+1 //cycle = 10.24ms , with prescaler 1024
	TIMSK1 = (1 << OCIE0A); //set the ISR COMPA vect
	
	//Interrupt 1
	EICRA |= (1 << ISC01) | (1 << ISC00); //set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0); //Turns on INT0
	
	//Interrupt 2
	EICRA |= (1 << ISC11) | (1 << ISC10); //set INT1 to trigger on ANY logic change
	EIMSK |= (1 << INT1); //Turns on INT1
	sei(); //enable interrupts
	
	TCCR0B |= (1 << CS01); // start the timer with prescaler 8
	TCCR1B |= (1 << CS02) | (1 << CS00); // start the timer with prescaler 8
}

void SensorInit(){
	uart_init(); // open the communication to the microcontroller
	io_redirect(); // redirect input and output to the uart
	init_interrupt();
	
	//IR specific
	ADMUX |= (1<<REFS0); //select Vref = AVcc
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)| (1<<ADEN); //set prescaler to 128 (ADPS) and turn on the ADC module (ADEN)
	
	//US specific
	DDRB = 0xFF; //set B ports to output
	DDRC = 0xF0; //set C ports to input
	PORTB = 0x00; //set output of all B ports to 0 (no output)
	PORTC = 0x3F; //set input of all C ports to 1 (no input)
	
	DDRD &= ~(1 << DDD2); //clear the PD2 pin (INT0)
	DDRD &= ~(1 << DDD3); //clear the PD3 pin (INT1)
	PORTD |= (1 << PORTD2); //turn on the Pull-up for PD2 (INT0)
	PORTD |= (1 << PORTD3); //turn on the Pull-up for PD2 (INT1)
}

void USSInitMeas(char sensorID){ //Ultrasonic Sensor Read Function
	switch(sensorID){
		case USS1:
		case USS3:
			ussTargetedSide = left;
			break;
		case USS2:
		case USS4:
			ussTargetedSide = right;
			break;
		default: break; //If default, unregistered sensor id is calling	
	}
	if(!isMeasuring){ //initiate ultrasonic pulse
		PORTB |= sensorID; //turn on PB0 (Trig Pin) for 10 us
		_delay_us(10);
		PORTB &= ~sensorID; //turn off PB0 (Trig Pin)
		isMeasuring = 1;
	}
	time1 = 0;
}

void USSHandleRead(){
	if(ussTargetedSide == ussMeasuredSide){
		ussTargetedSide = none;
		isMeasuring = 0;
		if(t > 150 && t < 6000){ //only consider reasonable measurements, 150~6000 (min/max)
			if(t > stopcrossvalue){
				//Handle object measured outside critical range
			}
			if(t <= stopcrossvalue){   //Print stop if distance is below threshold
				//Handle object measured inside critical range
			}
			if((j & 0b10000000)==0b10000000){ //Sensor side 1
				j = j ^ 0b10000000; //reset print state
			}
			else{ //Sensor side 2
				j = j ^ 0b01000000; //reset print state
			}
		}
	}
}
