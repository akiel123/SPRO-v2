/*
 * HardwareControl.h
 *
 * Created: 11/10/2016 15:32:55
 *  Author: manke
 */ 


#ifndef HARDWARECONTROL_H_
#define HARDWARECONTROL_H_

#define ibr 308.25
#define obr 328.25
#define ifr 332.158
#define ofr 312.408
#define wSideSens1 4.5 //Distance from front of car, to middle of sideSens1
#define wSideSens2 23 //Distance from front of car, to middle of sideSens2
#define wSideSens3 2.5 //Distance from back of car, to middle of sideSens3
#define wSideSens13 37 //Distance between sideSens1 and sideSens3
#define lowerLimitSens1 15
#define lowerLimitSens2 15
#define lowerLimitSens3 15
#define upperLimitSens1 70
#define upperLimitSens2 70
#define upperLimitSens3 70
#define PerfectAlignmentTolerance 0.5
#define carLength 44
#define carWidth 24.5
#define parkingMargin 5
#define stopdirectvalue 600
#define stopcrossvalue 600

//Motor values
#define MOTOR_FORWARD (1 << 6)
#define MOTOR_BACKWARD (1 << 7)
#define MOTOR_PORT PORTD

#define STEERING_PORT PORTD
#define STEERING_PIN (1 << 5)

//Sensor port values
#define USSPort PORTB
#define USS1 (1 << 0)
#define USS2 (1 << 1)
#define USS3 (1 << 2)
#define USS4 (1 << 3)
#define IRPort PORTC
#define IR1 (1 << 0)
#define IR2 (1 << 1)
#define IR3 (1 << 2)
#define IR4 (1 << 3)
#define intervalLength 1 //Interval between each reed switch update on the wheel

//Breaking parameters
#define breakingPower 0.5 //How much power should be used to break, in percentage of full power
#define breakSpeed 0.1 //Accepted speed for categorization "not moving" in m/s

int getCurrentTurnDirection();
void SetCommand(double, int, int);
void resetDistance();
double getDistance();
void resetBodyAngle();
double GetBodyAngleDeg();
double GetBodyAngleRad();
double GetWheelPosition();
double GetVelocity();
double GetUSSData(int);
double GetIRData(int);
double * GetObstacleOrientationR();
double * GetObstacleOrientationD();

//USS functions
void USSReadNext();
void init_interrupt();
void SensorInit();
void USSInitMeas(char sensorID);
void USSHandleRead();

#endif /* HARDWARECONTROL_H_ */