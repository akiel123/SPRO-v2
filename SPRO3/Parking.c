/*
 * ParkingAlgorithms.c
 *
 * Created: 21/11/2016 11:02:11 AM
 *  Author: Me
 */ 

#include "HardwareControl.h"
#include "Maneuvering.h"
#include "Safety.h"
#include "Parking.h"
#include <math.h>

void doGeneralPark(void){
	int sdSensor = alignWithObstacle(); //Smallest distance sensor
	doAlignedPark(sdSensor);
}
void doAlignedPark(int sdSensor){
	double distance = GetIRData(sdSensor);
	if(distance > 10) shiftDistanceFrontBack(distance - 2);
	else if(distance > 5) shiftDistanceBack1(distance - 2);
	double parkingSpace = locateParkingSpace(GetIRData(sdSensor));
	doDefinedAndAlignedPark(parkingSpace);
}
void doDefinedAndAlignedPark(double parkingSpace){ //Assumes the entire car is within the back obstacle
	if(parkingSpace < carLength + parkingMargin){
		ERROR();
	}
	double last1 = GetIRData(IR3);
	turnMiddle();
	SetCommand(1, 0, 0);
	//Drive the car to the edge...
	while(last1 - GetIRData(IR3) < irChangeThreshhold){
		yield();
	}
	//...  + margin of the obstacle
	resetDistance();
	while(getDistance() < wSideSens3 + parkingMargin / 2){
		yield();
	}
	shifDistanceLimitedSpace(carWidth, parkingSpace - carLength - parkingMargin / 2);
}

int alignWithObstacle(){ //returns the sensor ID, of the sensor closest to an obstacle
	double *sensorData = GetObstacleOrientationD(); //Store angle
	
	int orientator = 0;
	if((orientator = isOriented(sensorData))) return orientator; //if angle is 0, we are parallel and done aligning
	
	int mdir = 1;
	int mturn = 1;
	if(*(sensorData + 0) > *(sensorData + 2)) mdir = 1; //back is closer than front
	else mdir = -1;	//front is closer
	
	turnDirection(mturn);
	SetCommand(mdir, mturn, 0);
	
	while((orientator = isOriented(sensorData)) == 0){
		switch(isWithinBounds(sensorData)){ //calculate direction
			case -2: mdir = 0; mturn = 0; //All sensors are too close
			ERROR();
			break;
			case -1: mdir = 1; mturn = -1; //All sensors are too far
			break;
			case 1: mdir = -1; mturn = -1; //only front sensor is too far
			break;
			case 2: mdir = 1; mturn = -1; //only back sensor is too far
			break;
			case 3: mdir = -1; mturn = 1; //only front sensor is too close
			break;
			case 4: mdir = 1; mturn = 1; //only back sensor is too close
			break;
			case 5: mdir = 1; mturn = 1; //only front and back are too close
			break;
			default: //all sensors are in range
			break;
		}
		if(mturn != getCurrentTurnDirection()){
			//breaK();
			turnDirection(mturn);
		}
		SetCommand(mdir, mturn, 0);
	}
	int shortestSensor = IR1;
	double shortestDistance = GetIRData(IR1);
	if(GetIRData(IR2) < shortestDistance){
		shortestDistance = GetIRData(IR2);
		shortestSensor = IR2;
	}
	if(GetIRData(IR3) < shortestDistance){
		shortestDistance = GetIRData(IR3);
		shortestSensor = IR3;
	}
	if(orientator == -1){
		ERROR();
		return -1;
	}
	return shortestSensor;
}

double locateParkingSpace(double distance){ //Measures the parking space size buy driving back and forth measuring points on side sensor trigger
	double ir1last = GetIRData(IR1);
	double ir2last = GetIRData(IR2);
	double ir3last = GetIRData(IR3);
	double parkingSpace = 0;
	if((fabs(ir1last - distance) < PerfectAlignmentTolerance && fabs(ir3last - distance) < PerfectAlignmentTolerance)
	|| (!(fabs(ir1last - distance) < PerfectAlignmentTolerance) && !(fabs(ir3last - distance) < PerfectAlignmentTolerance))){
		//assume that the entirety of the car is within the front obstacle
		//Drive back until the back sensor passes the front obstacle, note position
		//Drive back iuntil back sensor enters the back obstacle, not distance between this position
		//And first position
		turnMiddle();
		SetCommand(-1, 0, 0);
		while(ir3last - GetIRData(IR3) < irChangeThreshhold){ //wait until distance on back sensor changes drastically
			ir3last = GetIRData(IR3);
			yield();
		}
		resetDistance();
		while(GetIRData(IR3) - ir3last < irChangeThreshhold){ //wait until distance on back sensor changes drastically
			ir3last = GetIRData(IR3);
			yield();
		}
		parkingSpace = getDistance();
	}
	if(fabs(ir1last - distance) < PerfectAlignmentTolerance && !(fabs(ir3last - distance) < PerfectAlignmentTolerance)){
		//assume that only the front of the car is within the front obstacle
		//Drive back until the back front sensor or back sensor passes the front or back obstacle, note position
		//Drive back iuntil back sensor enters the back obstacle, not distance between this position
		//And first position
		turnMiddle();
		SetCommand(-1, 0, 0);
		int doneFront = 0;
		int doneBack = 0;
		while(!doneFront || !doneBack){ //wait until distance on back sensor changes drastically
			if(GetIRData(IR3) - ir3last < irChangeThreshhold){
				if(!doneFront) resetDistance();
				else{
					parkingSpace = getDistance() + wSideSens13;
				}
				doneBack = 1;
			}
			if(ir1last - GetIRData(IR1) < irChangeThreshhold){
				if(!doneBack) resetDistance();
				else{
					parkingSpace = wSideSens13 - getDistance();
				}
				doneFront = 1;
			}
			ir1last = GetIRData(IR1);
			ir3last = GetIRData(IR3);
			
			yield();
		}
	}
	if(fabs(ir1last - distance) < PerfectAlignmentTolerance && !(fabs(ir3last - distance) < PerfectAlignmentTolerance)){
		//assume that only the back of the car is within the front obstacle
		//Drive forward until the back front sensor or back sensor passes the front or back obstacle, note position
		//Drive back until back sensor enters the back obstacle, not distance between this position
		//And first position
		turnMiddle();
		SetCommand(1, 0, 0);
		int doneFront = 0;
		int doneBack = 0;
		while(!doneFront || !doneBack){ //wait until distance on back sensor changes drastically
			if(GetIRData(IR1) - ir1last < irChangeThreshhold){
				if(!doneFront) resetDistance();
				else{
					parkingSpace = getDistance() + wSideSens13;
				}
				doneBack = 1;
			}
			if(ir3last - GetIRData(IR3) < irChangeThreshhold){
				if(!doneBack) resetDistance();
				else{
					parkingSpace = wSideSens13 - getDistance();
				}
				doneFront = 1;
			}
			ir1last = GetIRData(IR1);
			ir3last = GetIRData(IR3);
			
			yield();
		}
		resetDistance();
		while(GetIRData(IR2) - ir2last < irChangeThreshhold){ //wait until distance on back sensor changes drastically
			ir2last = GetIRData(IR2);
			yield();
		}
		parkingSpace = getDistance();
	}
	return parkingSpace;
}

int isOriented(double *orientations){ //returns
	if(fabs(*(orientations + 0) - *(orientations + 1)) < PerfectAlignmentTolerance) return 1;
	if(fabs(*(orientations + 1) - *(orientations + 2)) < PerfectAlignmentTolerance) return 2;
	if(fabs(*(orientations + 2) - *(orientations + 0)) < PerfectAlignmentTolerance) return 3;
	return 0;
}
int isWithinBounds(double *orientations){
	int isLess1 = fabs(*(orientations + 0)) < lowerLimitSens1;
	int isLess2 = fabs(*(orientations + 1)) < lowerLimitSens2;
	int isLess3 = fabs(*(orientations + 2)) < lowerLimitSens3;
	int isMore1 = fabs(*(orientations + 0)) < upperLimitSens1;
	int isMore2 = fabs(*(orientations + 1)) < upperLimitSens2;
	int isMore3 = fabs(*(orientations + 2)) < upperLimitSens3;
	if(isMore1 && isMore2 && isMore3) return -1;
	if(isLess1 && isLess2 && isLess3) return -2;
	if(isMore1 && isMore3) return 5;
	if(isMore1) return 1;
	if(isMore3) return 2;
	if(isLess1) return 3;
	if(isLess3) return 4;
	return 0;
}
