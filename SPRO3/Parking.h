/*
 * Parking.h
 *
 * Created: 21/11/2016 11:07:57 AM
 *  Author: Me
 */ 


#ifndef PARKING_H_
#define PARKING_H_

#define irChangeThreshhold 5 //Change in ir sensor distance accepted as 'rapid'

void doGeneralPark(void);
void doAlignedPark(int);
void doDefinedAndAlignedPark(double);
int alignWithObstacle();
double locateParkingSpace(double);
int isOriented(double*);
int isWithinBounds(double *orientations);


#endif /* PARKING_H_ */