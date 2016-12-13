/*
 * Safety.h
 *
 * Created: 21/11/2016 11:07:00 AM
 *  Author: Me
 */ 


#ifndef SAFETY_H_
#define SAFETY_H_

#define CriticalSafetyDistance 1;

enum USSHandleCase {finishingPark, findingSpot};

int HasReachedWall(void);
void USSHandle(double);
void yield(void);
void ERROR(void);



#endif /* SAFETY_H_ */