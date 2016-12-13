/*
 * Safety.c
 *
 * Created: 21/11/2016 11:03:05 AM
 *  Author: Me
 */ 
#include "Safety.h"

volatile enum USSHandleCase ussCase;
volatile int reachedWallFlag = 0;

int HasReachedWall(void){
	return reachedWallFlag;
}

void USSHandle(double distance){
	switch(ussCase){
		case findingSpot:
		break;
		case finishingPark:
		if(distance < 2){
			reachedWallFlag = 1;
		}
		break;
		
	}
}

void yield(void){ //do a manual interrupt
	
}
void ERROR(void){
	
}
