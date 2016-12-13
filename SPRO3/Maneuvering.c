/*
 * ControlAlgorithms.c
 *
 * Created: 11/10/2016 15:23:44
 *  Author: manke
 */ 


#include <math.h>
#include "MathExtra.h"
#include "Maneuvering.h"
#include "Safety.h"
#include "HardwareControl.h"

int currentTurnDirectionM = -2;

	
void shiftDistanceBack1(double distance){
	//Prepare variables
	double extraDistance = 0;
	double angle = 0;

	//Check if straight backing up is needed and prepare variables
	if (distance > ibr + obr){
		angle = M_PI_2; //90 degrees
		extraDistance = distance - (ibr + obr);
	}
	else angle = acos(1 - (distance / (ibr + obr)));

	//Start the turning based on calculated variables
	shiftDistanceBack2(angle, extraDistance);
}
void shiftDistanceBack2(double angle, double extraDistance){
	//Prepare variables
	int turnRadiusExceeded = extraDistance > 0;
	resetBodyAngle();
		
	breaK();
	turnRight();
	
	//Back while turning right
	SetCommand(-1, 1, 0);
	double a1 = fmod((- angle + rrad), rrad) + 0.01;
	double a2 = fmod((- angle + rrad), rrad) - 0.01;
	if (fmod((- angle + rrad), rrad) < 0.01) a2 = 0;
	if (fmod((- angle + rrad), rrad) > rrad - 0.01) a1 = rrad;
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < a2) yield(); //Until angle is more than required angle
	breaK();

	if (turnRadiusExceeded){ //If straight backing is necessary, do it now
		backUpDistance(extraDistance);
		breaK();
	}

	turnLeft();
	//Drive back while turning left
	SetCommand(-1, -1, 0);
	a1 = 0.02;
	a2 = 0;
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < fmod(a2,2 * M_PI)) yield(); //until back at start angle
	breaK();
	turnMiddle();
}

void shiftDistanceBackOffset(double distance, double offsetAngle) //Shifts backwards with an offset angle
{
	//Move angle within right range
	offsetAngle = fmod(offsetAngle + rrad, M_PI); //offset angle should be between 0 and 180 degrees
	//Make a copy of offsetAngle in degrees
	double offAngDeg = offsetAngle * Rad2Deg;

	int directionToGoal = 1; //is the goal infront of car or behind car
	if (distance < 0) directionToGoal= -1;

	//Defines whether the front is pointing towards the goal (or the back). If it is pointing towards, a forward approach will be taken to the goal
	//int approachForwards = distance > 0; //Apparently obsolete - await confirmation
	
	//Convert to 1 or 0 integer
	//int approachingDirection = System.Convert.ToInt32(approachForwards);

	//Prepare variables
	double extraDistance = 0; //How much forward driving is needed
	//double angle = 0; obsolete
	double anglePhi = 0; //How many degrees first turn will be
	resetBodyAngle();
	
	double minimum1PointDistance = (1 - fabs(cos(offsetAngle))) * (obr);	//The distance moved along y-axis if the car directly aligns
																		//itself with the desired axis, meassured at back wheel
																		//closest to the goal
	double maxCurveDistance = obr - (1 - sin(offsetAngle)) * ibr + ibr;	//The distance if car turns
																		//to an angle perpendicular to the desired angle, and then directly
																		//to a parallel angle. Maximum distance before emplying straight
																		//driving. Measured at back wheel closeset to the goal
	int initialDirection = 1; //Direction to do the first part of the turn (relative to the direction of the goal. 1 for same, -1 for opposite.


	if (maxCurveDistance < fabs(distance)) //If straight driving is to be employed, calculate distance of this.
	{
		anglePhi = M_PI_2 - offsetAngle; //Angle to turn to go perpendicular
		extraDistance = fabs(distance) - maxCurveDistance;	//Distance that would not be covered by the turn, and which should be driven straight when
		//perpendicular
	}
	else if (fabs(distance) > minimum1PointDistance)
	{	//Angle it has to turn towards wall, getting a total displacement larger than the minimum1PointDistance
		anglePhi = fmod(fabs(acos((ibr * cos(offsetAngle) + obr - fabs(distance)) / (ibr + obr)) - offsetAngle), M_PI_2);
	}
	else
	{	//Angle it has to turn away from wall, getting a smaller total displacement than the minimum1PointDistance
		anglePhi = -1 * fmod(fabs(acos((ibr * cos(fmod(offsetAngle,M_PI)) + obr - fabs(distance)) / (ibr + obr)) + offsetAngle), M_PI_2);
		initialDirection = -1; //Start by moving away from wall
	}

	int initialTurnDirection = 1;
	//Initial turning direction. Depending on which side of the car is closest to the goal
	if (offAngDeg < 90) initialTurnDirection = -1;

	breaK();

	if (initialTurnDirection == -1) turnLeft();
	else turnRight();

		//Back while turning
	SetCommand(directionToGoal * initialDirection, initialTurnDirection, 0);
	
	double addAng = 0; //The angle to add to startangle (0). When debugging, note, that if it actually has to turn away from the given y-axis, anglePhi is already negative
	
	if (offAngDeg < 90 && directionToGoal == 1) addAng = anglePhi; //it has to turn towards the relative y-axis, from positive x-axis (from 0-90 -> 90)
	else if (90 < offAngDeg && directionToGoal == 1) addAng = anglePhi * -1;	//turn towards +y-axis from -x-axis (180-90 -> 90)
																				//+y-axis (from 90-180 -> 180
	else if (offAngDeg < 90 && directionToGoal == -1) addAng = anglePhi * -1; //it has to turn towards the -y-axis from the +x-axis (360-270 -> 270)
	else if (90 < offAngDeg && directionToGoal == -1) addAng = anglePhi; // turn towards -y-axis from -x-axis (180-270 -> 270)
	addAng *= -1;

	double a1 = fmod((addAng + rrad), rrad) + 0.01f;
	double a2 = fmod((addAng + rrad), rrad) - 0.01f;
	if (fmod((addAng + rrad), rrad) < 0.01f) { a2 = 0; a1 = 0.02f; } //make sure that a1 doesnt go below zero
	if (fmod((addAng + rrad), rrad) > rrad - 0.01f) { a1 = rrad; a2 = rrad - 0.02f; } //make sure that a2 doesnt go above rrad.
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < fmod(a2, 2 * M_PI)) yield(); //Until angle is between the two angles

	breaK();


	if (extraDistance != 0)
	{//If staight backing is necessary, do it now
		backUpDistance(extraDistance);
		breaK();
	}

	turnLeft();

	//set add ang
	if (offAngDeg < 90 && directionToGoal == 1) addAng = offsetAngle * -1; //Sorry, dont feel like explaining <3. See similar segment above.
	else if (90 < offAngDeg && directionToGoal == 1) addAng = M_PI - offsetAngle;
	else if (offAngDeg < 90 && directionToGoal == -1) addAng = offsetAngle;
	else if (90 < offAngDeg && directionToGoal == -1) addAng = (M_PI - offsetAngle) * -1;
	addAng *= -1;
	//Drive towards goal, while turning towards parallel to non-offset angle
	SetCommand(directionToGoal, initialTurnDirection * -1, 0);
	
	a1 = fmod((addAng + rrad), rrad) + 0.01f;
	a2 = fmod((addAng + rrad), rrad) - 0.01f;
	if (fmod((addAng + rrad), rrad) < 0.01f) { a2 = 0; a1 = 0.02f; }
	if (fmod((addAng + rrad), rrad) > rrad - 0.01f) { a1 = rrad; a2 = rrad - 0.02f; }
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < fmod(a2, 2 * M_PI)) yield(); //until between a1 and a2

	breaK();
	turnMiddle();
}

void shiftDistanceFront1(double distance)
{
	//Prepare variables
	double extraDistance = 0;
	double angle = 0;

	//Check if straight driving is needed and prepare variables
	if (distance > (ibr + obr) * 2)
	{
		angle = 90 * Deg2Rad;
		extraDistance = distance - (ibr + obr) * 2;
	}
	else angle = acos(1 - (distance / ((ibr + obr) * 2)));
		
	//Start the turning based on calculated variables
	shiftDistanceFront2(angle, extraDistance);
	yield();
}
void shiftDistanceFront2(double angle, double extraDistance)
{
	//Enable automatic control

	//Prepare variables
	int turnRadiusExceeded = extraDistance > 0;
	resetBodyAngle();

	breaK();

	turnRight();
	//Drive forward while turning right
	SetCommand(1, 1, 0);
	double a1 = fmod((angle + rrad), rrad) + 0.01f;
	double a2 = fmod((angle + rrad), rrad) - 0.01f;
	if (fmod((angle + rrad), rrad) < 0.01f) a2 = 0;
	if (fmod((angle + rrad), rrad) > rrad - 0.01f) a1 = rrad;
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < a2) yield(); //Until angle is more than required angle
	breaK();

	//If staight driving is necessary, do it now
	if (turnRadiusExceeded)
	{
		driveDistance(extraDistance);
		breaK();
	}

	turnLeft();		//Drive forward while turning left
	SetCommand(1, -1, 0);
	a1 = 0.02;
	a2 = 0;
	while (GetBodyAngleRad() > a1 || GetBodyAngleRad() < fmod(a2, 2 * M_PI)) yield(); //until back at start angle
	breaK();
}
void shiftDistanceBackFront(double distance)
{
	shiftDistanceBack1(distance * 0.5);
	shiftDistanceFront1(distance * 0.5);
	turnMiddle();
}
void shiftDistanceFrontBack(double distance)
{
	shiftDistanceFront1(distance * 0.5);
	shiftDistanceBack1(distance * 0.5);
	turnMiddle();
}
void shiftDistanceLimitedSpace(double distance, double spaceLimit)
{
	//Prepare variables
	double extraDistance = 0;
	double angle = 0;

	//Check if limit comforming is necessary
	if (spaceLimit > (ibr + obr))
	{
		angle = 90 * Deg2Rad;
		extraDistance = spaceLimit - (ibr + obr) * 0.5;
	}
	else angle = asin(spaceLimit / ((ibr + obr)));

	double distanceShifted = 0;
	double stepLength = (1 - cos(angle)) * (ibr + obr) + extraDistance;

	int maxSteps = 10; //for debugging purposes
	int i = 0;
	while (i < maxSteps) //Shift back and front at the maximum calculated angle until goal is reachable in next step.
	{
		if (distance - distanceShifted < stepLength * 2) break;
		shiftDistanceBack2(angle, extraDistance);
		distanceShifted += stepLength;
		
		shiftDistanceFront2(angle, extraDistance);
		distanceShifted += stepLength;
		i++;
	}
	shiftDistanceBack1((distance - distanceShifted) / 2);
	shiftDistanceFront1((distance - distanceShifted) / 2);
	yield();
	//int turnRadiusExceeded = 0; obsolete
	extraDistance = 0;
	angle = 0;

	//Check if straight driving is needed and prepare variables
	if (distance > (ibr + obr) * 2)
	{
		angle = 90 * Deg2Rad;
		//turnRadiusExceeded = 1;
		extraDistance = distance - (ibr + obr) * 2;
	}
	else angle = asin(1 - (distance / ((ibr + obr) * 2)));
}

void breaK(void)
{
	SetCommand(0, 0, 0); //Break
	while (GetVelocity() > breakSpeed) yield(); // until at acceptable speed
}
void turnLeft(void)
{
	currentTurnDirectionM = -2;
	SetCommand(0, -1, 0); //Turn to the left
	while (!(GetWheelPosition() == -1)) yield(); // until wheel is allmost fully turned
	currentTurnDirectionM = -1;
	
}
void turnMiddle(void)
{
	currentTurnDirectionM = -2;
	if(GetWheelPosition() > 0 ) SetCommand(0, -1, 0); //Turn towards the left
	else if(GetWheelPosition() < 0) SetCommand(0, 1, 0); //Turn towards the right
	while (!(GetWheelPosition() == 0)) yield(); // until wheel is in the middle
	currentTurnDirectionM = 0;
}
void turnRight(void)
{
	currentTurnDirectionM = -2;
	SetCommand(0, 1, 0); //Turn to the right
	while (!(GetWheelPosition() == 1)) yield(); // until wheel is allmost fully turned
	currentTurnDirectionM = 1;
	}
void backUpDistance(double distance)
{
	double distanceStart = getDistance();
	turnMiddle ();
	SetCommand(-1, 0, 0); //Back up	
	while (distanceStart - getDistance() < distance) yield(); // until an appropriate distance from starting point
}
void driveDistance(double distance)
{
	double distanceStart = getDistance();
	turnMiddle ();
	SetCommand(1, 0, 0); //Back up
	while (getDistance() - distanceStart < distance) yield(); // until an appropriate distance from starting point
}

void turnDirection(int directionIsRight){
	if(directionIsRight == 1) turnRight();
	else if(directionIsRight == 0) turnMiddle();
	else if(directionIsRight == -1) turnLeft();
}
