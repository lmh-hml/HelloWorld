#ifndef CONTROLUTIL_H
#define CONTROLUTIL_H

#include "Control.h"

/*
	24/8/16: this file defines the data processing and update functions.
*/

//calculate the correct linear velocity by interpolation
float Control::calc_linearVel(int16_t& sensor_rd, const int& MAX_LIMIT, const int& MIN_LIMIT)
{
	int dist= sensor_rd>MAX_DIST ? MAX_DIST:sensor_rd;
    float linear = (float)(dist-MIN_LIMIT)/(float)(MAX_LIMIT-MIN_LIMIT);
    linear = linear>0 ? linear:0;
    
    return linear;
}

//Check if sides are blocked TODO:Typedef the right/left, current arguments too vague
bool Control::sidesBlocked( int16_t right, int16_t left)
{
	bool blocked = (right<MIN_SIDES || left<MIN_SIDES);
	if(blocked)
	{
		ROS_INFO("Sides blocked: right:%d, left:%d, MIN_SIDES: %d", right, left, MIN_SIDES);
	}
	return blocked;
}

//Determine the robot's movement state
MoveState Control::getDirection(float joy_lx, float joy_az)
{
	if( joy_az > 0.1 || joy_az<-0.1){ return MoveState::Turning;}
	else if( joy_lx >0.1 && joy_lx<= 1.0 ){ return MoveState::Forward; }
	else if( joy_lx < -0.1 && joy_lx>=-1.0) { return MoveState::Backward; }
	return MoveState::Still;
}

//The update. It's a state machine!
void Control::Update()
{
	ROS_INFO("Updatng");
	
	moveState = getDirection(joy_lx, joy_az);
	
	
	switch( moveState )
	{
		case MoveState::Turning :		
			ROS_INFO("Turning");
       		angular=1;linear=0;
			break;
        
        case MoveState::Forward :
        	ROS_INFO("Forwards");
        	linear= sidesBlocked(sensor[FrRight],sensor[FrLeft]) ? 0:calc_linearVel(sensor[Front], MAX_DIST, MIN_DIST); break;
        	
        case MoveState::Backward :
        	ROS_INFO("Backwards");
        	linear= sidesBlocked(sensor[BkRight],sensor[BkLeft]) ? 0:calc_linearVel(sensor[Back], MAX_DIST, MIN_DIST); break;
        
        case MoveState::Still    :
        	ROS_INFO("Intentionally stopped");
        default:
        	ROS_INFO("Staying still");
			linear=0; angular = 0;
			break;
	}
	//ROS_INFO("Publishing: lv:%f, jlx:%f, av:%f, jaz%f", linear, joy_lx, angular, joy_az);
    publishTwist(linear,joy_lx,angular,joy_az);
}
#endif
