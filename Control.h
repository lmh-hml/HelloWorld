#ifndef CONTROL_H
#define CONTROL_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fyp/Int16Array.h>
#include <dynamic_reconfigure/server.h>
#include <fyp/ControlDistConfig.h>

/* 24/8/16: The node declaration
*/

enum Joystick { LeftRight = 0, ToFro, Turn, Lever, SmallLeftRight, SmallToFro };
enum Sensors  { FrLeft=0, Front , FrRight, BkRight, Back, BkLeft, Total };
enum MoveState { Still=0, Forward,Backward, Turning,ClockWise, AntiClockWise};

class Control
{
	public:
		
	Control();

	//The functions:
        void Update();
        MoveState getDirection(float joy_lx, float joy_az);
        float calc_linearVel(int16_t& sensor_rd, const int& MAX_LIMIT, const int& MIN_LIMIT);
    	bool sidesBlocked( int16_t right, int16_t left);
        void dynamicReconfig(fyp::ControlDistConfig &config, uint32_t level );
        void publishTwist(float linearX, float joylinear ,float 	angularZ, float joyAngular);
        
    private:
    		
    	float joy_lx, joy_az;
        int16_t sensor[6]={0,0,0,0,0,0};
	//Updated by local function
        float linear,linearBack, angular;
        MoveState moveState;
	//To be published
        geometry_msgs::Twist twist;
	//Control parameters
        int MIN_DIST, MAX_DIST,MIN_SIDES;
		
	//The ros stuff:		
	ros::NodeHandle nh;
	ros::Publisher  Twist_pub;
	ros::Subscriber Joy_listener;
	ros::Subscriber US_listener;
        dynamic_reconfigure::Server<fyp::ControlDistConfig> server;

		void JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy);
		void SonicCallBack( const fyp::Int16Array::ConstPtr& ary);
                         
};









#endif
