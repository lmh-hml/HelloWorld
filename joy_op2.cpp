#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fyp/Int16Array.h>


enum Joystick { LeftRight = 0, ToFro, Turn, Lever, SmallLeftRight, SmallToFro };
enum Sensors  { First=0, Second , Third };

class JoyControl
{
	public:
		
		JoyControl();
	
        float lx,ly,lz;
		const float ax,ay,az;
        geometry_msgs::Twist twist;
		
		const int MIN_DIST, MAX_DIST;
				
		ros::NodeHandle nh;
		ros::Publisher  Twist_pub;
		ros::Subscriber Joy_listener;
		ros::Subscriber US_listener;
		
		void JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy);
		void SonicCallBack( const fyp::Int16Array::ConstPtr& ary);
};


JoyControl::JoyControl(): lx(0.5),ly(0), lz(0),
						  ax(0),ay(0), az(1), 
						  MIN_DIST(20), MAX_DIST(300)
{
	Twist_pub=nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
	Joy_listener=nh.subscribe<sensor_msgs::Joy>("joy",10, &JoyControl::JoyCallBack,this);
	US_listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10, &JoyControl::SonicCallBack,this);
}


void JoyControl::JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy)
{
	geometry_msgs::Twist twist;
	twist.linear.x=joy->axes[ToFro] * lx;
	twist.angular.z=joy->axes[Turn]  * az;
    ROS_INFO("JoyCallBack: Lx , Az: %f, %f", lx, az);
		
}

void JoyControl::SonicCallBack( const fyp::Int16Array::ConstPtr& ary)
{

	int16_t dist = ary->data[Second];
    ROS_INFO("SonicCallBack: data received: %d", dist);
	
    if( dist < MIN_DIST )
	{
		lx=0;
	}else
	{
		lx=0.5;
	}
}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"Jostick_control");
	JoyControl jc;
    ros::Rate hertz(20);
	ROS_INFO("Joystick control ready:");
	
    while(ros::ok())
    {
        jc.Twist_pub.publish(jc.twist) ;
        ROS_INFO("Published lx @ %f, az @ %f ", jc.twist.linear.x, jc.twist.angular.z);
        ros::spinOnce();
        hertz.sleep();
    }


}





