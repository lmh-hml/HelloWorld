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
	
        float linear,angular;
        float joy_linear, joy_az;
        geometry_msgs::Twist twist;
		
		const int MIN_DIST, MAX_DIST;
				
		ros::NodeHandle nh;
		ros::Publisher  Twist_pub;
		ros::Subscriber Joy_listener;
		ros::Subscriber US_listener;
		
		void JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy);
		void SonicCallBack( const fyp::Int16Array::ConstPtr& ary);
};


JoyControl::JoyControl(): linear(1),angular(1),
                          MIN_DIST(25), MAX_DIST(100)
{
	Twist_pub=nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
	Joy_listener=nh.subscribe<sensor_msgs::Joy>("joy",10, &JoyControl::JoyCallBack,this);
	US_listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10, &JoyControl::SonicCallBack,this);
}


void JoyControl::JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy)
{
    joy_linear=joy->axes[ToFro];
    joy_az=joy->axes[Turn];		
}

void JoyControl::SonicCallBack( const fyp::Int16Array::ConstPtr& ary)
{
	int16_t dist = ary->data[Second];	
    linear= (float)(dist-MIN_DIST)/(float)(MAX_DIST - MIN_DIST);
    linear = linear>0 ? linear:0;
    ROS_INFO("SonicCallBack: data received: %d", dist);

}



int main(int argc, char** argv)
{
	ros::init(argc,argv,"Jostick_control");
	JoyControl jc;
    ros::Rate hertz(20);
	ROS_INFO("Joystick control ready:");
	
    while(ros::ok())
    {
        if(jc.joy_linear>=0)
        {
        jc.twist.linear.x=jc.joy_linear * jc.linear;
        }
        else
        {
            jc.twist.linear.x=jc.joy_linear *0.5;
        }
        jc.twist.angular.z=jc.joy_az * jc.angular;
        jc.Twist_pub.publish(jc.twist) ;
      //  ROS_INFO("Published linear @ %f, az @ %f ,linear @ %f", jc.twist.linear.x, jc.twist.angular.z, jc.linear);
        ros::spinOnce();
        hertz.sleep();
    }


}





