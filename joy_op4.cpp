#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fyp/Int16Array.h>

enum Joystick { LeftRight = 0, ToFro, Turn, Lever, SmallLeftRight, SmallToFro };
enum Sensors  { First=0, Second , Third };
enum MoveState { Still=0, Forward,BackWard, ClockWise, AntiClockWise};

class JoyControl
{
	public:
		
		JoyControl();
	
        float linear, angular;
        float joy_lx, joy_az;
        int16_t sensor[3];
        geometry_msgs::Twist twist;
		
        const int MIN_DIST, MAX_DIST,MIN_SIDES;
				
		ros::NodeHandle nh;
		ros::Publisher  Twist_pub;
		ros::Subscriber Joy_listener;
		ros::Subscriber US_listener;

        void update();
        void publishTwist(float linearX, float joylinear ,float angularZ, float joyAngular);

    private:
		void JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy);
		void SonicCallBack( const fyp::Int16Array::ConstPtr& ary);
                           // TODO: Sonic _BACK_callBack
};


JoyControl::JoyControl(): linear(0), angular(0),
                          MIN_DIST(30), MAX_DIST(70),MIN_SIDES(40)
{
	Twist_pub=nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
	Joy_listener=nh.subscribe<sensor_msgs::Joy>("joy",10, &JoyControl::JoyCallBack,this);
	US_listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10, &JoyControl::SonicCallBack,this);
}


void JoyControl::JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy)   //this collects relevant joystick msg
{
    joy_lx=joy->axes[ToFro];
    joy_az=joy->axes[Turn];		
}

void JoyControl::SonicCallBack( const fyp::Int16Array::ConstPtr& ary) //this collects info from front sensors
{
    sensor[0]    = ary->data[First];
    sensor[1]    = ary->data[Second];
    sensor[2]    = ary->data[Third];


}

 void JoyControl::update()
{
  //  int dist= sensor[Second]>MAX_DIST ? MAX_DIST:sensor[Second];
   // linear = (float)(dist-MIN_DIST)/(float)(MAX_DIST - MIN_DIST);

     int dist= sensor[Second]>MAX_DIST ? MAX_DIST:sensor[Second];
     linear = (float)(dist-MIN_DIST)/(float)(MAX_DIST - MIN_DIST);
     ROS_INFO("lx ready: %f",linear);


    if( joy_az > 0.1 || joy_az < -0.1 )    //if joystick is twisted
    {
        angular=1;
        linear=0;

    }
    else
    {
        angular = 0;

        if( joy_lx >0.1 && joy_lx<= 1.0 )   //if joystick is tilted forwards
        {
           linear = linear>0 ? linear:0;

        }else if(joy_lx< -0.1 && joy_lx>= -1.0)  //if joystick is tilted backwards
        {
            linear = 0.2;
        }

        if( sensor[First]<MIN_SIDES )
        {
            linear=0;
        }

        if( sensor[Third]<MIN_SIDES)
        {
            linear=0;
        }
    }

}

void JoyControl::publishTwist(float linearX, float joylinear ,float angularZ, float joyAngular)
{
     twist.linear.x = linearX * joylinear;
     twist.angular.z= angularZ * joyAngular;
     Twist_pub.publish(twist) ;

     ROS_INFO("Published state is: LX: %f, joy: %f , az: %f, joyaz: %f", linearX, joylinear, angularZ, joyAngular );
}





int main(int argc, char** argv)
{
    ros::init(argc,argv,"Joy_Op4");
	JoyControl jc;
    ros::Rate hertz(20);
	ROS_INFO("Joystick control ready:");
    jc.publishTwist(0,0,0,0);

	
    while(ros::ok())
    {
        jc.update();
        jc.publishTwist(jc.linear,jc.joy_lx,jc.angular,jc.joy_az);

        ros::spinOnce();
        hertz.sleep();
    }

    jc.publishTwist(0,0,0,0);


}





