#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <fyp/Int16Array.h>

/*
 * Implemented 2 sets of sensors, shfted some instruction to places that make sense.
 */

enum Joystick { LeftRight = 0, ToFro, Turn, Lever, SmallLeftRight, SmallToFro };
enum Sensors  { First=0, Second , Third, Fourth, Fifth, Sixth };
enum MoveState { Still=0, Forward,BackWard, ClockWise, AntiClockWise};

class JoyControl
{
	public:
		
		JoyControl();
	
        float linear,linearBack, angular;
        float joy_lx, joy_az;
        int16_t sensor[6];
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
                          MIN_DIST(40), MAX_DIST(90),MIN_SIDES(50)
{
	Twist_pub=nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel",1);
	Joy_listener=nh.subscribe<sensor_msgs::Joy>("joy",10, &JoyControl::JoyCallBack,this);
	US_listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10, &JoyControl::SonicCallBack,this);
}


void JoyControl::JoyCallBack(const sensor_msgs::Joy::ConstPtr& joy)   //this collects relevant joystick msg
{
    joy_lx=joy->axes[ToFro];
    joy_az=joy->axes[Turn];		
   // ROS_INFO("Received state is: joy: %f ,joyaz: %f", joy_lx,joy_az );

}

void JoyControl::SonicCallBack( const fyp::Int16Array::ConstPtr& ary) //this collects info from front sensors
{
    sensor[0]    = ary->data[First];
    sensor[1]    = ary->data[Second];
    sensor[2]    = ary->data[Third];
    sensor[3]    = ary->data[Fourth];
    sensor[4]    = ary->data[Fifth];
    sensor[5]    = ary->data[Sixth];

}

 void JoyControl::update()
{
      ROS_INFO("Updatng");

    if( joy_az > 0.1 || joy_az < -0.1 )    //if joystick is twisted
    {
        ROS_INFO("Turning");

        angular=1;
        linear=0;
        linearBack=0;

        publishTwist(0,joy_lx,angular,joy_az);

        return;

    }else
    {
        angular = 0;

        if( joy_lx >0.1 && joy_lx<= 1.0 )   //if joystick is tilted forwards
        {
            ROS_INFO("Forwards");

            int dist= sensor[Second]>MAX_DIST ? MAX_DIST:sensor[Second];
            linear = (float)(dist-MIN_DIST)/(float)(MAX_DIST - MIN_DIST);
            linear = linear>0 ? linear:0;

            ROS_INFO("Published state is: LX: %f", linear);


            if( sensor[First]<MIN_SIDES || sensor[Third]<MIN_SIDES)
            {
                linear=0;
                ROS_INFO("sides change lx to 0: %d, %d", sensor[First], sensor[Third]);

            }

            publishTwist(linear,joy_lx,angular,joy_az);


        }else if(joy_lx< -0.1 && joy_lx>= -1.0)
        {
            ROS_INFO("Backwards");

            int distBack= sensor[Fifth]>MAX_DIST ? MAX_DIST:sensor[Fifth];
            linearBack = (float)(distBack-MIN_DIST)/(float)(MAX_DIST - MIN_DIST);
            linearBack = linearBack>0 ? linearBack:0;

            if( sensor[Fourth]<MIN_SIDES || sensor[Sixth]<MIN_SIDES)
            {
                linearBack=0;
                ROS_INFO("sides change lx to 0");

            }

            publishTwist(linearBack,joy_lx,angular,joy_az);

        }
        else
        {
            publishTwist(0,joy_lx,0,joy_az);

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

        ros::spinOnce();
        hertz.sleep();
    }


}





