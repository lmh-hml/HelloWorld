#include <ros/ros.h>
#include <fyp/Int16Array.h>
#include <deque>

class SigFilter
{
    public:

        SigFilter();

        typedef std::deque<fyp::Int16Array> Collection;
        Collection buffer;
        fyp::Int16Array msg;
        fyp::Int16Array filter();
        void collectArray(const fyp::Int16Array::ConstPtr& array);
        ros::NodeHandle nh;
        ros::Subscriber listener;
        ros::Publisher  Array_pub;

};

SigFilter::SigFilter(){

    listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10,&SigFilter::collectArray,this);
    Array_pub=nh.advertise<fyp::Int16Array>("filteredArray",1);

}

void SigFilter::collectArray(const fyp::Int16Array::ConstPtr& array)
{
  int size=buffer.size();

    if(size>=10)
    {
        msg=filter();
        Array_pub.publish(msg);
        buffer.clear();
    }
    buffer.push_back(*array);

  //  ROS_INFO("Current buffr size: %d",size);


}


fyp::Int16Array SigFilter::filter()
{
  fyp::Int16Array toReturn;

  if(buffer.size()>=10)
  {
    int16_t temp[]={ 0,0,0 };

    for(int count=0; count<buffer.size(); ++count)
    {
        temp[0] += buffer[0].data[0];
        temp[1] += buffer[1].data[1];
        temp[2] += buffer[2].data[2];

       // ROS_INFO("during additon: (+ %d) %d", buffer[1].data[1],temp[1]);
    }

  //  ROS_INFO("before division: %d", temp[1]);

    temp[0] = (int16_t) temp[0]/10;
    temp[1] = (int16_t) temp[1]/10;
    temp[2] = (int16_t) temp[2]/10;

    ROS_INFO("after division: %d", temp[1]);

    toReturn.data[0]=temp[0];
    toReturn.data[1]=temp[1];
    toReturn.data[2]=temp[2];
  }
  else if(buffer.size() )
  {
    toReturn=buffer.back();
  }

    return toReturn;

}



int main(int argc, char** argv)
{
  ros::init(argc,argv,"SigFig");
  SigFilter sf;
  ROS_INFO("SigFig ready:");


  ros::spin();



}
