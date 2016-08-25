#include <ros/ros.h>
#include <fyp/Int16Array.h>
#include <array>
#include <assert.h>

class SigFilter
{
    public:

        SigFilter();

        std::array< fyp::Int16Array, 10 > buffer;
        int head, tail, size;
        const int MAX;
        fyp::Int16Array msg;

        void increment(int& index);
        void insertData(const fyp::Int16Array& data);
        void removeData();
        fyp::Int16Array filter();

        ros::NodeHandle nh;
        ros::Subscriber listener;
        ros::Publisher  Array_pub;
        void collectArray(const fyp::Int16Array::ConstPtr& array);

};

SigFilter::SigFilter():head(0), tail(0),size(0), MAX(10){

    listener=nh.subscribe<fyp::Int16Array>("ultrasonicArray",10,&SigFilter::collectArray,this);
    Array_pub=nh.advertise<fyp::Int16Array>("filteredArray",1);

}

void SigFilter::collectArray(const fyp::Int16Array::ConstPtr& array)
{    
    insertData(*array);
}

void SigFilter::increment(int& index)
{
    index++;
    if( index >= MAX) index= index% MAX;
}

void SigFilter::insertData(const fyp::Int16Array& data)
{
    if( size!=0 && tail == head )
    {
        increment(head);

    }

    buffer[tail]=data;
    increment(tail);
    size++;


}

void SigFilter::removeData()
{
    buffer[ tail ].data= {0.0,0.0,0.0};
    increment(head);
    size--;
}

fyp::Int16Array SigFilter::filter()
{
    fyp::Int16Array toReturn;

    int dividend = buffer.size();

    if(buffer.size()>=MAX)
    {
      int16_t temp[]={ 0,0,0 };

    for(int count=0; count<buffer.size(); ++count)
    {
        temp[0] += buffer[0].data[0];
        temp[1] += buffer[1].data[1];
        temp[2] += buffer[2].data[2];

        ROS_INFO("during additon: (+ %d) %d", buffer[1].data[1],temp[1]);
    }

    ROS_INFO("before division: %d", temp[1]);

    temp[0] = (int16_t) temp[0]/dividend;
    temp[1] = (int16_t) temp[1]/dividend;
    temp[2] = (int16_t) temp[2]/dividend;

    ROS_INFO("after division: %d", temp[1]);

    toReturn.data[0]=temp[0];
    toReturn.data[1]=temp[1];
    toReturn.data[2]=temp[2];
  }
  else if(buffer.size()<MAX )
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

  while(ros::ok())
  {
      sf.msg=sf.filter();
      sf.Array_pub.publish(sf.msg);
      ros::spinOnce();
  }

}
