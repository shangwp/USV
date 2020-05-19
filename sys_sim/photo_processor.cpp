#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <std_msgs/ByteMultiArray.h>
#include <sys_sim/photo_trans.h>

using namespace std;
class photoProcessor
{
public:
    photoProcessor()
    {
        pub_photo_trans = nh.advertise<sys_sim::photo_trans>("photo_trans",1000);
        sub_photo = nh.subscribe("input_photo",1000,&photoProcessor::onmsg_photo,this);
        printf("success connected!\n");
    }
    void onmsg_photo(const std_msgs::ByteMultiArray& msg)
    {
        sys_sim::photo_trans  photo_trans;
        for(int i=0;i<msg.data.size();i++)
        {
            photo_trans.color.push_back(msg.data[i]);
        }
        pub_photo_trans.publish(photo_trans);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_photo;
    ros::Publisher pub_photo_trans;
};
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"photo_processor");
    photoProcessor photo;
    ros::spin();
}