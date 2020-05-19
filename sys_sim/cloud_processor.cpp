#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <sys_sim/radar_trans.h>

using namespace std;

class cloudProcessor
{
public:
    cloudProcessor()
    {
        pub_radar_trans = nh.advertise<sys_sim::radar_trans>("radar_trans",1000);
        sub_cloud = nh.subscribe("velodyne_points",1000,&cloudProcessor::onmsg_cloud,this);
        printf("success connected!\n");
    }
    void onmsg_cloud(const std_msgs::Float64MultiArray& msg)
    {
        sys_sim::radar_trans radar_trans;
        for(int i=0;i<msg.data.size()/6;i++)
        {
            radar_trans.localx.push_back(msg.data[6*i+0]); 
            radar_trans.localy.push_back(msg.data[6*i+1]); 
            radar_trans.globalx.push_back(msg.data[6*i+2]); 
            radar_trans.globaly.push_back(msg.data[6*i+3]); 
            radar_trans.radius.push_back(msg.data[6*i+4]); 
            radar_trans.color.push_back(msg.data[6*i+5]); 
        }
        pub_radar_trans.publish(radar_trans);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_radar_trans;
};
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"cloud_processor");
    cloudProcessor cloud;
    ros::spin();
}