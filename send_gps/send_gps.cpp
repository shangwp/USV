#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/GPSFix.h>
#include <stdlib.h>
#include <iomanip>

using namespace std;

class send_gps
{
public:
    send_gps(int a)
    {
        pub = nh.advertise <gps_common::GPSFix>("send_gps",1000);
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&send_gps::onmsg_gps,this);
        printf("success connected");
    }
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)
    {
        gps_common::GPSFix msg1;
        msg1.latitude=msg.latitude;
        msg1.longitude=msg.longitude;
        msg1.pitch=msg.position_covariance[3];
        msg1.dip=msg.position_covariance[1];
        msg1.roll=msg.position_covariance[2];
        msg1.track=msg.position_covariance[1];
        msg1.header.frame_id='gps';
        msg1.err_speed=1;
        pub.publish(msg1);
        printf("%lf\n",msg1.roll);
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_gps;
};
int main(int argc,char ** argv){
    ros::init(argc,argv,"send_gps");
    int a=1;
    send_gps send(a);
    ros::spin();
    return 0;
}