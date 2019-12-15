#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <fstream>
#include <vector>

#define _USE_MATH_DEFINES 
using namespace std;
double time_step=0.1;//时间间隔设置为1s
double axis_length=0.5;//转轴长度,暂定为螺旋桨到船中部距离
struct boat_state
{
    double x,y;
    double x_m,y_m;
    double heading;
    double v;
};
class sim_boat
{
public:
    sim_boat()
    {
        pub = nh.advertise<sensor_msgs::NavSatFix>("unionstrong/gpfpd",1000);
        pub_cloud = nh.advertise<std_msgs::Float64MultiArray>("filtered_points",1000);
        sub = nh.subscribe("vtg",1000,&sim_boat::onmsg_vtg,this);
        state.heading=0;
        state.v=1;
        state.x=113.3831616;
        state.y=23.06900558;

	state.x_m= state.x*20037508.34/180;
	state.y_m = log(tan((90+state.y)*M_PI/360))/(M_PI/180);
	state.y_m = state.y_m*20037508.34/180;              //对接收到的经纬度进行处理，得到墨卡托坐标（单位m）
        
        send_gps_msg.position_covariance[1]=state.heading/M_PI*180;
        send_gps_msg.longitude=state.x;
        send_gps_msg.latitude=state.y;

        //double cloud_set[16]={2,5,2,-1,4,5,4,-1,2,5,2,-1,4,5,4,-1};
        //for(int i=0;i<16;i++)
        //cloud_msg.data.push_back(cloud_set[i]);
    }
    void onmsg_vtg(const geometry_msgs::Vector3 &msg)//收到控制命令，无人船运动，更新状态
    {
        update_state(msg.x,msg.y);
    }
    void pub_state()
    {
        pub.publish(send_gps_msg);
        //pub_cloud.publish(cloud_msg);
    }
    void update_state(double v_control,double r_control)
    {
        //将控制量转换为物理量
        state.v=5*v_control; //速度,单位m/s
        double orientation=r_control*60*M_PI/180;
        double delta_heading=state.v*sin(orientation)/axis_length*time_step;
        //update
        state.x_m=state.x_m+state.v*cos(orientation)*sin(state.heading)*time_step;//墨卡托坐标更新
        state.y_m=state.y_m+state.v*cos(orientation)*cos(state.heading)*time_step;
        state.x = state.x_m / 20037508.34 * 180;            //经纬坐标更新
        state.y = state.y_m / 20037508.34 * 180;
        state.y = 180 / M_PI * (2 *atan(exp(state.y * M_PI / 180)) - M_PI / 2);
        state.heading=state.heading+delta_heading;  //艏向更新

        //gps发布信息更新
        send_gps_msg.position_covariance[1]=state.heading/M_PI*180;
        if(send_gps_msg.position_covariance[1]<0)
            send_gps_msg.position_covariance[1]=send_gps_msg.position_covariance[1]+2*M_PI;
        send_gps_msg.longitude=state.x;
        send_gps_msg.latitude=state.y;
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Publisher pub_cloud;
    ros::Subscriber sub;
    sensor_msgs::NavSatFix send_gps_msg;
    std_msgs::Float64MultiArray cloud_msg;
    boat_state state;

};

int main(int argc , char ** argv )
{
    ros::init(argc,argv,"sim_boat");
    sim_boat sim;

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        sim.pub_state();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
