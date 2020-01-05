// #include <ros/ros.h>
// #include <geometry_msgs/Vector3.h>
// #include <sensor_msgs/PointCloud2ConstPtr>
// #include <std_msgs/MultiArrayDimension.h>
 #include <std_msgs/Float64MultiArray.h>
// #include <iostream>
// #include <stdlib.h>
// #include <math.h>
// #include <vector>
// #include <string>
// #include <pcl/point_types.h>
// #include <pcl/conversions.h>
// #include <pcl/ModelCoefficients.h>
// #define _USE_MATH_DEFINES 
// using namespace std;
//************************仅仅包含激光雷达避障部分，不包含tracking及移动目标
class cloud_pub //使用class以便同时实现订阅与发布功能
{
public:
    cloud_pub()
    {

        pub = nh.advertise <geometry_msgs::Vector3>("cloud_pub",1000);                             //发布速度方向（控制
		sub_cloud = nh.subscribe("filtered_points",1000,&cloud_pub::onmsg_cloud,this);           //接收激光雷达数据
		printf("success connected");
    }

	void onmsg_cloud(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)  //接收激光雷达信息并计算vo域
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);    
		pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);                     

		// int num=current_pc_ptr->points.size()*2;
		// std_msgs::MultiArrayLayout msg;
		std_msgs::Float64MultiArray m;
 		// const unsigned int data_sz = num;
		// m.layout.dim.push_back(std_msgs::MultiArrayDimension());
		// m.layout.dim[0].size = data_sz;
		// m.layout.dim[0].stride = 1;
		// m.layout.dim[0].label = "cloud";

		// std_msgs::Float64MultiArray msg;

		for (size_t i = 0; i < current_pc_ptr->points.size(); i++) 
		{
			 msg.data.push_back(current_pc_ptr->point[i].x);
			 msg.data.push_back(current_pc_ptr->point[i].y);

			//m.data[2*i]=current_pc_ptr->point[i].x;
			//m.data[2*i+1]=current_pc_ptr->point[i].y;
		}
		pub.publish(m); //发布速度值与方向*/
	}
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
	ros::Subscriber sub_cloud;

};

int main(int argc,char ** argv){
    ros::init(argc,argv,"cloud_pub");
    cloud_pub cloud(); //start exec
    ros::spin(); // spin() will not return until the node has been shutdown
    return 0;
}










