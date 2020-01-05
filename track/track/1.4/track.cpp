#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <string>

#define _USE_MATH_DEFINES 
using namespace std;
double boat_size=0.6;
double kp=10;
//************************仅仅包含激光雷达避障部分，不包含tracking及移动目标
class track //使用class以便同时实现订阅与发布功能
{
public:
    track()
    {
        pub = nh.advertise <geometry_msgs::Vector3>("vtg",1000);                             //发布速度方向（控制
		sub_cloud = nh.subscribe("filtered_points",1000,&track::onmsg_cloud,this);           //接收激光雷达数据
		printf("success connected");
    }
	/*******************vo避障**********************/
	void onmsg_cloud(const std_msgs::Float64MultiArray &msg)  //接收激光雷达信息并计算vo域
	{
		printf("****************\n");                    
		vector<double> push_degree;
		vector< vector<double> > vo_degree;
		int size= end(msg.data) - begin(msg.data);
		printf("%d\n",size);
		double delta_control;
		/********************vo计算***********************/
		for (size_t i = 0; i < size/8; i++)  //4个点为一个障碍物
		{
			double temp_degree; 
			double min_degree=M_PI;//最大值       
			double max_degree=-M_PI;//最小值
			double min_num_dis,max_num_dis;//考虑船体积 θ=boat_size/num_dis
			for(int j=0;j<4;j++)//找出一个障碍物的vo边界
			{
				
				double x=msg.data[(i+j)*2];
				double y=msg.data[(i+j)*2+1];
				
				temp_degree=atan(y/x);//计算与激光雷达x轴（艏向）方向夹角
				printf("%lf %lf\n",x,y);
				if(y<=0)                                                 //分象限讨论，将夹角限制在(-π，π]内
				{
					if(x>0) temp_degree=-temp_degree;
					else if(x<0) temp_degree=M_PI-temp_degree;
					else if(x==0) temp_degree=M_PI/2;
				}
				else if(y>0)
				{
					if(x>0) temp_degree=-temp_degree;
					else if(x==0) temp_degree=-M_PI/2;
					else temp_degree=-M_PI-temp_degree;
				}
				
				if(temp_degree>max_degree)	{
					max_degree=temp_degree;  //更新最大值
					max_num_dis=sqrt(x*x+y*y);
				}
				if(temp_degree<min_degree)	{
					min_degree=temp_degree;  //更新最小值
					min_num_dis=sqrt(x*x+y*y);
				}
			}
			if(max_degree-min_degree>M_PI)//障碍物的vo范围总是小于π，大于则说明经过-x轴
			{
				push_degree.push_back(max_degree-boat_size/max_num_dis);//考虑体积
				push_degree.push_back(min_degree+boat_size/min_num_dis);
			}
			else 
			{
				// if(min_degree-M_PI/36>-M_PI) min_degree=min_degree-M_PI/36; //向周围扩展10°
				// else min_degree=-M_PI;
				// if(max_degree+M_PI/36<M_PI) max_degree=max_degree+M_PI/36;
				// else max_degree=M_PI;
				push_degree.push_back(min_degree-boat_size/min_num_dis);
				push_degree.push_back(max_degree+boat_size/max_num_dis);
			}
			vo_degree.push_back(push_degree);//储存边界
			push_degree.clear();
		}
		/********************vo计算***********************/
		geometry_msgs::Vector3 vtg;
		//combine_vo();//将相邻的障碍物合并
		delta_control=judge_control(delta_control,vo_degree);
		if(delta_control<M_PI/4&&delta_control>=0)  //幂函数控制器
			delta_control=sqrt(delta_control*M_PI)/2;
		if(delta_control>-M_PI/4&&delta_control<0)
			delta_control=-sqrt(-delta_control*M_PI)/2;
		vtg.x=0.15; //速度默认接近1m/s
		if(delta_control>=0)  //控制量大于零，即艏向在顺时针方向大于目标方向，将控制量变小，逆时针运动
	    {
			//ni  -1
			//分段控制
			if(delta_control<=M_PI/2)             //90度以内比例控制方向，90度为满舵
			{
				vtg.y=-(delta_control)/M_PI*2; 
			}
			else                          //大于90度直接满舵
			{
				vtg.y=-1;              
			}
		    		
	    }
	    else if(delta_control<0)
	    {
			//顺时针转向
			//shun  +1
			if(delta_control >= -M_PI/2)
			{
				vtg.y = -delta_control /M_PI*2;
			}	
			else
			{
				vtg.y=1;
			}
		}
		printf("delta-control=%lf\n",delta_control);
		printf("v=%f,r=%f\n",vtg.x,vtg.y);
		pub.publish(vtg); //发布速度值与方向*/
	}
	double judge_control(double delta_control,vector<vector<double> > vo_degree)      //判断控制量是否在vo内，在则修正
	{
		double heading_degree=-delta_control;
		for(int i=0;i<vo_degree.size();i++)
		{
			printf("min=%lf max=%lf\n",vo_degree[i][0],vo_degree[i][1]);
			if(vo_degree[i][0]<vo_degree[i][1])  //不过界情况
			{
				if(heading_degree>vo_degree[i][0]&&heading_degree<vo_degree[i][1])
				{
					if(heading_degree-vo_degree[i][0]<vo_sdegree[i][1]-heading_degree) heading_degree=vo_degree[i][0];
					else heading_degree=vo_degree[i][1];
				}
				break;
			}
			else  //过界
			{
				if(heading_degree>vo_degree[i][0]||heading_degree<vo_degree[i][1])
				{
					double temp_delta1,temp_delta2;
					temp_delta1=heading_degree-vo_degree[i][0];
					temp_delta2=vo_degree[i][1]-heading_degree;
					if(temp_delta1<-M_PI)	temp_delta1=temp_delta1+M_PI*2;
					if(temp_delta2<-M_PI)	temp_delta2=temp_delta2+M_PI*2;
					if(temp_delta1<temp_delta2)	heading_degree=temp_delta1;
					else heading_degree=temp_delta2;
				}
			}
		}printf("success connected");
		return -heading_degree;
	}
	/*void combine_vo()
	{

	}*/
	/*******************vo避障**********************/
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
	ros::Subscriber sub_cloud;
	
	
};

int main(int argc,char ** argv){
    ros::init(argc,argv,"track");
    track track1; //start exec
    ros::spin(); // spin() will not return until the node has been shutdown
    return 0;
}










