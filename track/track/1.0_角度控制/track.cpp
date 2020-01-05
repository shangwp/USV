#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <fstream>
#define _USE_MATH_DEFINES 
using namespace std;
ofstream log_control("log_control.csv",ios::out);
ofstream log_gps("gps_log.csv",ios::out);
class track //使用class以便同时实现订阅与发布功能
{
public:
    track(double x,double y)
    {
		lon_m=x*20037508.34/180;                 
		lat_m=log(tan((90+y)*M_PI/360))/(M_PI/180);
		lat_m=lat_m*20037508.34/180;                    //计算目标点墨卡托坐标
        pub = nh.advertise <geometry_msgs::Vector3>("vtg",1000);            //发布topic为 vtg  来更新速度
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&track::onmsg_gps,this);//信息源,均从gps中读取
		//sub_hdt = nh.subscribe("Heading",1000,&track::onmsg_hdt,this);
		
    }
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)//接收到gps信号时
    {	geometry_msgs::Vector3 vtg;	
		printf("num:%ld %f %f \n",num,a[num][0],a[num][1]);
		if(msg.longitude<0.1) //如果gps经纬度为0，0，无动作
		{
			vtg.x=0;
			vtg.y=0;
		}
		else{
			//printf("%f",msg.longitude);
			//printf("%f\n",msg.longitude);
		
		 //待发送的速度与方向，速度存在x，方向存在y
//船位置墨卡托转化
		double x,y;
		x = msg.longitude*20037508.34/180;
		y = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
		y = y*20037508.34/180;              //对接收到的经纬度进行处理，得到墨卡托坐标（单位m）
		log_gps<<setprecision(12)<<msg.longitude<<","<<setprecision(12)<<msg.latitude<<","
				<<setprecision(12)<<x<<","<<setprecision(12)<<y<<","
				<<setprecision(12)<<msg.position_covariance[5]<<","<<setprecision(12)<<msg.position_covariance[6]<<","
				<<setprecision(12)<<msg.position_covariance[7]<<endl;
//目标点距离计算
		double distance;
		distance=sqrt((lon_m-x)*(lon_m-x)+(lat_m-y)*(lat_m-y));//计算据目的地距离
		if(distance<0.2)//距离足够小则停止
		{
	    	vtg.x=0;
	    	vtg.y=0;
		log_control<<setprecision(12)<<msg.position_covariance[1]<<","<<setprecision(12)<<"0"<<","<<setprecision(12)<<"0"<<","<<setprecision(12)<<"0"<<",";	
		log_control<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
		printf("v=%f r=%f\n",vtg.x,vtg.y);
		pub.publish(vtg); //发布速度值与方向
		}
		else
		{
//控制量delta计算区 strat
			printf("distance=%f a1= %f\n",distance,msg.position_covariance[1]);//打印船头初始朝向, 正北方向顺时针[0,360）
	    	double a1; //艏向
			double ap;//目标朝向
	    	
			a1=msg.position_covariance[1]*M_PI/180;   //将艏向转化为弧度并顺时针增加正北为0点范围(-π,π]
			if(a1>M_PI)
			{
				a1=a1-2*M_PI;
			}
			
			ap=atan((lon_m-x)/(lat_m-y));  //计算船与目标点连线与正北方向夹角顺时针增加正北为0点范围（-π,π] 
	    	if(lat_m<=y&&lon_m>x) ap=M_PI/2-ap;
			if(lat_m<=y&&lon_m<=x) ap=-M_PI/2-ap;
			double delta=a1-ap;                             //控制量,目标为零 a1艏向,ap目标朝向
			printf("%f %f %f\n",a1,ap,delta);
			log_control<<setprecision(12)<<msg.position_covariance[1]<<","<<setprecision(12)<<a1<<","<<setprecision(12)<<ap<<","<<setprecision(12)<<delta<<",";
//控制量delta计算区  end up

//速度控制区 start
	//控制量足够小时（角度差足够小，不需要转向
	    	if(fabs(delta)<0.103)                          //差距大约5度时停止转向
	    	{
				vtg.x=0.15;									//速度值
				vtg.y=0;
									//方向  -1左转为逆时针转动，+1右转顺时针转动
	    	}
	//角度差较大，需要转向时
	    	else
	    	{
			//逆时针转向
	        	if((delta>=0&&delta<M_PI)||delta<=-M_PI)  //控制量大于零，即艏向在顺时针方向大于目标方向，将控制量变小，逆时针运动
	        	{
	 				//ni  -1
		    		vtg.x=0.15;
					if(delta<0)
					{
						delta=delta+M_PI*2;       //delta变到0到π内，以便分段转换为控制量
					}
					//分段控制
					if(delta<=M_PI/2)             //90度以内比例控制方向，90度为满舵
					{
						vtg.y=-(delta)/M_PI*2; 
					}
					else                          //大于90度直接满舵
					{
						vtg.y=-1;              
					}
		    		
	    		}
	    		else if((delta<0&&delta>-M_PI)||delta>=M_PI)//逆时针转动
	    		{
			//顺时针转向
					//shun  +1
		    		vtg.x=0.15;
					if(delta>0)
					{
						delta=delta-M_PI*2;
					}
					//分段控制
					if(delta>=-M_PI/2)
					{
						vtg.y=-delta/M_PI*2;
					}
					else
					{
						vtg.y=1;
					}
	    		}
	   		}
//速度控制区 end up
		}
		//printf("%f%f",vtg.x,vtg.y);
		log_control<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
		printf("v=%f r=%f\n",vtg.x,vtg.y);
		pub.publish(vtg); //发布速度值与方向
		}
    }
	void onmsg_hdt(const std_msgs::Float64& msg2);
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_gps;
	
    double lon_m; //目标点坐标
    double lat_m;
    //double a0;//hdt
	
};

int main(int argc,char ** argv){
	log_control<<"a0,"<<"a1,"<<"ap,"<<"delta"<<"v,"<<"r,"<<endl;
	log_gps<<"longitude,"<<"latitude,"<<"Mo_x,"<<"Mo_y,"<<"ve,"<<"vn,"<<"vu"<<endl;
    	ros::init(argc,argv,"track");
	double lon0=113.3809862;
	double lat0=23.0671321;
    	track track1(lon0,lat0); //start exec
	//ros::MultiThreadedSpinner s(2);  //多线程
  	ros::spin();  
	log_control.close();
    	return 0;
}
/*void track::onmsg_hdt(const std_msgs::Float64& msg2)
{
	//printf("%f",msg2.data);
	a0=msg2.data;
	//ros::Rate loop_rate(10);//block chatterCallback2() 0.5Hz
  	//loop_rate.sleep();
 }*/

