#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#define _USE_MATH_DEFINES 
using namespace std;
ofstream log_control("log_control.csv",ios::out);
ofstream track_set("track_set.csv",ios::out);
ofstream track_real("track_real.csv",ios::out);
vector<vector<double> >a;  //二维数组存储读入变量
vector<double>b;
inline void file_to_string(vector<string> &record, const string& line, char delimiter);
inline double string_to_float(string str);
int num=0;
double k=2;
double vt=1.0;
double degree[2];
void read()
{
    vector<string> row;
    string line;
    string filename;
    ifstream in("/home/deepdriving/catkin_ws/src/track/track.csv");  
    if (in.fail())  { cout << "File not found" <<endl; return ; } 
    while(getline(in, line)  && in.good() )
    {
        file_to_string(row, line, ',');  //把line里的单元格数字字符提取出来，“,”为单元格分隔符
        for(int i=1; i>=0; i--){
        	double x;
        	double y=string_to_float(row[i]);
        	if(i==1)
        	{
        		x=y*20037508.34/180; 
			}
			else if(i==0)
			{
				                
				x=log(tan((90+y)*M_PI/360))/(M_PI/180);
				x=x*20037508.34/180;  
			}
            b.push_back(x);
        }
		track_set<<setprecision(12)<<b[0]<<","<<setprecision(12)<<b[1]<<endl;
        a.push_back(b);
        b.clear();
    }
    in.close();
    return ;
}

inline void file_to_string(vector<string> &record, const string& line, char delimiter)
{
    int linepos=0;
    char c;
    int linemax=line.length();
    string curstring;
    record.clear();
    while(linepos<linemax)
    {
        c = line[linepos];
        if(isdigit(c)||c=='.'){
            curstring+=c;
        }
        else if(c==delimiter&&curstring.size()){
            record.push_back(curstring);
            curstring="";
        }
        ++linepos;
    }
    if(curstring.size())
        record.push_back(curstring);
    return;
}

inline double string_to_float(string str){
    int i=0,len=str.length();
    double sum=0;
    while(i<len){
        if(str[i]=='.') break;
        sum=sum*10+str[i]-'0';
        ++i;
    }
    ++i;
    double t=1,d=1;
    while(i<len){
        d*=0.1;
        t=str[i]-'0';
        sum+=t*d;
        ++i;
    }
    return sum;
}
/*
double stanley_dis(double x,double y,int num,double degree)
{
	if(num==0)
	{

	}
}
int find_nearest(double x,double y)
{
	double degree
}
*/
class track //使用class以便同时实现订阅与发布功能
{
public:
    track(int &a)
    {
		/*lon_m=x*20037508.34/180;                 
		lat_m=log(tan((90+y)*M_PI/360))/(M_PI/180);
		lat_m=lat_m*20037508.34/180;          */          //计算目标点墨卡托坐标
        pub = nh.advertise <geometry_msgs::Vector3>("vtg",1000);            //发布topic为 vtg  来更新速度
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&track::onmsg_gps,this);
		printf("success connected");//信息源,均从gps中读取
		//sub_hdt = nh.subscribe("Heading",1000,&track::onmsg_hdt,this);
		
    }
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)//接收到gps信号时
    {
		printf("num:%d lon:%f lat:%f\n",num,a[num][0],a[num][1]);
		geometry_msgs::Vector3 vtg; //待发送的速度与方向，速度存在x，方向存在y
		if(msg.longitude<0.1) //如果gps经纬度为0，0，无动作
		{
                vtg.x=0;
	    	    vtg.y=0;
                pub.publish(vtg);
                track_real<<setprecision(12)<<0<<","<<setprecision(12)<<0<<setprecision(12)<<msg.longitude<<","<<setprecision(12)<<msg.latitude
            <<setprecision(12)<<msg.position_covariance[5]<<","<<setprecision(12)<<msg.position_covariance[6]<<endl;
		}
		else{
			//printf("%f",msg.longitude);
			//printf("%f\n",msg.longitude);
		
		
        //track_real<<setprecision(12)<<num<<",";
//船位置墨卡托转化
		double x,y;
		x = msg.longitude*20037508.34/180;
		y = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
		y = y*20037508.34/180;              //对接收到的经纬度进行处理，得到墨卡托坐标（单位m）
		track_real<<setprecision(12)<<x<<","<<setprecision(12)<<y<<setprecision(12)<<msg.longitude<<","<<setprecision(12)<<msg.latitude
            <<setprecision(12)<<msg.position_covariance[5]<<","<<setprecision(12)<<msg.position_covariance[6]<<endl;
		double distance1;
		double distance2;
		double degree_dis;
		distance1=sqrt((a[0][0]-x)*(a[0][0]-x)+(a[0][1]-y)*(a[0][1]-y));
		distance2=sqrt((a[1][0]-x)*(a[1][0]-x)+(a[1][1]-y)*(a[1][1]-y));
		if(distance2<=0.5)
		{
			vtg.x=-0.2;
			vtg.y=0;
			if(distance2<=0.2)
			{
				vtg.x=-0.2;
				vtg.y=0;
			}
			log_control<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
			pub.publish(vtg); //发布速度值与方向*/
		}
		else
		{
		double del_degree_dis;
		double dis_stanley;
		if(distance1<=distance2)
		{
			degree_dis=atan((a[1][0]-x)/(a[1][1]-y)); 
			if(a[1][1]<=y&&a[1][0]>x) degree_dis=M_PI/2-degree_dis;
			if(a[1][1]<=y&&a[1][0]<=x) degree_dis=-M_PI/2-degree_dis;
			del_degree_dis=degree[0]-degree_dis;
			dis_stanley=fabs(distance2*sin(del_degree_dis));
		}
		else
		{
			degree_dis=atan((a[0][0]-x)/(a[0][1]-y)); 
			if(a[0][1]<=y&&a[0][0]>x) degree_dis=M_PI/2-degree_dis;
			if(a[0][1]<=y&&a[0][0]<=x) degree_dis=-M_PI/2-degree_dis;
			del_degree_dis=degree[0]-degree_dis;
			dis_stanley=fabs(distance2*sin(del_degree_dis));
		}
		double degree_stanley_control;
		double a1;
		a1=msg.position_covariance[1]*M_PI/180;   //将艏向转化为弧度并顺时针增加正北为0点范围(-π,π]
		if(a1>M_PI)
		{
			a1=a1-2*M_PI;
		}

		double et;
		double k_l=(a[0][1]-a[1][1])/(a[0][0]-a[1][0]);
		if(y>=k_l*(x-a[0][0])+a[0][1]&&k_l>=0)
		{
			et=-atan(k*dis_stanley/vt);
		}
		else if(y>=k_l*(x-a[0][0])+a[0][1]&&k_l<0)
		{
			et=atan(k*dis_stanley/vt);
		}
		else if(y<k_l*(x-a[0][0])+a[0][1]&&k_l>=0)
		{
			et=atan(k*dis_stanley/vt);
		}
		else if(y<k_l*(x-a[0][0])+a[0][1]&&k_l<0)
		{
			et=-atan(k*dis_stanley/vt);
		}


		degree_stanley_control=a1-degree[0]+et;
		vtg.x=0.2;
		if((degree_stanley_control>=0&&degree_stanley_control<M_PI)||degree_stanley_control<=-M_PI)  //控制量大于零，即艏向在顺时针方向大于目标方向，将控制量变小，逆时针运动
	    {
	 		//ni  -1
		    
			if(degree_stanley_control<0)
			{
				degree_stanley_control=degree_stanley_control+M_PI*2;       //delta变到0到π内，以便分段转换为控制量
			}
			//分段控制
			if(degree_stanley_control<=M_PI/2)             //90度以内比例控制方向，90度为满舵
			{
				vtg.y=-(degree_stanley_control)/M_PI*2; 
			}
			else                          //大于90度直接满舵
			{
				vtg.y=-1;              
			}
		    		
	    }
	    else if((degree_stanley_control<0&&degree_stanley_control>-M_PI)||degree_stanley_control>=M_PI)
	    {
			//顺时针转向
			//shun  +1
			if(degree_stanley_control>0)
			{
				degree_stanley_control = degree_stanley_control - M_PI*2;
			}
			//分段控制
			if(degree_stanley_control >= -M_PI/2)
			{
				vtg.y = -degree_stanley_control / M_PI*2;
			}
			else
			{
				vtg.y=1;
			}
	    }
		log_control<<setprecision(12)<<msg.position_covariance[1]<<","<<setprecision(12)<<a1<<","<<setprecision(12)<<degree[0]<<","<<setprecision(12)<<degree_stanley_control<<",";
		log_control<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
		pub.publish(vtg); //发布速度值与方向*/
		}
		
//目标点距离计算
/*		double distance;
		distance=sqrt((a[num][0]-x)*(a[num][0]-x)+(a[num][1]-y)*(a[num][1]-y));
        
		while(distance<5)//距离足够小则停止
		{
			distance=sqrt((a[num][0]-x)*(a[num][0]-x)+(a[num][1]-y)*(a[num][1]-y));//计算据目的地距离,直到找到新点

			if(num<a.size()-1)
			{
                vtg.x=-0.1;
	    	    vtg.y=0;
                pub.publish(vtg);
				num++;
				printf("%d",num);				
			}
            else if(num==a.size()-1)
            {
                break;
            }
		}
		if(distance<0.5)
        {
                vtg.x=-0.1;
	    	    vtg.y=0;
                pub.publish(vtg);
        }
//控制量delta计算区 strat
			printf("a1= %f\n",msg.position_covariance[1]);//打印船头初始朝向, 正北方向顺时针[0,360）
	    	double a1; //艏向
			double ap;//目标朝向
	    	
			a1=msg.position_covariance[1]*M_PI/180;   //将艏向转化为弧度并顺时针增加正北为0点范围(-π,π]
			if(a1>M_PI)
			{
				a1=a1-2*M_PI;
			}
			
			ap=atan((a[num][0]-x)/(a[num][1]-y));  //计算船与目标点连线与正北方向夹角顺时针增加正北为0点范围（-π,π] 
	    	if(a[num][1]<=y&&a[num][0]>x) ap=M_PI/2-ap;
			if(a[num][1]<=y&&a[num][0]<=x) ap=-M_PI/2-ap;
			double delta=a1-ap;                             //控制量,目标为零 a1艏向,ap目标朝向
			printf("%f %f %f %f\n",a1,ap,delta,distance);
			log_control<<setprecision(12)<<msg.position_covariance[1]<<","<<setprecision(12)<<a1<<","<<setprecision(12)<<ap<<","<<setprecision(12)<<delta<<",";
//控制量delta计算区  end up

//速度控制区 start

	//角度差较大，需要转向时

			//逆时针转向
	        	if((delta>=0&&delta<M_PI)||delta<=-M_PI)  //控制量大于零，即艏向在顺时针方向大于目标方向，将控制量变小，逆时针运动
	        	{
	 				//ni  -1
		    		vtg.x=0.2;
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
	    		else if((delta<0&&delta>-M_PI)||delta>=M_PI)
	    		{
			//顺时针转向
					//shun  +1
		    		vtg.x=0.2;
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
	   		
//速度控制区 end up
		
		//printf("%f%f",vtg.x,vtg.y);
		log_control<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
		pub.publish(vtg); //发布速度值与方向*/
		}
    }
	void onmsg_hdt(const std_msgs::Float64& msg2);
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_gps;
	
    //double lon_m; //目标点坐标
    //double lat_m;
    //double a0;//hdt
	
};
void read_file()
{
	ifstream inFile("/home/deepdriving/catkin_ws/src/track:",ios::in);
}
int main(int argc,char ** argv){
	log_control<<"a0,"<<"a1,"<<"ap,"<<"delta,"<<"v,"<<"r,"<<endl;
	read();
    printf("total numbers of point: %ld\n",a.size());
    cout<<"top 5dims of person 1:"<<endl;
    for(int i=0;i<=1;i++)
        cout<<a[i][0]<<" ";
    ros::init(argc,argv,"track");
	double ap;
	ap=atan((a[1][0] - a[0][0])/(a[1][1] - a[0][1]));  //计算船与目标点连线与正北方向夹角顺时针增加正北为0点范围（-π,π] 
	if(a[0][1]<=a[1][1]&&a[0][0]>a[1][0]) ap=M_PI/2-ap;
	if(a[0][1]<=a[1][1]&&a[0][0]<=a[1][0]) ap=-M_PI/2-ap;
	degree[0]=ap;
	degree[1]=ap;
	int b=0;
    track track1(b); //start exec
	//ros::MultiThreadedSpinner s(2);  //多线程
  	ros::spin();  
	log_control.close();
	track_real.close();
	track_set.close();
    return 0;
}
/*void track::onmsg_hdt(const std_msgs::Float64& msg2)
{
	//printf("%f",msg2.data);
	a0=msg2.data;
	//ros::Rate loop_rate(10);//block chatterCallback2() 0.5Hz
  	//loop_rate.sleep();
 }*/

