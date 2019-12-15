#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2ConstPtr>
#include <iostream>
#include <stdlib.h>
#include <iomanip>
#include <math.h>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#define _USE_MATH_DEFINES 
using namespace std;

ofstream track_set("track_set.csv",ios::out);
ofstream track_log("track_log.csv",ios::out);

vector<vector<double> >a;  //目标点
int num_curr=0; //当前目标点
int look_ahead_point;

double k=0.2; 		//横向偏转比例
double vt=1.0;		//速度1.0m/s,控制命令0.2
double l=0.3;     //轴长
//----module读取csv文件转化为二维数组a,并记录a内值(目标点值)
inline void file_to_string(vector<string> &record, const string& line, char delimiter);
inline double string_to_float(string str);
void read_file();
void read();

/********************************** pure pursuit循迹 **************************************/ 
//寻找最近点作为目标点,规定新点只能沿路径朝前  ??可能会跳过点
int nearest_find(double x,double y)
{
	double temp_dis;
	double min_dis=100000;
	int    res_num;
	int    length_point_num=a.size();
	for(int i=num_curr;i<length_point_num-1;i++)
	{
		temp_dis=sqrt((a[i][0]-x)*(a[i][0]-x)+(a[i][1]-y)*(a[i][1]-y));
		if(temp_dis<=min_dis)
		{
			res_num=i;
			min_dis=temp_dis;
		}
	}
	return res_num;
}

double pp_delta_find_degree(double x,double y,double heading)
{
	double degree_usv;//无人船->目标点角度
	double delta_u2p_degree;//α
	double delta_control;
	// // // // // // delta1 = heading - degree[num_curr];
	// // // // // // if(delta1>M_PI) delta1=delta1-2*M_PI;
	// // // // // // if(delta1<-M_PI) delta1=delta1+2*M_PI; //delta1<0顺时针转 +
	// // // // // // printf("delta1=%lf\n",delta1);
	double ld=sqrt((a[look_ahead_point][0]-x)*(a[look_ahead_point][0]-x)+(a[look_ahead_point][1]-y)*(a[look_ahead_point][1]-y));

	//计算无人船->目标点角度
	degree_usv=atan((a[look_ahead_point][0]-x)/(a[look_ahead_point][1]-y)); 
	if(a[look_ahead_point][1]<=y&&a[look_ahead_point][0]>x) degree_usv=M_PI+degree_usv;
	if(a[look_ahead_point][1]<=y&&a[look_ahead_point][0]<=x) degree_usv=-M_PI+degree_usv;
	//α
	delta_u2p_degree=heading-degree_usv;
	if(delta_u2p_degree>M_PI) delta_u2p_degree=delta_u2p_degree-2*M_PI;
	if(delta_u2p_degree<-M_PI) delta_u2p_degree=delta_u2p_degree+2*M_PI;

	if(delta_u2p_degree<=-M_PI/2||delta_u2p_degree>=M_PI/2)
	{
		if(delta_u2p_degree>0)
		    delta_control=M_PI/2;
		if(delta_u2p_degree<0)
			delta_control=-M_PI/2;
	}
	else
	{
		delta_control=atan(sin(delta_u2p_degree)*2*l/ld);
	}

	track_log<<setprecision(12)<<delta_u2p_degree<<","<<setprecision(12)<<ld<<","<<setprecision(12)<<delta_control<<",";
	printf("α=%lf,ld=%lf,delta=%lf,\n",delta_u2p_degree,ld,delta_control);
	return delta_control;
}
/**************************** pure pursuit循迹 **************************************/
class track //使用class以便同时实现订阅与发布功能
{
public:
    track(int init_track)
    {
        pub = nh.advertise <geometry_msgs::Vector3>("vtg",1000);         
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&track::onmsg_gps,this);
		sub_cloud = nh.subscribe("filtered_points",1000,&track::onmsg_cloud,this);
		printf("success connected");
    }
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)//接收到gps信号时
    {
		printf("num:%d lon:%f lat:%f\n",num_curr,a[num_curr][0],a[num_curr][1]);
		geometry_msgs::Vector3 vtg; //待发送的速度与方向，速度存在x，方向存在y
		if(msg.longitude<0.1) //如果gps经纬度为0，0，无动作
		{
                vtg.x=0;
	    	    vtg.y=0;
                pub.publish(vtg);
                printf("gps no signal\n");
		}
		else
		{
			//printf("%f",msg.longitude);
			//printf("%f\n",msg.longitude);
		
		
        //track_real<<setprecision(12)<<num<<",";
//船位置墨卡托转化
			double x,y;
			x = msg.longitude*20037508.34/180;
			y = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
			y = y*20037508.34/180;              //对接收到的经纬度进行处理，得到墨卡托坐标（单位m）

			//计算船头朝向
			double heading;
			heading=msg.position_covariance[1]/180*M_PI;
			if(heading>M_PI)
				heading=heading-2*M_PI;


			num_curr=nearest_find(x,y);
			if(num_curr<a.size()-10)
				look_ahead_point=num_curr+10;
			else 
				look_ahead_point=a.size()-1;
			
			printf("ux=%lf,uy=%lf,heading=%lf\n px=%lf,py=%lf\n",x,y,heading,a[num_curr][0],a[num_curr][1]);
			track_log<<setprecision(12)<<x<<","<<setprecision(12)<<y<<","<<setprecision(12)<<heading<<","<<setprecision(12)<<a[num_curr][0]<<","
            <<setprecision(12)<<a[num_curr][1]<<","<<setprecision(12)<<a[look_ahead_point][0]<<","<<setprecision(12)<<a[look_ahead_point][1]<<",";



			//计算与目标角度差值
			double delta_control;
			delta_control=pp_delta_find_degree(x,y,heading);
			delta_control=judge_control(delta_control);
			
			vtg.x=0.2; //速度默认接近1m/s
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
			track_log<<setprecision(12)<<vtg.x<<","<<setprecision(12)<<vtg.y<<endl;
			printf("v=%f,r=%f\n",vtg.x,vtg.y);
			pub.publish(vtg); //发布速度值与方向*/
			
		}
    }
	/*******************vo避障**********************/
	void onmsg_cloud(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)  //接收激光雷达信息并计算vo域
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
		vectot<double> push_degree;
		for (size_t i = 0; i < current_pc_ptr->points.size(); i+=4)  //计算vo
		{
			double temp_degree;
			double min_degree=M_PI;
			double max_degree=-M_PI;
			
			for(int j=0;j<4;j++)//找出vo边界
			{
				temp_degree=atan(current_pc_ptr->points[i+j].y/current_pc_ptr->point[i+j].x);
				if(y<0)
				{
					if(x>=0) temp_degree=-temp_degree;
					if(x<0) temp_degree=M_PI-temp_degree;
				}
				else if(y>0)
				{
					if(x>=0) temp_degree=-temp_degree;
					else temp_degree=-M_PI-temp_degree;
				}
				else if(y==0) temp_degree=-y/fabs(y)*temp_degree;
				if(temp_degree>max_degree)	max_degree=temp_degree;
				if(temp_degree<min_degree)	min_degree=temp_degree;
			}
			if(max_degree-min_degree>M_PI)
			{
				push_degree.push_back(max_degree-M_PI/18);
				push_degree.push_back(min_degree+M_Pi/18);
			}
			else
			{
				if(min_degree-M_PI/18>-M_PI) min_degree=min_degree-M_PI/18;
				else min_degree=-M_PI;
				if(max_degree+M_PI/18<M_PI) max_degree=max_degree+M_PI/18;
				else max_degree=M_PI;
				push_degree.push_back(min_degree);
				push_degree.push_back(max_degree);
			}
			vo_degree.push_back(push_degree);//储存边界
			push_degree.clear();
		}
	}
	double judge_control(double delta_control)      //判断控制量是否在vo内，在则修正
	{
		double heading_degree=-delta_control;
		for(int i=0;i<vo_degree.size();i++)
		{
			if(vo_degree[i][0]<vo_degree[i][1])  //不过界情况
			{
				if(heading_degree>vo_degree[i][0]&&heading_degree<vo_degree[i][1])
				{
					if(heading_degree-vo_degree[i][0]<vo_degree[i][1]-heading_degree) heading_degree=vo_degree[i][0];
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
		}
		return -heading_degree;
	}
	/*******************vo避障**********************/
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_gps;
	ros::Subscriber sub_cloud;
	vector<vector<double>> vo_degree;
};

int main(int argc,char ** argv){
	track_log<<"usv.x,"<<"usv.y,"<<"usv.heading,"<<"nearest.x,"<<"nearest.y,"<<"look_ahead.x,"<<"look_ahead.y,"<<"α,"<<"ld,"<<"delta_control,"<<"v,"<<"r"<<endl;
	track_set<<"x,"<<"y"<<endl;
	read();
    printf("total numbers of point: %ld\n",a.size());
	double ap;

	printf("preprocessor finish\n");

    ros::init(argc,argv,"track");
    track track1(); //start exec
	ros::MultiThreadedSpinner spinner(2); // Use 2 threads  多线程同时接受多个回调函数
    spinner.spin(); // spin() will not return until the node has been shutdown



	track_set.close();
	track_log.close();
    return 0;
}


// *********************** //读取csv文件（循迹使用
void read_file()
{
	ifstream inFile("/home/shang/catkin_ws/src/track:",ios::in);
}
void read()
{

    vector<string> row;
    string line;
    string filename;
	vector<double>b;
	int mod=20;       //取点间隔,从原始gps采集数据取点
	int flag_read=0;  //
    ifstream in("/home/shang/catkin_ws/src/track/track.csv");  
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

        flag_read++;
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
// **********************************   //











