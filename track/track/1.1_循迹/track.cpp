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
ofstream track_set("track_real.csv",ios::out);
ofstream track_real("track_real.csv",ios::out);
vector<vector<double> >a;  //二维数组存储读入变量
vector<double>b;
inline void file_to_string(vector<string> &record, const string& line, char delimiter);
inline double string_to_float(string str);
int num=0;
void read()
{
    vector<string> row;
    string line;
    string filename;
    ifstream in("trackt.csv");  
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
class track //使用class以便同时实现订阅与发布功能
{
public:
    track(double x,double y)
    {
		/*lon_m=x*20037508.34/180;                 
		lat_m=log(tan((90+y)*M_PI/360))/(M_PI/180);
		lat_m=lat_m*20037508.34/180;          */          //计算目标点墨卡托坐标
        pub = nh.advertise <geometry_msgs::Vector3>("vtg",1000);            //发布topic为 vtg  来更新速度
        sub_gps = nh.subscribe("unionstrong/gpfpd",1000,&track::onmsg_gps,this);//信息源,均从gps中读取
		//sub_hdt = nh.subscribe("Heading",1000,&track::onmsg_hdt,this);
		
    }
    void onmsg_gps(const sensor_msgs::NavSatFix& msg)//接收到gps信号时
    {
		printf("num:%ld lon:%f lat:%f \n",num,a[num][0],a[num][1]);
		if(msg.longitude<0.1) //如果gps经纬度为0，0，无动作
		{
			vtg.x=0;
	    	vtg.y=0;
			pub.publish(vtg); 
		}
		else{
			//printf("%f",msg.longitude);
			//printf("%f\n",msg.longitude);
		
		geometry_msgs::Vector3 vtg; //待发送的速度与方向，速度存在x，方向存在y
//船位置墨卡托转化
		double x,y;
		x = msg.longitude*20037508.34/180;
		y = log(tan((90+msg.latitude)*M_PI/360))/(M_PI/180);
		y = y*20037508.34/180;              //对接收到的经纬度进行处理，得到墨卡托坐标（单位m）
		track_real<<setprecision(12)<<x<<","<<setprecision(12)<<y<<setprecision(12)<<msg.longitude<<","<<setprecision(12)<<msg.latitude<<endl;
//目标点距离计算
		double distance;
		distance=sqrt((a[num][0]-x)*(a[num][0]-x)+(a[num][1]-y)*(a[num][1]-y));
		while(distance<0.2)//距离足够小则停止
		{
			distance=sqrt((a[num][0]-x)*(a[num][0]-x)+(a[num][1]-y)*(a[num][1]-y));//计算据目的地距离,直到找到新点
	    	vtg.x=0;
	    	vtg.y=0;
			if(num<=a.size())
			{
				num++;
				print("%d",num);				
			}
		}
		else
		{
//控制量delta计算区 strat
			printf("a1= %f\n",msg.position_covariance[1]);//打印船头初始朝向, 正北方向顺时针[0,360）
	    	double a1; //艏向
			double ap;//目标朝向
	    	
			a1=msg.position_covariance[1]*M_PI/180;   //将艏向转化为弧度并顺时针增加正北为0点范围(-π,π]
			if(a1>M_PI)
			{
				a1=a1-2*M_PI;
			}
			
			ap=atan((lon_m-x)/(lat_m-y));  //计算船与目标点连线与正北方向夹角顺时针增加正北为0点范围（-π,π] 
	    	if(lat_m<y) ap=-ap;
			
			double delta=a1-ap;                             //控制量,目标为零 a1艏向,ap目标朝向
			printf("%f %f %f\n",a1,ap,delta);
			log_control<<setprecision(12)<<msg.position_covariance[1]<<","<<setprecision(12)<<a1<<","<<setprecision(12)<<ap<<","<<setprecision(12)<<delta<<",";
//控制量delta计算区  end up

//速度控制区 start
	//控制量足够小时（角度差足够小，不需要转向
	    	if(fabs(delta)<0.0873)                          //差距大约5度时停止转向
	    	{
				vtg.x=0.15;									//速度值
				vtg.y=0;									//方向  -1左转为逆时针转动，+1右转顺时针转动
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
						delta=-delta-M_PI*2;
					}
					//分段控制
					if(delta>=-M_PI/2)
					{
						vtg.y=delta/M_PI*2;
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
void read_file()
{
	ifstream inFile("G:",ios::in);
}
int main(int argc,char ** argv){
	log_control<<"a0,"<<"a1,"<<"ap,"<<"delta,"<<"v,"<<"r,"<<endl;
	read();
    printf("total numbers of point: %d\n",a.size());
    cout<<"top 5dims of person 1:"<<endl;
    for(int i=0;i<=4;i++)
        cout<<a[0][i]<<" ";
    ros::init(argc,argv,"track");
	double lon0=113.3820844;
	double lat0=23.0678931;
    track track1(lon0,lat0); //start exec
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

