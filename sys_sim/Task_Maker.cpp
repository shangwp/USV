#include <bits/stdc++.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sys_sim//targetPoint.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim/boatObstacle.h>
#include <sys_sim/usv.h>
#include <sys_sim/rectangleObstacle.h>
using namespace std;
class Pathplan{
public:
	Pathplan(int select){
		position.push_back(0);
		position.push_back(0);
		position.push_back(0);
		had_got=false;
		pub_order=0;
		got_the_obstacle=false;
		had_plan=false;
		_select=select;
		sub_usv = nh.subscribe("usv",1000,&Pathplan::set_the_boat,this);
        pub_targetPoint = nh.advertise <sys_sim::targetPoint>("targetPoint",1000);
        feed_back = nh.subscribe("target_feedback",1000,&Pathplan::get_feedback,this);
     
	}
	bool had_got_msg(){
		return had_got;
	}
	bool sucess_to_destination(){
		if (sqrt(pow(position[0]-470,2)+pow(position[1]+100,2))<2) return true;
		else return false;
	}
	void set_the_boat(const sys_sim::usv& msg){
		position[0]=msg.x;
		position[1]=msg.y;
		position[2]=msg.hdt;
		cout<<"船位置信息："<<position[0]<<" "<<position[1]<<endl;
	}
	void get_feedback(const sys_sim::targetPoint& msg){
		if (msg.x.size()==0) {cout<<"反馈信息为空"<<endl;return ;}//反馈信息为空就直接结束
		if (msg.z[0]==-1) {//标志位为-1，则设置had_got
							cout<<"确认控制层获得点信息"<<endl;
							had_got=true;
						}
	}
	void plan_the_path(){
		the_plan_path.clear();
		if (_select==1){
			double tempx[]={10,10,30,30,50,50,70,70,90,90,110,110,130,130,150,150,170,170,190,190,210,210,230,230,250,250,270,270,290,290,310,310,330,330,350,350,370,370,390,390,410,410,430,430,450,450,470,470};
			double tempy[]={100,110,110,100,-100,-110,-110,-100,100,110,110,100,-100,-110,-110,-100,100,110,110,100,-100,-110,-110,-100,100,110,110,100,-100,-110,-110,-100,100,110,110,100,-100,-110,-110,-100,100,110,110,100,-100,-110,-110,-100};
			for (int i=0;i<47;i++){
				vector<double> temp;
				temp.push_back(tempx[i]);
				temp.push_back(tempy[i]);
				the_plan_path.push_back(temp);
			}
		}
		else if (_select==2){
			double tempx[19]={13,13,13,13,13,13,13,13,13,15,15.5,18,25,27.5,30,32,27.5,15,0};
			double tempy[19]={15,13,11,9,7,5,3,1,-1,0,0,0,12,1,-10,-8,1,0,-20};
			for (int i=0;i<19;i++){
				vector<double> temp;
				temp.push_back(tempx[i]);
				temp.push_back(tempy[i]);
				the_plan_path.push_back(temp);
			}
		}
		else if (_select==3){
			double tempx[]={0};
			double tempy[]={0};
			for (int i=0;i<sizeof(tempx);i++){
				vector<double> temp;
				temp.push_back(tempx[i]);
				temp.push_back(tempy[i]);
				the_plan_path.push_back(temp);
			}
		}
		cout<<"the_plan_path一共有"<<the_plan_path.size()<<"个点\n";
	}
	void pub_target_shanghai(){
		plan_the_path();
		target_point.x.clear();
		target_point.y.clear();
		target_point.z.clear();
		for (int i=0;i<the_plan_path.size();i++){
			target_point.x.push_back(the_plan_path[i][0]);
			target_point.y.push_back(the_plan_path[i][1]);
			target_point.z.push_back(0);
		}
		cout<<"一共发布点"<<target_point.x.size()<<endl;
		for (int i=0;i<target_point.x.size();i++){
			cout<<target_point.x[i]<<" "<<target_point.y[i]<<" "<<target_point.z[i]<<endl;
		}
		pub_targetPoint.publish(target_point);
		cout<<"点已经发布！";
	}
private:
	ros::NodeHandle nh;//节点
    ros::Publisher pub_targetPoint;//这是发布目标点信息的
    ros::Subscriber sub_globalObstacle;//订阅了障碍球的信息
    ros::Subscriber sub_usv;//订阅船的信息
    ros::Subscriber feed_back;//订阅控制层发布的信息
    sys_sim::targetPoint target_point;//发布目标点的信息：x,y和软硬信息
	vector<vector<double>> the_plan_path;
	vector<double> position;
	bool had_got;
	bool got_the_obstacle;
	bool had_plan;
	int pub_order;
	int _select;
};

int main(int argc,char ** argv){
	ros::init(argc,argv,"Pathplan");
	int select;
	cout<<"选择项目序号：1、2or3"<<endl;
	cin>>select;
	Pathplan plan(select);

	ros::Rate loop_rate(1);//f为发布频率（相对于sys_sim.cpp提高了一倍的频率） 
	while(ros::ok() && !plan.sucess_to_destination())//到终点就不发布信息了
    {
        if (!plan.had_got_msg())plan.pub_target_shanghai();//发布目标
        ros::spinOnce();
        loop_rate.sleep();//睡眠相应时间
    } 
}