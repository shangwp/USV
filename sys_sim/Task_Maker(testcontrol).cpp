#include <bits/stdc++.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <algorithm>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim//targetPoint.h>
#include <sys_sim/usv.h>
using namespace std;
double T0=18;
double T1=1e-9;
double r=0.99;
int len=1;
class Pathplan
{
public:
	Pathplan(){//构造函数
		position.push_back(0);
		position.push_back(0);
		nums=0;
		all=0;
		has_plan=false;
		success=false;
		sub_usv = nh.subscribe("usv",1000,&Pathplan::set_the_boat,this);
		sub_globalObstacle = nh.subscribe("globalObstacle",1000,&Pathplan::add_ball,this);
        pub_targetPoint = nh.advertise <sys_sim::targetPoint>("targetPoint",1000);
        feed_back = nh.subscribe("target_feedback",1000,&Pathplan::get_feedback,this);
	}
	void get_feedback(const sys_sim::targetPoint& msg){//这个函数用来接受控制层的反馈
			if (feedback_point.size()==0)//feedbakc_point用来存储接受的反馈的点的信息，如果最开始是空，则直接接受
			{
				feedback_point.push_back(msg.x[0]);
				feedback_point.push_back(msg.y[0]);
				feedback_point.push_back(msg.z[0]);
				if (path.size()!=0) {vector<int>::iterator tempx=path.begin();path.erase(tempx);}
			}
			else {
				if(msg.x[0]!=feedback_point[0] || msg.y[0]!=feedback_point[1] ){
							feedback_point[0]=msg.x[0];
							feedback_point[1]=msg.y[0];
							feedback_point[2]=msg.z[0];
							if (path.size()!=0) {vector<int>::iterator tempx=path.begin();path.erase(tempx);}
						}
					}
			//feedbakc_point不为空且与新发送的msg信息不同，则说明控制层到了目标点，并寻找下一个目标点，此时就进行一系列操作即可			
				for (int i=0;i<yellow_balls.size();i++){//救起来的球要从yellow_ball中删除，以免下次再次规划
					if (yellow_balls[i][0]==feedback_point[0] && yellow_balls[i][1]==feedback_point[1])
					{ 
					complished.push_back(feedback_point);
					yellow_balls[i][2]=1;
					nums++;//表示已经就救起来的数量
					break;
					}
				}
			
				
		
	}	
	void set_the_boat(const sys_sim::usv& msg)//获取船的位置信息
	{
		cout<<"set_the_boat\n";
		position[0]=msg.x;
		position[1]=msg.y;
		cout<<"set_the_boat\n";

	}

	void add_ball(const sys_sim::globalObstacle& msg)//获取球的信息并在出现新黄球的时候规划
	{
		cout<<"add_ball\n";
		if (msg.x.size()>all)
		{
			bool is_plan=false;
			for (int i=all;i<msg.x.size();i++)
			{
				all++;
				if (msg.color[i]==4)
				{
					vector<double> temp;
					temp.push_back(msg.x[i]);
					temp.push_back(msg.y[i]);
					temp.push_back(0);
					is_plan=true;
					yellow_balls.push_back(temp);
					cout<<"add_ball_yellow\n";
				}
			}
			cout<<"add_ball\n";
			has_plan=path_plan(is_plan);
		}
		
	}
	void get_the_matrix(vector<vector<double>>& matrix){//这个是模拟退火算法计算路径的一部分
		cout<<"get_the_matrix\n";
		vector<vector<double>> temp;
		temp.push_back(position);

		if (find_order.size()!=0) find_order.clear();
		for (int i=0;i<yellow_balls.size();i++){
			if (yellow_balls[i][2]==0) {
				find_order.push_back(i);
				temp.push_back(yellow_balls[i]);
			}

		}
		for (int i=0;i<temp.size();i++){
			vector<double> x(temp.size());
			matrix.push_back(x);
		}
		for (int i=0;i<temp.size();i++){
			for (int j=0;j<temp.size();j++){
				if (i!=j)matrix[i][j]=sqrt(pow(temp[i][0]-temp[j][0],2)+pow(temp[j][1]-temp[j][1],2));
				else matrix[i][j]=0;
			}
		}
		cout<<"get_the_matrix\n";
	}
	void change(vector<int>&v){//这个是模拟退火算法计算路径的一部分
		cout<<"change\n";
    	int x = rand()%v.size();
    	int y = rand()%v.size();
    	while(y == x) y = rand()%v.size();
    	for(int i=0; i+x<=(x+y>>1); ++i)
        	if (x+i!=0 && y-i!=0) swap(v[i+x], v[y-i]);
        cout<<"change\n";
	}
	double all_dis(vector<int> patha,const vector<vector<double>>& matrix){
		cout<<"all_dis\n";
		double all_distance=0;
		for (int i=0;i<patha.size()-1;i++){
			all_distance+=matrix[patha[i]][patha[i+1]];

		}
		cout<<"all_dis\n";
		return all_distance;
	}
	void tuihuo(const vector<vector<double>>& matrix){//模拟退火算法得到一条路径的序号
		cout<<"tuihuo\n";
		vector<int> paths;
		for (int i=0;i<matrix.size();i++){
			paths.push_back(i);
		}
		srand((unsigned int)time(NULL));

		while (T0>T1)
		{
			for (int i=0;i<len;i++)
			{
				vector<int>temp=paths;
				change(temp);
				double pre=all_dis(paths,matrix);
				double cur=all_dis(temp,matrix);
				if(cur<pre||exp(-(cur-pre)/T0)>((double)rand())/RAND_MAX){
					paths=temp;
				}
				cout<<"tuihuo\n";
			}
			T0*=r;
		}
		vector<int>::iterator tempx=paths.begin();
		paths.erase(tempx);
		path=paths;
	cout<<"tuihuo\n";
	
	}
	bool path_plan(bool is_plan)//规划函数
	{
		cout<<"path_plan\n";
		if (is_plan==false) return false;
		else 
		{

			vector<vector<double>> matrix;
			get_the_matrix(matrix);
			tuihuo(matrix);
			cout<<"path_plan\n";
			return true;
		}
	}
	
	
	void pub_target_task2() //发布函数
	{

		if (nums==15) {success=true;return ;}//如果够15个点就直接判断任务完成，结束即可
		if (has_plan!=true) {
			cout<<"pub_target_task2_none\n";
			target_point.x.push_back(50);
			target_point.y.push_back(50);
			target_point.z.push_back(0);
			pub_targetPoint.publish(target_point);
			cout<<target_point.x[0]<<" "<<target_point.y[0]<<" "<<target_point.z[0]<<endl;
			return ;
		}
		if (path.size()!=0)//当还有规划好的没有到达的目标点，就直接按要求发布
			{
				cout<<"pub_target_task2_some\n";
			target_point.x.clear();
			target_point.y.clear();
			target_point.z.clear();
			if (feedback_point.size()!=0)
			{	target_point.x.push_back(feedback_point[0]);
				target_point.y.push_back(feedback_point[1]);
				target_point.z.push_back(feedback_point[2]);
					}
			for (int i=0;i<path.size();i++){
							target_point.x.push_back(yellow_balls[find_order[path[i]-1]][0]);
							target_point.y.push_back(yellow_balls[find_order[path[i]-1]][1]);
							target_point.z.push_back(2);
						}

					}
			
		
		else  {//当已经到了所有规划好的目标点，就直接继续发布最后一个硬目标点，并随即产生一个点来搜索区域

			if (feedback_point.size()!=0){
				target_point.x.push_back(feedback_point[0]);
				target_point.y.push_back(feedback_point[1]);
				target_point.z.push_back(feedback_point[2]);
			}
			vector<int> ret;
			get_the_max(ret);
			srand((unsigned int)time(NULL));
			target_point.x.push_back((double)(rand() % (ret[1]-ret[0] + 1) + ret[0]));
			target_point.y.push_back((double)(rand() % (ret[3]-ret[2] + 1) + ret[2]));
			target_point.z.push_back(0);
						cout<<"pub_target_task2_allright\n";
		}
		cout<<position[0]<<" "<<position[1]<<endl;
		for (int i=0;i<target_point.x.size();i++){
				cout<<target_point.x[i]<<" "<<target_point.y[i]<<" "<<target_point.z[i]<<endl;
			}
		pub_targetPoint.publish(target_point);
	}
	
	void get_the_max(vector<int>& ret){//这个是产生随即点函数的一部分
		
		if (complished.size()==0) {
			
			ret.push_back((int)position[0]),ret.push_back((int)yellow_balls[0][0]),ret.push_back((int)position[1]),ret.push_back((int)yellow_balls[0][1]);
		}
		else {
			int xmax=0,ymax=0;
			for (int i=0;i<complished.size();i++){
					
					if (xmax<(int)complished[i][0]) xmax=(int)complished[i][0];
					if (ymax<(int)complished[i][1]) ymax=(int)complished[i][1];
					
				}
				
			ret.push_back((int)position[0]),ret.push_back(xmax),ret.push_back((int)position[1]),ret.push_back(ymax);
			}

			cout<<"get_the_max\n";
	}
	
	~Pathplan()
	{
		yellow_balls.clear();
		white_balls.clear();
		position.clear();
		complished.clear();
		path.clear();
		nums=0;
	}
	
	bool sucess_to_destination(){
		return success;
	}

private:
	ros::NodeHandle nh;//节点
    ros::Publisher pub_targetPoint;//这是发布目标点信息的
    ros::Subscriber sub_globalObstacle;//订阅了障碍球的信息
    ros::Subscriber sub_usv;//订阅船的信息
    ros::Subscriber feed_back;//订阅控制层发布的信息
    sys_sim::targetPoint target_point;//发布目标点的信息：x,y和软硬信息
	vector<vector<double>> yellow_balls;//需要营救的人质:x,y
	vector<vector<double>> white_balls;//需要避开的障碍物
	vector<vector<double>> complished;
	vector<int> path;//辅助查找最短路径的向量数组
	vector<double> position;//船现在的位置
	vector<double> feedback_point;//存储控制层返回的点的信息
	vector<int> find_order;
	bool success;//判断最后是不是所有球体已经救援完成
	bool has_plan;
	int nums;//已经救援的球体
	int all;//所有已经获取信息的球体
};
bool flag = 0;
void get_feedback(const sys_sim::targetPoint& msg)
{
	cout<<msg.z[0]<<endl;
	cout<<"get feedback"<<endl;
	if(msg.z[0]==-1)
	{
		cout<<"t1"<<endl;
		flag =1;
	}
}

int main(int argc,char ** argv){
	ros::init(argc,argv,"Pathplan");
	//Pathplan plan;
    ros::Publisher pub_targetPoint1;
	ros::Subscriber feed_back;//订阅控制层发布的信息
	ros::NodeHandle nh1;
	sys_sim::targetPoint target_point1;//发布目标点的信息：x,y和软硬信息
	pub_targetPoint1 = nh1.advertise <sys_sim::targetPoint>("targetPoint",1000);
	feed_back = nh1.subscribe("target_feedback",1000,&get_feedback);
	ros::Rate loop_rate(10);//f为发布频率（相对于sys_sim.cpp提高了一倍的频率） 
	double x[]={20};
	double y[]={40};
	int    z[]={2};
	target_point1.x.push_back(x[0]);
	target_point1.y.push_back(y[0]);
	target_point1.z.push_back(z[0]);
	
	while(ros::ok() )//到终点就不发布信息了  && !plan.sucess_to_destination()
    {
		cout<<flag<<endl;
        //plan.pub_target_task2();//发布目标
		if(flag==0)
		{
			pub_targetPoint1.publish(target_point1);
		}
        ros::spinOnce();
        loop_rate.sleep();//睡眠相应时间
    } 
}
