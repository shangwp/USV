#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim//taskPoint.h>
#include <stdlib.h>
#include <vector>
using namespace std;
//任务层:
//    通过订阅全局障碍物信息和无人船信息，并根据任务来确定航线

//发布
/*
    目标点序列，从无人船当前位置，到可以确定的最远的目标点（例如：终点）的一系列点
    topic = "taskPoint"
    type  = "sys_sim::taskPoint"
             float64[] x; //目标点x坐标
             float64[] y; //目标点y坐标
             float64[] z; //目标朝向
    使用方式 float64[] 实际为vector<double>类型
    1.发布数据
*/

//订阅
/*
    全局障碍物信息
        topic='globalObstacle'
        type='sys_sim::globalObstacle
            float64[] x    
            float64[] y      位置（x，y)
            float64[] radius 球半径
            char[] color'    颜色
        使用方式(在对应回调函数内使用msg)
        1.读取，使用数组形式
        msg.x[i]
        msg.y[i]
        msg.z[i]
        2.获取数组长度
        msg.x.size();
        msg.y.size();
        msg.z.size();
    无人船状态信息
        topic='usv' 
        type='geometry_msgs::Vector3.h'
                x，
                y，位置（x，y）
                z，艏向
        使用方式(在对应回调函数内使用msg)
        1.读取
            msg.x
            msg.y
            msg.z
*/

class taskLayer
{
public:
    taskLayer()
    {
        sub_usv = nh.subscribe("usv",1000,&taskLayer::onmsg_usv,this);
        sub_globalObstacle = nh.subscribe("globalObstacle",1000,&taskLayer::onmsg_global,this);
        pub_taskPoint = nh.advertise <sys_sim::taskPoint>("taskPoint",1000);
    }
    //同时接受两个消息需要进行消息同步，这里实现软同步，在一个回调函数内更新私有成员，在另一个回调函数内使用。
    void onmsg_usv(const geometry_msgs::Vector3& msg)//无人船信息回调
    {
        usv.x = msg.x;
        usv.y = msg.y;
        usv.z = msg.z;
        cout<<"usv: "<<usv.x<<" "<<usv.y<<" "<<usv.z<<endl;
    }
    void onmsg_global(const sys_sim::globalObstacle& msg)//全局障碍物信息回调
    {
        cout<<"success"<<endl;
        sys_sim::taskPoint taskPoint;
        for(int i=0;i<msg.x.size();i++)
        {
            cout<<"globalObstacle: "<<msg.x[i]<<" "<<msg.y[i]<<" "<<msg.color[i]<<" "<<msg.radius[i]<<endl;
        }
        //设置目标点
        for(int i=0;i<5;i++)
        {
            taskPoint.x.push_back(i*50);
            taskPoint.y.push_back(0);
            taskPoint.z.push_back(0);
        }
        cout<<taskPoint.x[1]<<endl;
        pub_taskPoint.publish(taskPoint); //发布目标点
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub_taskPoint;
    ros::Subscriber sub_usv;
    ros::Subscriber sub_globalObstacle;
    geometry_msgs::Vector3 usv;
};

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"taskLayer");
    taskLayer task;
    ros::spin();
}