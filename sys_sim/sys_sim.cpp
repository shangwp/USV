#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim//taskPoint.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
using namespace std;
//环境和感知层：
//   构建环境地图，保存环境参数和无人船状态，形成感知数据

//订阅
/*接受无人船控制信息，用来更新 
    控制量
        topic='vtg'  
        typer='geometry_msgs::Vector3'
                x, 速度控制量
                y, 转向控制量
                z, 未使用
*/

//发布
/*发布局部障碍物信息，全局障碍物信息，无人船状态信息
    局部障碍物信息
        范围：只关注距离无人船50范围内的障碍物
        坐标系:以无人船位置为原点，无人船朝向为y轴正向，垂直右侧x轴正向建立平面直角坐标系；
        topic='localObstacle'
        type='sys_sim::localObstacle
            float64[] x //  相对于x船的距离，以无人船右侧为x轴正方向
            float64[] y //  以无人船船头方向为y轴正方向
            char[] color'   颜色
    全局障碍物信息
        范围：环境内所有障碍物
        坐标系： 全局坐标系
        topic='globalObstacle'
        type='sys_sim::globalObstacle
            float64[] x    
            float64[] y      位置（x，y)
            float64[] radius 球半径
            char[] color'    颜色
    无人船状态信息
        topic='usv' 
        type='geometry_msgs::Vector3.h'
                x，全局x坐标
                y，全局y坐标
                z，艏向
*/

double time_step = 0.1;// 10hz,0.1s
struct boat//无人船状态量
{
    double vel; //无人船速度（指速率，单位m/s）
    double hdt; //无人船艏向，正北为0°，顺时针方向为正[0~360)
    double x,y; //全局坐标
    
    boat()
    {
        vel = 0;
        hdt = M_PI/2;
        x = 0;
        y = 0;
    }
};
struct ball//障碍球信息
{
    double x,y;//球位置
    double radius;//球半径
    unsigned char color;//颜色，0黑色，1红色，2蓝色，3绿色，4黄色
};
struct map_sim//环境地图
{
    ball obstacle[7];//环境构成为多个障碍球
    map_sim()
    {
    //1.扬帆起航地图
    //起点上下两个绿球
    obstacle[0].x=0;
    obstacle[0].y=5;
    obstacle[0].radius=0.75;
    obstacle[0].color=3;//绿球

    obstacle[1].x=0;
    obstacle[1].y=-5;
    obstacle[1].radius=0.75;
    obstacle[1].color=3;//绿球

    obstacle[2].x=50;
    obstacle[2].y=0;
    obstacle[2].radius=0.75;
    obstacle[2].color=1;//红球，左侧通过（y+方向）

    obstacle[3].x=100;
    obstacle[3].y=0;
    obstacle[3].radius=0.75;
    obstacle[3].color=0;//黑球，顺时针绕行一周

    obstacle[4].x=150;
    obstacle[4].y=0;
    obstacle[4].radius=0.75;
    obstacle[4].color=2;//蓝球，右侧通行（y-方向）

    //终点两个绿球
    obstacle[5].x=200;
    obstacle[5].y=5;
    obstacle[5].radius=0.75;
    obstacle[5].color=3;//绿球

    obstacle[6].x=200;
    obstacle[6].y=-5;
    obstacle[6].radius=0.75;
    obstacle[6].color=3;//绿球
    }
};

class sys
{
public:
    sys()
    {
        pub_globalObstacle = nh.advertise <sys_sim::globalObstacle>("globalObstacle",1000);
        pub_localObstacle = nh.advertise <sys_sim::localObstacle>("localObstacle",1000);
        pub_usv = nh.advertise <geometry_msgs::Vector3>("usv",1000);
        sub_vtg = nh.subscribe("vtg",1000,&sys::onmsg_vtg,this);
        printf("success connected!");
    }
    void onmsg_vtg(const geometry_msgs::Vector3& vtg)
    {
        
        //根据控制量更新无人船状态
        boat1.vel = vtg.x*5;
        boat1.hdt = boat1.hdt + vtg.y*M_PI/2;
        if(boat1.hdt <0) boat1.hdt + M_PI*2;
        else if(boat1.hdt>M_PI*2) boat1.hdt = boat1.hdt - (M_PI*2);

        double trans_degree;
        trans_degree = boat1.hdt;
        if(boat1.hdt>M_PI) trans_degree = boat1.hdt - M_PI*2;
        trans_degree = -trans_degree;
        trans_degree = trans_degree + M_PI/2;
        if(trans_degree > M_PI) trans_degree = trans_degree - M_PI*2; 

        boat1.x = boat1.x + boat1.vel*time_step*cos(trans_degree);
        boat1.y = boat1.y + boat1.vel*time_step*sin(trans_degree);
    }
    void sensor_layer()//形成感知数据
    {
        //局部障碍物 localObstacle
        //范围：只关注距离无人船50范围内的障碍物
        //坐标系:以无人船位置为原点，无人船朝向为y轴正向，垂直右侧x轴正向建立平面直角坐标系；
        int num_ob=0;//局部障碍物数量
        sys_sim::localObstacle local_ob;
        for(int i=0;i<7;i++)
        {
            int x_ob = map1.obstacle[i].x;
            int y_ob = map1.obstacle[i].y;
            double radius = map1.obstacle[i].radius;
            if(((boat1.x-x_ob)*(boat1.x-x_ob)+(boat1.y-y_ob)*(boat1.y-y_ob))<2500) //只计算50m范围内的局部障碍物
            {
                //计算无人船距障碍物距离，和无人船朝向与障碍物方向夹角
                //夹角-180°到180°，顺时针为正，以船头朝向为0
                double distance = sqrt((boat1.x-x_ob)*(boat1.x-x_ob)+(boat1.y-y_ob)*(boat1.y-y_ob));
                double re_angle = -atan2((boat1.x-x_ob),(boat1.y-y_ob));
                
                //障碍物颜色
                local_ob.color.push_back(map1.obstacle[i].color);
                //障碍物四边形包围圈的顶点
                for(int j=0;j<4;j++)
                {
                    local_ob.x.push_back(distance*cos(re_angle)+radius*cos(j*M_PI/2+re_angle));
                    local_ob.y.push_back(distance*sin(re_angle)+radius*sin(j*M_PI/2+re_angle));
                }               
                num_ob++;
            }
        }

        //全局障碍物 globalObstacle
        //范围：环境内所有障碍物
        //坐标系： 全局坐标系
        sys_sim::globalObstacle global_ob;
        for(int i=0;i<7;i++)
        {
            global_ob.x.push_back(map1.obstacle[i].x);
            global_ob.y.push_back(map1.obstacle[i].y);
            global_ob.radius.push_back(map1.obstacle[i].radius);
            global_ob.color.push_back(map1.obstacle[i].color);
        }
        //发布障碍物信息
        pub_localObstacle.publish(local_ob);
        pub_globalObstacle.publish(global_ob);
    }
    void pub_state()
    {
        //发布无人船状态，保存在3维向量中，x，y分量表示位置，z分量表示艏向
        geometry_msgs::Vector3 usv;
        usv.x = boat1.x;
        usv.y = boat1.y;
        usv.z = boat1.hdt;
        pub_usv.publish(usv);

        sensor_layer();//形成感知层数据并发布
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub_localObstacle;
    ros::Publisher pub_globalObstacle;
    ros::Publisher pub_usv;
    ros::Subscriber sub_vtg;
    map_sim map1;
    boat boat1;
};
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"sys_sim");
    sys sim;
    //ros::Time::init();//an error https://blog.csdn.net/github_39611196/article/details/83046478
    //在声明对象后放置loop_rate可以防止以上错误
    int f = static_cast<int>(1/time_step);//发布消息频率与time_step保持一致
    ros::Rate loop_rate(10);//f为发布频率 
    
    while(ros::ok())
    {
        sim.pub_state();//发布环境信息，感知层信息
        ros::spinOnce();
        loop_rate.sleep();//睡眠相应时间
    } 
}


