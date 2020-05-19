#include <ros/ros.h>
#include <math.h>
#include <stdlib.h>
#include <cstdlib> // Header file needed to use srand and rand
#include <ctime> // Header file needed to use time
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ByteMultiArray.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim/boatObstacle.h>
#include <sys_sim/usv.h>
#include <sys_sim/rectangleObstacle.h>

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
            int[] color'   颜色
    全局障碍物信息
        范围：环境内所有障碍物
        坐标系： 全局坐标系
        topic='globalObstacle'
        type='sys_sim::globalObstacle
            float64[] x    
            float64[] y      位置（x，y)
            float64[] radius 球半径
            int[] color'    颜色，0黑色，1红色，2蓝色，3绿色，4黄色

    障碍船信息
        topic='boatObstcale'
        type='sys_sim::boatObstacle'
              float64[] x
              float64[] y
              float64[] hdt
              float64[] radius
              float64[] vel
    无人船状态信息
        topic='usv' 
        type='sys_sim::usv'
            float64  x
            float64  y
            float64  hdt
            float64  vel
            float64  radius

*/
//算法思路
/*
    定义环境地图,规定每个障碍物的全局坐标,半径,颜色  ->map_sim
    接收到控制层指令,便更新无人船状态,即执行运动操作,未接收到控制层指令则不更新,相当于原地不动 ->onmsg_vtg
    以固定频率发布无人船信息和障碍物信息 -> pub_state
    局部障碍物以船头为y轴的局部坐标系,计算50m邻域内的障碍物在该坐标系的位置,以固定频率发布 -> sensor_layer
    全局障碍物开始为空,累计记录50m范围内的障碍物,发现新的障碍物或者旧障碍物变动比较大,便发布新的全局障碍物信息
*/
//整体任务架构
/*
    任务规划: F(u,e) 
    F任务规划,结果为一系列点{p1,p2,p3,...,pn}.  u无人船状态,e 全局障碍物
    当e不变,假设无人船很好的沿着预定轨迹运动,则F(u_{i+1},e) \in F(u_i,e). 不需要重新规划路线(但需要在控制层决定下一个目标点的更新)
    当e改变,便需要重新规划路径.
    所以可以以全局障碍物发布时机作为任务规划的时机


    硬目标点
    软目标点
*/
double time_step = 0.1;// 10hz,0.1s
int   pub_flag = 0;//pub_flag 0使用简洁发布,pub_flag 1使用感知层发布.
//环境定义
struct boat//无人船状态量
{
    double vel; //无人船速度（指速率，单位m/s）
    double hdt; //无人船艏向，正北为0°，顺时针方向为正[0~360)
    double x,y; //全局坐标
    double radius;
    boat()
    {
        vel = 0;
        hdt = 0;
        x = 0;
        y = 16;
        radius = 2;
    }
};
struct ball//障碍球信息
{
    double x,y;//球位置
    double radius;//球半径,统一定为0.75
    char color;//颜色，0黑色，1红色，2蓝色，3绿色，4黄色
};
struct buoy//游标类型
{
    double x;
    double y;
};
//环境地图
// 使用方法:
//   在class sys的private中修改mapx_sim map为
//   map1_sim map;或map2_sim map或...
struct map1_sim  //扬帆起航
{
    ball obstacle[7];
    map1_sim()
    {
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
struct map2_sim  //飓风营救
{
        ball obstacle[20];//注意球间距离至少15m
        map2_sim()
        {
            double temp[2][20]={
                {20,30,60,105,140,155,170, 22, 32, 55, 98,120,130,160,180,45,112, 57,87,150},
                {40,80,65, 30, 35, 85, 35,-32,-80,-30,-50,-90,-10,-60, -5, 8, 83,-62,-8,-45}
            };
            for(int i=0;i<20;i++)
            {
                obstacle[i].radius = 0.75;
                obstacle[i].x = temp[0][i];
                obstacle[i].y = temp[1][i];
                if(i<15)
                {
                    obstacle[i].color=4;
                }
                else obstacle[i].color=0;
            }
        }

};
struct map3_sim  //跨越险阻
{
    boat boat_ob[8];
    ball obstacle[40];
    map3_sim()
    {
        for(int i=0;i<8;i++)
        {
            if(30*i<40)
            {
                boat_ob[i].x = 40;
                boat_ob[i].y = 30*i;
                boat_ob[i].hdt = 0;
                boat_ob[i].vel = 1.5;
                boat_ob[i].radius = 1;
            }
            else if(30*i<80)
            {
                boat_ob[i].x = 40+(30*i-40);
                boat_ob[i].y = 40;
                boat_ob[i].hdt = M_PI_2;
                boat_ob[i].vel = 1.5;
                boat_ob[i].radius = 1;
            }
            else if(30*i<160)
            {
                boat_ob[i].x = 80;
                boat_ob[i].y = 40-(30*i-80);
                boat_ob[i].hdt = M_PI;
                boat_ob[i].vel = 1.5;
                boat_ob[i].radius = 1;               
            }
            else if(30*i<200)
            {
                boat_ob[i].x = 80-(30*i-160);
                boat_ob[i].y = -40;
                boat_ob[i].hdt = M_PI*3/2;
                boat_ob[i].vel = 1.5;
                boat_ob[i].radius = 1;               
            }
            else if(30*i<240)
            {
                boat_ob[i].x = 40;
                boat_ob[i].y = -40+(30*i-200);
                boat_ob[i].hdt = 0;
                boat_ob[i].vel = 1.5;
                boat_ob[i].radius = 1;       
            }
        }
    }

};
struct map4_sim  //载誉而归
{
    ball obstacle[51];
    map4_sim()
    {
        for(int i=0;i<51;i++)  //赋初值，从上到下赋值，第一行和最后一行颜色为黑，其余初始为红
        {
            if(i<7)
            {
                obstacle[i].x=20*(i+1);
                obstacle[i].y=60;
                obstacle[i].color=0;
                obstacle[i].radius=0.75;
            }
            else if(i<14)
            {
                obstacle[i].x=20*(i+1-7);
                obstacle[i].y=40;
                obstacle[i].color=1;
                obstacle[i].radius=0.75;
            }
            else if(i<23)
            {
                obstacle[i].x=20*(i-14);
                obstacle[i].y=20;
                obstacle[i].color=1;
                obstacle[i].radius=0.75;
            }
            else if(i<28)
            {
                obstacle[i].x=40+20*(i-23);
                obstacle[i].y=0;
                obstacle[i].color=1;
                obstacle[i].radius=0.75;
            }
            else if(i<37)
            {
                obstacle[i].x=-20+20*(i-28);
                obstacle[i].y=-20;
                obstacle[i].color=1;
                obstacle[i].radius=0.75;
            }
            else if(i<44)
            {
                obstacle[i].x=-40+20*(i-37);
                obstacle[i].y=-40;
                obstacle[i].color=1;
                obstacle[i].radius=0.75;
            }
            else if(i<51)
            {
                obstacle[i].x=-60+20*(i-44);
                obstacle[i].y=-60;
                obstacle[i].color=0;
                obstacle[i].radius=0.75;
            }
        }
            //第一行最后一行以外的4各黑球
        obstacle[7].color=0;
        obstacle[13].color=0;
        obstacle[37].color=0;
        obstacle[43].color=0;
        int temp[12]={17,18,26,27,28,29,30,32,34,36,37,41};//绿球颜色赋值
        for(int i=0;i<12;i++)
        {
            obstacle[temp[i]-1].color = 3;
        }
    }
};
struct map5_sim  //上海1
{
    ball obstacle[17];//0,黑色固定  1红色移动,前5个定义移动浮标,后面定义固定浮标.
    map5_sim()
    {
        double temp_x1[5]={105,240,320,360,430};
        double temp_y1[5]={0,40,-20,10,0};
        for(int i=0;i<5;i++)
        {
            obstacle[i].x=temp_x1[i];
            obstacle[i].y=temp_y1[i];
            obstacle[i].color=1;
            obstacle[i].radius=1;
        }
        double temp_x2[12]={20,60,100,140,180,220,260,300,340,380,420,460};
        double temp_y2[12]={100,-100,100,-100,100,-100,100,-100,100,-100,100,-100};
        for(int i=0;i<12;i++)
        {
            obstacle[i+5].x=temp_x2[i];
            obstacle[i+5].y=temp_y2[i];
            obstacle[i+5].color=0;
            obstacle[i+5].radius=1;
        }
    }

};
struct map6_sim//上海项目2
{
    ball obstacle[3];//圆形浮标
    buoy rectangle[26];//长方形浮标,通过口
    map6_sim()
    {
        double tempx[3]={0,25,30};
        double tempy[3]={-20,10,-8};
        for(int i=0;i<3;i++)
        {
            obstacle[i].x=tempx[i];
            obstacle[i].y=tempy[i];
            obstacle[i].radius=1;
            obstacle[i].color=1;
        }
        for(int j=0;j<13;j++)
        {
            rectangle[j*2+0].x = 15;
            rectangle[j*2+0].y = 15-j*2.5;
            rectangle[j*2+1].x = 16;
            rectangle[j*2+1].y = 15-j*2.5;
            if(j==6) //第3道浮标为空
            {
                rectangle[j*2+0].x = 0;
                rectangle[j*2+0].y = 0;
                rectangle[j*2+1].x = 0;
                rectangle[j*2+1].y = 0;
            }
        }
    }
};
struct map7_sim//上海项目3
{
    buoy rectangle[26*6];
    map7_sim()
    {
        for(int i=0;i<6;i++)
        {
            for(int j=0;j<13;j++)
            {
                rectangle[i*26+j*2+0].x = 10+10*i;
                rectangle[i*26+j*2+0].y = 15-j*2.5;
                rectangle[i*26+j*2+1].x = 11+10*i;
                rectangle[i*26+j*2+1].y = 15-j*2.5;
            }
        }
        //第一列第3道浮标为空
        rectangle[0*26+6*2+0].x = 0;
        rectangle[0*26+6*2+0].y = 0;
        rectangle[0*26+6*2+1].x = 0;
        rectangle[0*26+6*2+1].y = 0;
        //第二列第2道浮标空
        rectangle[1*26+4*2+0].x = 0;
        rectangle[1*26+4*2+0].y = 0;
        rectangle[1*26+4*2+1].x = 0;
        rectangle[1*26+4*2+1].y = 0;
        //第三列第4道浮标空
        rectangle[2*26+8*2+0].x = 0;
        rectangle[2*26+8*2+0].y = 0;
        rectangle[2*26+8*2+1].x = 0;
        rectangle[2*26+8*2+1].y = 0;
        //第4列第3道浮标空
        rectangle[3*26+6*2+0].x = 0;
        rectangle[3*26+6*2+0].y = 0;
        rectangle[3*26+6*2+1].x = 0;
        rectangle[3*26+6*2+1].y = 0;
        //第5列第3道浮标空
        rectangle[4*26+6*2+0].x = 0;
        rectangle[4*26+6*2+0].y = 0;
        rectangle[4*26+6*2+1].x = 0;
        rectangle[4*26+6*2+1].y = 0;
        //第六列第2道浮标空
        rectangle[5*26+4*2+0].x = 0;
        rectangle[5*26+4*2+0].y = 0;
        rectangle[5*26+4*2+1].x = 0;
        rectangle[5*26+4*2+1].y = 0;
    }
};
class sys
{
public:
    sys()
    {
        pub_globalObstacle = nh.advertise <sys_sim::globalObstacle>("globalObstacle",1);
        pub_localObstacle = nh.advertise <sys_sim::localObstacle>("localObstacle",1000);
        pub_usv = nh.advertise <sys_sim::usv>("usv",1000);

        pub_gpfpd = nh.advertise<sensor_msgs::NavSatFix>("unionstrong/gpfpd",1000);
        pub_cloud = nh.advertise<std_msgs::Float64MultiArray>("velodyne_points",1000);
        pub_photo = nh.advertise<std_msgs::ByteMultiArray>("input_photo",10);

        sub_vtg = nh.subscribe("vtg",1000,&sys::onmsg_vtg,this);
        pub_sep = 0;
        printf("success connected!\n");

        unsigned seed;//生成随机数
        seed = time(0);
        srand(seed);
        
    }
    void onmsg_vtg(const geometry_msgs::Vector3& vtg)
    {        
        //根据控制量更新无人船状态
        cout<<"vx "<<vtg.x<<"vy "<<vtg.y<<endl;
        boat1.vel = vtg.x;
        //保持朝向在0,2pi间,顺时针为正,0为正北朝向(y轴正向)
        boat1.hdt = boat1.hdt + vtg.y*M_PI/2*time_step;
        if(boat1.hdt < 0) boat1.hdt = boat1.hdt + M_PI*2;
        else if(boat1.hdt>M_PI*2) boat1.hdt = boat1.hdt - (M_PI*2);

        double trans_degree;
        trans_degree = boat1.hdt - M_PI/2;
        if(trans_degree < M_PI) trans_degree = -trans_degree;
        else  trans_degree = 2*M_PI - trans_degree;
        cout<<"vel:"<<boat1.vel<<" hdt:"<<boat1.hdt<<"trans_degree "<<trans_degree<<endl;
        boat1.x = boat1.x + boat1.vel*time_step*cos(trans_degree);
        boat1.y = boat1.y + boat1.vel*time_step*sin(trans_degree);
        cout<<"x "<<boat1.x<<"y "<<boat1.y<<endl;
        //update_boat();

    }
    void sensor_layer()//形成感知数据
    {
        int size1 = sizeof(map.obstacle)/sizeof(map.obstacle[0]);
        int size2 = sizeof(map.rectangle)/sizeof(map.rectangle[0]);
        

        //局部障碍物 localObstacle
        //范围：只关注距离无人船50范围内的障碍物
        //坐标系:以无人船位置为原点，无人船朝向为y轴正向，垂直右侧x轴正向建立平面直角坐标系；
        sys_sim::localObstacle local_ob;
        for(int i=0;i<size1;i++)
        {
            double x_ob = map.obstacle[i].x;
            double y_ob = map.obstacle[i].y;
            double radius = map.obstacle[i].radius;
            int color = map.obstacle[i].color;
            if(((boat1.x-x_ob)*(boat1.x-x_ob)+(boat1.y-y_ob)*(boat1.y-y_ob))<2500) //只计算50m范围内的局部障碍物
            {
                //计算无人船距障碍物距离，和无人船朝向与障碍物方向夹角
                //夹角-180°到180°，顺时针为正，以船头朝向为0
                double distance = sqrt((boat1.x-x_ob)*(boat1.x-x_ob)+(boat1.y-y_ob)*(boat1.y-y_ob));
                double re_angle = atan2((y_ob-boat1.y),(x_ob-boat1.x));

                //将无人船艏向变换到一般极坐标系下-pi,pi
                double trans_angle = boat1.hdt - M_PI/2;
                if(trans_angle<=M_PI)
                    trans_angle = -trans_angle;
                else
                    trans_angle = 2*M_PI - trans_angle;

                //计算两角差值,并转换到(-pi,pi]下,此时是与y轴夹角
                double local_angle = re_angle - trans_angle;
                if(local_angle>M_PI)
                    local_angle -= M_PI*2;
                else if(local_angle<=-M_PI)
                    local_angle += M_PI*2;
                //转移到与x轴夹角
                local_angle = local_angle + M_PI/2;
                if(local_angle>M_PI)
                    local_angle -= M_PI*2;
                //局部障碍物信息
                local_ob.color.push_back(color);
                local_ob.radius.push_back(radius);
                local_ob.x.push_back(distance*cos(local_angle));
                local_ob.y.push_back(distance*sin(local_angle));              
            }
        }

        //全局障碍物 globalObstacle
        //范围：所有已经发现的障碍物
        //坐标系： 全局坐标系usv
        // for(int i=0;i<size;i++)
        // {
        //     global_ob.x.push_back(map.obstacle[i].x);
        //     global_ob.y.push_back(map.obstacle[i].y);
        //     global_ob.radius.push_back(map.obstacle[i].radius);
        //     global_ob.color.push_back(map.obstacle[i].color);
        // }
        sys_sim::globalObstacle global_ob;

        for(int i=0;i<size2/26;i++)
        {
            int min = -1;
            int max = -1;
            int middle = -1;
            for(int j=0;j<13;j++)
            {
                double temp_dis1 = 1000;
                double temp_dis2 = 1000;
                cout<<"fabs"<<fabs(map.rectangle[i*26+j*2+0].x)<<endl;
                if(fabs(map.rectangle[i*26+j*2+0].x)>0.1)
                {
                    temp_dis1 = sqrt(pow(map.rectangle[i*26+j*2+0].x - boat1.x,2)+
                    pow(map.rectangle[1*26+j*2+0].y - boat1.y,2));
                    temp_dis2 = sqrt(pow(map.rectangle[i*26+j*2+1].x - boat1.x,2)+
                    pow(map.rectangle[1*26+j*2+1].y - boat1.y,2));
                }
                else if(fabs(map.rectangle[i*26+j*2+0].x)<0.1)
                {
                   middle = j;
                }
                cout<<"j "<<j<<" "<<temp_dis1<<endl;
                if(temp_dis1<10||temp_dis2<10)
                {
                        if(min == -1)
                        {
                            min = j;
                        }
                        else
                        {
                            max = j;
                        }
                }

            }
            cout<<"min "<<min<<"middle"<<middle<<"max"<<max<<endl;
            if(min == -1&& max == -1)
                continue;
            else if(min<middle&&middle<max)
            {
                if(min==middle-1)
                {
                    global_ob.x.push_back(map.rectangle[i*26+(middle-2)*2+0].x);
                    global_ob.y.push_back(map.rectangle[i*26+(middle-2)*2+0].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+(middle-2)*2+1].x);
                    global_ob.y.push_back(map.rectangle[i*26+(middle-2)*2+1].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+0].x);
                    global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+0].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+1].x);
                    global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+1].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);
                }
                else if(min<middle-1)
                {
                    if(min==0)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(min)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                        
                    }
                    else if(min>0)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle-1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle-1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }

                }
                if(max==middle+1)
                {
                    global_ob.x.push_back(map.rectangle[i*26+max*2+0].x);
                    global_ob.y.push_back(map.rectangle[i*26+max*2+0].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+max*2+1].x);
                    global_ob.y.push_back(map.rectangle[i*26+max*2+1].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+0].x);
                    global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+0].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);

                    global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+1].x);
                    global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+1].y);
                    global_ob.color.push_back(0);
                    global_ob.radius.push_back(1.25);
                }
                else if(max>middle+1)
                {
                    if(max == 12)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(middle+1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle+1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle+1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle+1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+max*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+max*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+max*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+max*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);


                    }
                    else if(max<12)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(middle+1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle+1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(middle+1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(middle+1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }

                }
            }
            else if(middle == -1)
            {
                if(max == -1)
                {
                    if(min == 0)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+0*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+0*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+0*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+0*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+1*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+1*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+1*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+1*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                    else if(min == 12)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+11*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+11*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+11*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+11*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+12*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+12*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+12*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+12*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                    else
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min+1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min+1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min+1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min+1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                }
                else
                {
                    if(min==0)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+0*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+0*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+0*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+0*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                    else if(min>0)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(min-1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(min-1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                    if(max==12)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+12*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+12*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+12*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+12*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                    else if(max<12)
                    {
                        global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+0].x);
                        global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+0].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);

                        global_ob.x.push_back(map.rectangle[i*26+(max+1)*2+1].x);
                        global_ob.y.push_back(map.rectangle[i*26+(max+1)*2+1].y);
                        global_ob.color.push_back(0);
                        global_ob.radius.push_back(1.25);
                    }
                }
                
            }
        }

        //cout<<flag1<<endl;
        if(pub_flag == 0)
        {
            //发布障碍物信息
            pub_localObstacle.publish(local_ob);
            // if(flag1 == 1)
            // {
            //     pub_globalObstacle.publish(global_ob);
            //     printf("************\n*************\n");
            // }
            if(global_ob.x.size()>0)
            pub_globalObstacle.publish(global_ob);
        }
        // else if(pub_flag == 1)
        // {
        //     std_msgs::Float64MultiArray temp_msg1;
        //     std_msgs::ByteMultiArray temp_msg2;
        //     for(int i=0;i<local_ob.x.size();i++)
        //     {
        //         temp_msg1.data.push_back(local_ob.x[i]);
        //         temp_msg1.data.push_back(local_ob.y[i]);
        //         temp_msg1.data.push_back(temp_ob.x[i]);
        //         temp_msg1.data.push_back(temp_ob.y[i]);
        //         temp_msg1.data.push_back(local_ob.radius[i]);
        //         temp_msg1.data.push_back(local_ob.color[i]);
                
        //         temp_msg2.data.push_back(local_ob.color[i]);
        //     }
            
        //     pub_cloud.publish(temp_msg1);
        //     pub_photo.publish(temp_msg2);
        // }
    }
    void pub_state()
    {
        //发布无人船状态，保存在3维向量中，x，y分量表示位置，z分量表示艏向
        sys_sim::usv usv;
        usv.x = boat1.x;
        usv.y = boat1.y;
        usv.hdt = boat1.hdt;
        usv.vel = boat1.vel;
        usv.radius = boat1.radius;

        sensor_msgs::NavSatFix temp_msg;
        temp_msg.latitude = boat1.y;
        temp_msg.longitude = boat1.x;
        temp_msg.position_covariance[1] = boat1.hdt;
        temp_msg.position_covariance[5] = boat1.vel;
        temp_msg.position_covariance[0] = boat1.radius;

        if(pub_flag == 0)
            pub_usv.publish(usv);
        else if(pub_flag == 1)
            pub_gpfpd.publish(temp_msg);
        sensor_layer();//形成感知层数据并发布
    }
    void update_boat()//针对map5的移动浮标进行位置更新
    {

        for(int i=0;i<5;i++)
        {
            map.obstacle[i].x += (rand()%10000)*0.0002-1;
            map.obstacle[i].y += (rand()%10000)*0.0002-1;
        }
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub_localObstacle;
    ros::Publisher pub_globalObstacle;
    ros::Publisher pub_boatObstacle;
    ros::Publisher pub_usv;
    ros::Publisher pub_gpfpd;
    ros::Publisher pub_cloud;
    ros::Publisher pub_photo;
    ros::Subscriber sub_vtg;
    int pub_sep;
    map6_sim map;
    
    boat boat1;
};
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"sys_sim");
    sys sim;
    //ros::Time::init();//an error https://blog.csdn.net/github_39611196/article/details/83046478
    //在声明对象后放置loop_rate可以防止以上错误
    int f = static_cast<int>(1/time_step);//发布消息频率与time_step保持一致
    ros::Rate loop_rate(f);//f为发布频率 
    
    while(ros::ok())
    {
        sim.pub_state();//发布环境信息，感知层信息
        ros::spinOnce();
        loop_rate.sleep();//睡眠相应时间
    }
}


