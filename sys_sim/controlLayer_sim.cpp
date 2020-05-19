
#include <ros/ros.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <geometry_msgs/Vector3.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim//targetPoint.h>
#include <sys_sim/usv.h>
//实现功能
/*
    控制层接受目标点序列,更新目标点
    控制层自主避障
*/
using namespace std;

double k=0.2; 		//横向偏转比例


class controlLayer
{
public:
    controlLayer()
    {
        

        sub_usv = nh.subscribe("usv",1000,&controlLayer::onmsg_usv,this);
        sub_targetPoint = nh.subscribe("targetPoint",1000,&controlLayer::onmsg_target,this);
        sub_localObstacle = nh.subscribe("localObstacle",1000,&controlLayer::onmsg_localOb,this);
        //sub_taskPoint = nh.subscribe("taskPoint",1000,&controlLayer::onmsg_task,this);
        pub_vtg = nh.advertise <geometry_msgs::Vector3>("vtg",1000);
        pub_target_feedback = nh.advertise<sys_sim::targetPoint>("target_feedback",1000);
        //初始化参数
        task_flag=0;

    }

void onmsg_usv(const sys_sim::usv& msg)
{   //控制层主程序，处理信息并发布控制量
    //首先保存接受到的无人船信息
    usv.x = msg.x;
    usv.y = msg.y;
    usv.hdt = msg.hdt;
    cout<<"usv: "<<usv.x<<" "<<usv.y<<" "<<usv.hdt<<endl;
    //判断控制行为
    int control_flag = cal_task();//判断控制行为,返回值0循迹,1避障,2任务,-1停止
    cout<<"control_flag= "<<control_flag<<endl;
    target_feedback();//发布目标点反馈,将软硬度为1的目标点返回.
    if(control_flag == -1)
    {
        geometry_msgs::Vector3 vtg;
        //控制标志-1,停止
        vtg.x = 0;
        vtg.y = 0;
        pub_vtg.publish(vtg);
        return ;
    }
    else if(control_flag == 2)
    {
        //控制标志2,执行救援任务
        rescue_task();
        return ;
    }
    else if(control_flag == 1)
    {
        //控制标志1,执行避障
        obstacle_avoidance();
        return ;
    }
    else if(control_flag == 0)
    {
        //控制标志0,执行循迹
        // stanley_method();
        change_hdt();//只改变角度
        return ;
    }
}
void onmsg_localOb(const sys_sim::localObstacle& msg)
{
    local_obstacle.x.clear();
    local_obstacle.y.clear();
    local_obstacle.color.clear();
    local_obstacle.radius.clear();
    //保存接收到的障碍物
    for(int i = 0; i< msg.x.size(); i++)
    {
        local_obstacle.x.push_back(msg.x[i]);
        local_obstacle.y.push_back(msg.y[i]);
        local_obstacle.color.push_back(msg.color[i]);
        local_obstacle.radius.push_back(msg.radius[i]);
    }
}
void onmsg_target(const sys_sim::targetPoint& msg)
{
    target_point.x.clear();
    target_point.y.clear();
    target_point.z.clear();
   //保存接收到的目标点
    task_number = 0;//每次更新目标序列重置目标点序号
    for(int i = 0; i < msg.x.size(); i++)
    {
        target_point.x.push_back(msg.x[i]);
        target_point.y.push_back(msg.y[i]);
        target_point.z.push_back(msg.z[i]);
    }
    sys_sim::targetPoint feedback;
    double tempx=0;
    double tempy=0;
    int tempz=-1;
    feedback.x.push_back(tempx);
    feedback.y.push_back(tempy);
    feedback.z.push_back(tempz);
    pub_target_feedback.publish(feedback);
}
int cal_task()
{
    cout<<"*************"<<endl;
    // if(target_point.z.size()!=0&&target_point.z[task_number]==2)
    // {
    //     //软硬标志位设为1,表示必须先完成
    //     target_point.z[task_number]=1;
    //     task_flag = 1;
    // }
    cout<<"task_Number== "<<task_number<<endl;
    if(task_number==target_point.x.size())
    {
        //当前目标点序列执行完毕
        return -1;
    }
    //计算无人船与目标点距离
    double temp_dis;
    cout<<"usv.x= "<<usv.x<<"usv.y= "<<usv.y<<endl;
    cout<<"tagetx= "<<target_point.x[task_number]<<"targety= "<<target_point.y[task_number]<<endl;
    
    temp_dis = sqrt(pow((usv.x-target_point.x[task_number]),2)
                + pow((usv.y-target_point.y[task_number]),2));
    cout<<"temp_dis= "<<temp_dis<<endl;
    cout<<"task_flag= "<<task_flag<<endl;
    //判断目标黄球
    if(target_point.z[task_number]==2||task_flag == 1)
    {
        //如果存在目标，判断是否完成任务，是否需要执行撞击，全否则判断避障
        if(temp_dis<10)
        {

            task_flag = 1;
            cout<<"set task_flag= "<<task_flag<<endl; 


            target_point.z[task_number]=1;
            for(int i=0;i<local_obstacle.x.size();i++)
            {
                double temp_dis_ball;
                temp_dis_ball =sqrt(pow(local_obstacle.x[i],2)+
                                    pow(local_obstacle.y[i],2));
                if(local_obstacle.color[i]==4&&temp_dis_ball<0.75)
                {
                    //与障碍求距离小于0.5说明完成任务,更新目标点.
                    cout<<"task is over"<<endl;
                    update_target();
                    
                }
                if(local_obstacle.color[i]==4&&temp_dis_ball<10)
                {
                    return 2;//返回值2,表示执行任务
                }
            }
        }
        return 0;
    }
    else
    {
        //判断如果与目标点小于2m,更新目标点,由于是否避障不影响目标点的更新,所以更新置前.
        if(temp_dis<2)
        {
            update_target();
        }
        //检测避障
        for(int i=0;i<local_obstacle.x.size();i++)
        {
            double temp_dis_ball;
            temp_dis_ball = sqrt(pow(local_obstacle.x[i],2)+pow(local_obstacle.y[i],2));
            cout<<"temp_dis_ball= "<<temp_dis_ball<<endl;
            if(temp_dis_ball<5)
            {
                return 1;//返回值1执行避障
            }
        }
        return 0;
    }
}

void stanley_method()
{
    geometry_msgs::Vector3 vtg;
    double x=target_point.x[task_number];
    double y=target_point.y[task_number];
    double z=1;
    cout<<"x: "<<x<<"y: "<<y<<endl;
    
    double distance;
    distance = sqrt((x-usv.x)*(x-usv.x)+(y-usv.y)*(y-usv.y));
    cout<<"distance: "<<distance<<endl;
    // if(distance<1)
    // {
    //     vtg.x=0;
    //     vtg.y=0;
    // }
    // else
    // {
    //船头朝向与目标点预期朝向差距
    double delta_h = usv.hdt - z;//艏向差，成分1
    if(delta_h>M_PI) delta_h = delta_h - 2*M_PI;
    else if(delta_h<-M_PI) delta_h = delta_h + 2*M_PI;
    double e_dis;//横向差，成分2
    double degree_u2p;
    double delta_e;
    degree_u2p=atan((x-usv.x)/(y-usv.y)); // 正无穷 M_PI/2 负无穷 -M_PI   0,0
    cout<<"pre_degree_u2p: "<<degree_u2p<<endl;
    if(y<=usv.y&&x>usv.x) degree_u2p=M_PI+degree_u2p;
    if(y<=usv.y&&x<=usv.x) degree_u2p=-M_PI+degree_u2p;
    double delta_a=degree_u2p-z;
    if(delta_a>M_PI) delta_a=delta_a-2*M_PI;
    if(delta_a<-M_PI) delta_a=delta_a+2*M_PI;
    e_dis=fabs(distance*sin(delta_a));
    double et=atan(k*e_dis*e_dis/1/2);
    cout<<"delta_a: "<<delta_a<<"degreee_u2p: "<<degree_u2p<<endl;
    if(delta_a>=0)
    {
        et=-et;
    }
    cout<<"e_dis: "<<e_dis<<"et: "<<et<<"delta: "<<delta_h<<endl;
    double delta_sum_result = et+delta_h;
    if(delta_sum_result>M_PI) delta_sum_result=delta_sum_result-2*M_PI;
    if(delta_sum_result<-M_PI) delta_sum_result=delta_sum_result+2*M_PI;
    cout<<"delta_sum_result "<<delta_sum_result<<endl;
    vtg.x=1; //速度默认接近1m/s
    if(delta_sum_result>=0)  //控制量大于零，即艏向在顺时针方向大于目标方向，将控制量变小，逆时针运动
    {
        //ni  -1
        if(delta_sum_result<=M_PI/2)             //90度以内比例控制方向，90度为满舵
        {
            vtg.y=-(delta_sum_result)/M_PI*2; 
        }
        else                          //大于90度直接满舵
        {
            vtg.y=-1;              
        }
            
    }
    else if(delta_sum_result<0)
    {
        //顺时针转向
        //shun  +1
        if(delta_sum_result >= -M_PI/2)
        {
            vtg.y = -delta_sum_result /M_PI*2;
        }	
        else
        {
            vtg.y=1;
        }
    }
    // }
    cout<<"x: "<<vtg.x<<endl<<"y: "<<vtg.y<<endl;
    pub_vtg.publish(vtg);
}
void change_hdt()
{
    double temp_x = target_point.x[task_number];
    double temp_y = target_point.y[task_number];
    double degree_usv2point;
    degree_usv2point = atan2(temp_y - usv.y,temp_x -usv.x);

    //无人船艏向0,2pi转换到常规极坐标角度-pi,pi
    double trans_hdt = usv.hdt;
    trans_hdt -= M_PI/2;
    if(trans_hdt<M_PI)
        trans_hdt = -trans_hdt;
    else
        trans_hdt = M_PI*2-trans_hdt;
    //无人船朝向与局部目标点方向夹角
    double delta_degree = degree_usv2point - trans_hdt;
    if(delta_degree>M_PI)
        delta_degree -= M_PI*2;
    else if(delta_degree<-M_PI)
        delta_degree += M_PI*2;
    double v=1;
    cal_control(delta_degree,v);
}
void rescue_task()
{
    printf("Hello,I'm in rescuing task!\n");
    //用于保存到障碍物的相对角度
    double degree2obstcale=0;
    for(int i=0;i<local_obstacle.x.size();i++)
    {
        double temp_x = local_obstacle.x[i];
        double temp_y = local_obstacle.y[i];
        double radius = local_obstacle.radius[i];
        double temp_dis = sqrt(pow(temp_x,2) + pow(temp_y,2));
        if(temp_dis<10&&local_obstacle.color[i]==4)
        {
             degree2obstcale=atan2(temp_y,temp_x);
            degree2obstcale -= M_PI/2;
            if(degree2obstcale<=0)
                degree2obstcale = -degree2obstcale;
            else
                degree2obstcale = 2*M_PI - degree2obstcale;
            break;
        }
        //如果没有发现10m内的黄球
        if(i==(local_obstacle.x.size()-1))
        {
            printf("can't find target");
        }
    }
    if(degree2obstcale>M_PI)
        degree2obstcale -= M_PI*2;
    double v = 0.5;
    cal_control(degree2obstcale,v);
}
void obstacle_avoidance()
{
    printf("Hello,I'm in obstacle avoidance!\n");
    //建立一个vector保存障碍分区
    printf("0");
    vector<int> obstacle_area;
    //遍历障碍物,将10m内的障碍物保存到障碍分区中
    printf("1");
    cal_obstacle_area(obstacle_area);
    //在现有区域内找到最近的不碰撞区域,与0邻接的最近的无数字区域.
    int target_area=360;
    int target_flag=0;//用于标记是否找到
    while(true)
    {
        //两个标记,target_area和i同时搜索,找到不在障碍去的即可,没有则加,如果同时不在障碍区,优先左侧.
        int i=0;
        int his_flag1=0;
        int his_flag2=0;
        for(int j=0;j<obstacle_area.size();j++)
        {
            if(obstacle_area[j]==360-i)
            {
                his_flag1 = 1;
            }
            if(obstacle_area[j]==target_area)
            {
                his_flag2 = 1;
            }
            if(his_flag1==1&&his_flag2==1)
                break;
        }
        if(his_flag1==0)
        {
            target_area = i;
            break;
        }
        if(his_flag2==0)
        {
            break;
        }
        i++;
        target_area--;
    }
    //转换为对应弧度制,本身为相对角度,不需要再转换,为正,需要顺时针转动
    double target_degree =(double)target_area*M_PI/180;
    if(target_degree>M_PI)
        target_degree -= M_PI*2;
    double v=1;
    cal_control(target_degree,v);
}
void cal_obstacle_area(vector<int>& area)
{
    printf("2");
    //将10m内的障碍物所在区间设为集合
    for(int i=0;i<local_obstacle.x.size();i++)
    {
        double temp_x = local_obstacle.x[i];
        double temp_y = local_obstacle.y[i];
        double radius = local_obstacle.radius[i];
        double temp_dis = sqrt(pow(temp_x,2) + pow(temp_y,2));

        if(temp_dis<10)
        {
            //计算障碍物所在的角度,化为0,2pi,y轴正向为0,顺时针为正弧度化为角度
            double degree2obstcale=atan2(temp_y,temp_x);
            degree2obstcale -= M_PI/2;
            if(degree2obstcale<=0)
                degree2obstcale = -degree2obstcale;
            else
                degree2obstcale = 2*M_PI - degree2obstcale;
            degree2obstcale = degree2obstcale*180/M_PI;
            //顺时针正向,计算障碍物区间的左翼和右翼,由于是局部障碍物,默认船头在0
            double left_side,right_side;
            left_side = degree2obstcale - (radius+usv.radius)/temp_dis;
            right_side = degree2obstcale + (radius+usv.radius)/temp_dis;
            if(left_side<0) left_side += 360;
            if(right_side>360) right_side -=360;
            //向下取整
            for(int i=floor(left_side);i<=floor(right_side);i++)
            {
                //加入前先查重,不重复才加入
                int repeat_flag = 0;
                for(int j=0;j<area.size();j++)
                {
                    if(area[j]==i)
                    {
                        repeat_flag = 1;
                        break;
                    }
                }
                if(repeat_flag==0)
                area.push_back(i);
            }
        }
    }
}
void cal_control(double delta,double v)
{
    //通过delta = 目标 - 无人船艏向,范围-pi,pi ,v速度控制量
    geometry_msgs::Vector3 vtg;//定义控制量
    if(delta>=0)
    {
        if(delta<M_PI/2)
        {
            vtg.y = -delta/M_PI*2;
        }
        else
        {
            vtg.y = -1;
        }
    }
    if(delta<0)
    {
        if(delta>-M_PI/2)
        {
            vtg.y = -delta/M_PI*2;
        }
        else
        {
            vtg.y = 1;
        }
    }
    vtg.x = v;
    pub_vtg.publish(vtg);
}
void update_target()
{
    cout<<"update target_point"<<endl;
    task_flag = 0;//任务完成,先将标志置为0;
    cout<<"set_task= "<<task_flag<<endl;
    task_number++;//更新目标点
    //2为附带任务目标点,一旦开始任务就需要执行完成
    // if(target_point.z[task_number]==2)
    // {
    //     //软硬标志位设为1,表示必须先完成
    //     target_point.z[task_number]=1;
    //     task_flag = 1;
    // }
}
void target_feedback()
{
    sys_sim::targetPoint feedback;
    if(task_number==target_point.x.size())//没有目标点时
    {
        double tempx=0;
        double tempy=0;
        int tempz=-2;
        feedback.x.push_back(tempx);
        feedback.y.push_back(tempy);
        feedback.z.push_back(tempz);
        pub_target_feedback.publish(feedback);
        return ;
    }
    for(int i=task_number;i<target_point.x.size();i++)
    {
        if(target_point.z[i]==1)
        {
            feedback.x.push_back(target_point.x[i]);
            feedback.y.push_back(target_point.y[i]);
            feedback.z.push_back(target_point.z[i]);
        }
    }
    if(feedback.x.size()==0)//时刻回传当前目标点
    {
        feedback.x.push_back(target_point.x[task_number]);
        feedback.y.push_back(target_point.y[task_number]);
        feedback.z.push_back(target_point.z[task_number]);
    }
    pub_target_feedback.publish(feedback);
}
private:


    ros::NodeHandle nh;
    ros::Publisher pub_vtg;
    ros::Publisher pub_target_feedback;
    ros::Subscriber sub_usv;
    ros::Subscriber sub_targetPoint;
    ros::Subscriber sub_localObstacle;
    //保存接收信息
    sys_sim::usv usv;//无人船信息
    sys_sim::targetPoint target_point;//目标点序列
    sys_sim::localObstacle local_obstacle;//障碍物
    //标志位
    int task_number;//当前进行的目标点序号
    bool task_flag;//到目标点后执行任务的标志,完成任务后才可以更新目标点,默认为0,有任务时为1,任务完成置0
};




int main(int argc,char ** argv)
{
    
    ros::init(argc,argv,"controlLayer");
    controlLayer control;
    ros::spin();
}
