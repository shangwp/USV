#include <math.h>
struct boat//无人船状态量
{
    double vel; //无人船速度（指速率，单位m/s）
    double hdt; //无人船艏向，正北为0°，顺时针方向为正[0~360)
    double x,y; //全局坐标
    double radius;
    boat()
    {
        vel = 0;
        hdt = M_PI/2;
        x = 0;
        y = 0;
        radius = 2;
    }
};
struct ball//障碍球信息
{
    double x,y;//球位置
    double radius;//球半径,统一定为0.75
    char color;//颜色，0黑色，1红色，2蓝色，3绿色，4黄色
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
                {20,30,60,105,140,155,170, 22, 32, 55, 98,120,130,160,180,45,112, 57,87,-150},
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