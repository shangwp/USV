#include <ros/ros.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <sensor_msgs/NavSatFix.h>
#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim/photo_trans.h>
#include <sys_sim/radar_trans.h>
#include <sys_sim/usv.h>

using namespace std;
class dataFusion
{
public:
    dataFusion()
    {
        pub_globalObstacle = nh.advertise <sys_sim::globalObstacle>("globalObstacle",1);
        pub_localObstacle = nh.advertise <sys_sim::localObstacle>("localObstacle",1000);
        pub_usv = nh.advertise <sys_sim::usv>("usv",1000);

        sub_gpfpd = nh.subscribe("unionstrong/gpfpd",1000,&dataFusion::onmsg_gpfpd,this);
        sub_photo_trans = nh.subscribe("photo_trans",1000,&dataFusion::onmsg_photo_trans,this);
        sub_radar_trans = nh.subscribe("radar_trans",1000,&dataFusion::onmsg_radar_trans,this);
        pub_sep = 0;
        printf("success connected!\n");
    }
    void onmsg_photo_trans(const sys_sim::photo_trans& msg)
    {
        for(int i=0;i<msg.color.size();i++)
        {
            temp_color.push_back(msg.color[i]);
        }
    }
    void onmsg_radar_trans(const sys_sim::radar_trans& msg)
    {
        sys_sim::localObstacle local_ob;
        for(int i=0;i<msg.localx.size();i++)
        {
            local_ob.x.push_back(msg.localx[i]);
            local_ob.y.push_back(msg.localy[i]);
            local_ob.color.push_back(msg.color[i]);
            local_ob.radius.push_back(msg.radius[i]);
        }

        int flag1 = 0;
        if(global_ob.x.size()==0)
        {
            for(int i=0;i<msg.globalx.size();i++)
            {
                
                global_ob.x.push_back(msg.globalx[i]);
                global_ob.y.push_back(msg.globaly[i]);
                global_ob.color.push_back(msg.color[i]);
                global_ob.radius.push_back(msg.radius[i]);
            }
            flag1 = 1;
        }
        else
        {
            //判断重复的不要加进去
            for(int i=0;i<msg.globalx.size();i++)
            {
                int flag2 = 0;
                for(int j=0;j<global_ob.x.size();j++)
                {
                    if(fabs(msg.globalx[i]-global_ob.x[j])<0.1&&fabs(msg.globaly[i]-global_ob.y[j])<0.1)
                    {
                        flag2 = 1;
                        break;
                    }
                }
                if(flag2 == 0) //没有重复即为新
                {
                    flag1 = 1;//标志着有新加入
                    global_ob.x.push_back(msg.globalx[i]);
                    global_ob.y.push_back(msg.globaly[i]);
                    global_ob.color.push_back(msg.color[i]);
                    global_ob.radius.push_back(msg.radius[i]);
                }
            }
        }

        pub_localObstacle.publish(local_ob);
        // if(flag1 == 1)
        // {
        //     pub_globalObstacle.publish(global_ob);
        //     printf("************\n*************\n");
        // }
        if(pub_sep==1)
        {
            pub_globalObstacle.publish(global_ob);
        }
        pub_sep++;
        pub_sep=pub_sep%10;
    }
    void onmsg_gpfpd(const sensor_msgs::NavSatFix& msg)
    {
        sys_sim::usv usv;
        usv.x = msg.latitude;
        usv.y = msg.longitude;
        usv.hdt = msg.position_covariance[1];
        usv.vel = msg.position_covariance[5];
        usv.radius = msg.position_covariance[0];
        pub_usv.publish(usv);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_photo_trans;
    ros::Subscriber sub_radar_trans;
    ros::Subscriber sub_gpfpd;

    ros::Publisher pub_usv;
    ros::Publisher pub_localObstacle;
    ros::Publisher pub_globalObstacle;
    sys_sim::photo_trans temp_photo;
    sys_sim::globalObstacle global_ob;
    vector<char> temp_color;
    int pub_sep;
};
int main(int argc,char ** argv)
{
    ros::init(argc,argv,"data_fusion");
    dataFusion fusion;
    ros::spin();
}