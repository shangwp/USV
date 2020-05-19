#include <ros/ros.h>
#include <stdlib.h>
#include <vector>

#include <sys_sim/localObstacle.h>
#include <sys_sim/globalObstacle.h>
#include <sys_sim/boatObstacle.h>
#include <sys_sim/usv.h>

using namespace std;


int main(int argc,char ** argv)
{
    ros::init(argc,argv,"sensorLayer");
    sensorLayer control;
    ros::spin();
}