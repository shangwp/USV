#include <ros/ros.h>
#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "sim/example.hpp"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  lcm::LCM lcm;
  if (!lcm.good())
  {
    return 1;
  }
  
  sim::example my_data;
  my_data.a = 1.1;
  my_data.b = 2.1;
  lcm.publish("EXAMPLE", &my_data);//第一个参数是通道名，第二个参数是数据指针，第三个参数是长度
  std::cout << "发送成功!";
 
  ros::spinOnce();
}
