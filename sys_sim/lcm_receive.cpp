#include <ros/ros.h>
#include <iostream>
#include "lcm/lcm-cpp.hpp"
#include "sim/example.hpp"
class MyMessageHandler 
{
public:
  void onMessage(const lcm::ReceiveBuffer* rbuf, const std::string& channel,const sim::example* msg) 
  {
    std::cout << msg->a << std::endl;
    std::cout << msg->b << std::endl;
    std::cout << "接收成功!";
  }
};
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
 
  lcm::LCM lcm;
  MyMessageHandler handler;
  lcm.subscribe("EXAMPLE", &MyMessageHandler::onMessage, &handler);
  while (true)
    lcm.handle();
 
  ros::spin();
}

