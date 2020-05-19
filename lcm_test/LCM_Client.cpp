#include <iostream>
#include "lcm/lcm-cpp.hpp"
 
int main()
{
	lcm::LCM lcm;
	if (!lcm.good())
	{
		return 1;
	}
	char data[5];
	data[0] = 1;
	data[1] = 5;
	data[2] = 1;
	data[3] = 2;
	data[4] = 1;
	lcm.publish("EXAMPLE", data,5);//第一个参数是通道名，第二个参数是数据指针，第三个参数是长度
	std::cout << "发送成功!";
	return 0;
}


