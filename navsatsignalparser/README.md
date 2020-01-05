# NavSatSignalParser

ROS下集成惯导的解析程序，解析通用的GPFPD报文，获取经纬度、三轴角度等数据

### 使用方法

- 1 在一个命令行开启：

		roscore

- 2 在第二个命令行开启(ttyUSBO看连接的惯导设备号)：

		rosrun unionstrong_driver nmea_serial_driver _port:/dev/ttyUSB0 _baud:=115200

