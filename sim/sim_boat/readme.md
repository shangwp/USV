1.将sim_boat文件夹置于catkin_ws/src下
2.cd ~/catkin_ws
3.catkin_make
4.rosrun sim_boat sim_boat
5.同时启动track rosrun track track, 在开启终端的目录下会生成track_log.csv，track_set.csv。前者记录了循迹过程中的各种信息，后者记录了设定的轨迹。
6.将track_log.csv 与 vis/lat.py 置于同一目录下，运行lat.py,得到循迹的折线图。
