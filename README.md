Latest version: 将代码整合为catkin_workspace版本，新增了视野检测器（VisibleGrid），有C++和Python版本。Python版本不完善，速度也较慢，不建议使用。同时修改了部分其他代码。老版代码见版本v1.0。

可以roslaunch gridmap VisibleGrid_cpp_0.5.launch查看效果。

以下为老版README：

本repo包含用于绘制比赛赛场outdoor1.world二维栅格图的代码和相关数据，以及基于栅格图的虚拟目标检测器。栅格图的分辨率为0.1m，大小为1800*1000，栅格[0,0]的Gazebo坐标为(-50, -50)。栅格图的绘制高度为2.5m。

绘制二维栅格图时使用的.world文件为outdoor1_gas_station.npy（需要将hokuyo_lidar_no_gravity添加至~/.gazebo/models等Gazebo模型路径下）。具体的绘制方法为：运行roscore、rosrun gazebo_ros gazebo outdoor1_gas_station.npy、python MapDrawer.py后，拖动Gazebo图形界面中的LiDAR至想要绘制的区域即可更新该区域信息。

MapProcessor.py将.npy中的地图膨胀，并将闭合区域floodfill填充。

GridPublisher.py可以将.npy文件中的地图信息转化为OccupancyGrid信息并publish。

FakeDetector.py为虚拟目标检测器。


