#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unistd.h>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
using namespace std;

typedef pair<int, int> pairii;
typedef pair<double, double> pairdd;

vector<pairii> getCellLists(Eigen::Vector2d pt1, Eigen::Vector2d pt2, double res, Eigen::Vector2d origin);

class VisibleGridNode
{
public:
    VisibleGridNode();
    
    void gridUpdate();  // 从yaml文件中初始化部分参数
    
private:
    void occupancygridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void linkposeCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
    void intrinsicsCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    void publishloopCallback(const ros::TimerEvent& event);
    
    ros::NodeHandle n_;
    
    // 地图信息和相机、目标的位姿
    nav_msgs::OccupancyGrid grid_msg_;
    double res_;
    Eigen::Vector2d origin_;
    int map_width_;
    int map_height_;
    Eigen::Vector3d t_wc_;
    Eigen::Quaterniond R_wc_;
    bool is_grid_init_;
    bool is_camera_init_;

    // 相机link的名字
    string camera_name_ = "iris_0::base_link";

    // 相机参数
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    int h_;
    int w_;
    bool is_intrinsics_init_;

    double max_dist_;
    vector<signed char> visible_grid_;

    ros::Publisher gridPub_;

    ros::Subscriber gridSub_;
    ros::Subscriber linkposeSub_;
    ros::Subscriber intrinsicsSub_;

    ros::Timer publishloop_timer_;
};


