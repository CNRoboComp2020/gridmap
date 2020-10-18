#include "VisibleGrid.h"
#include <iostream>
#include <sys/time.h>
//#include <xmlrpcpp/XmlRpcValue.h>
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

vector<pairii> getCellLists(Eigen::Vector2d pt1, Eigen::Vector2d pt2, double res, Eigen::Vector2d origin)
{
    Eigen::Vector2d pt1_ref = (pt1 - origin) / res;
    Eigen::Vector2d pt2_ref = (pt2 - origin) / res;
    double x1 = pt1_ref[0], y1 = pt1_ref[1];
    double x2 = pt2_ref[0], y2 = pt2_ref[1];
    vector<pairii> cell_list;
    if (fabs(x1-x2) < 1e-6 or floor(x1) == floor(x2))
    {
        if (y1 <= y2)
        {
            for (int y = int(floor(y1)); y <= int(floor(y2)); y++)
            {
                cell_list.push_back(pairii(int(floor(x1)), y));
            }
        }
        else
        {
            for (int y = int(floor(y1)); y >= int(floor(y2)); y--)
            {
                cell_list.push_back(pairii(int(floor(x1)), y));
            }
        }
    }
    else if (fabs(y1-y2) < 1e-6 or floor(y1) == floor(y2))
    {
        if (x1 <= x2)
        {
            for (int x = int(floor(x1)); x <= int(floor(x2)); x++)
            {
                cell_list.push_back(pairii(x, int(floor(y1))));
            }
        }
        else
        {
            for (int x = int(floor(x1)); x >= int(floor(x2)); x--)
            {
                cell_list.push_back(pairii(x, int(floor(y1))));
            }
        }
    }
    else
    {
        // 思路：求解线段与格线的所有交点，进而得到线段经过的所有格子
        // y=kx+b, x=ly+c
        bool inv;
        if (floor(x1) > floor(x2))
        {
            int temp;
            inv = true;
            temp = x1; x1 = x2; x2 = temp;
            temp = y1; y1 = y2; y2 = temp;
        }
        else
        {
            inv = false;
        }
        // 此后均按照x1<x2情况处理，cell_list从左往右
        vector<pairdd> intersect_list_x, intersect_list_y;
        double k = (y2-y1) / (x2-x1);
        double b = (x2*y1-x1*y2) / (x2-x1);
        for (int x = int(floor(x1))+1; x <= int(floor(x2)); x++)
        {
            intersect_list_x.push_back(pairdd(double(x), k*double(x)+b));
        }
        if (k > 0)
        {
            for (int y = int(floor(y1))+1; y <= int(floor(y2)); y++)
            {
                intersect_list_y.push_back(pairdd((double(y)-b)/k, double(y)));
            }
        }
        else
        {
            for (int y = int(floor(y1)); y >= int(floor(y2))+1; y--)
            {
                intersect_list_y.push_back(pairdd((double(y)-b)/k, double(y)));
            }
        }
        int ptrx = 0, ptry = 0;
        bool isX;
        while (ptrx < intersect_list_x.size() || ptry < intersect_list_y.size())  // 改用一次归并排序
        {
            if (ptry >= intersect_list_y.size())
            {
                isX = true;
            }
            else if (ptrx >= intersect_list_x.size())
            {
                isX = false;
            }
            else
            {
                isX = intersect_list_x[ptrx].first < intersect_list_y[ptry].first;
            }
            if (isX)
            {
                cell_list.push_back(pairii(int(intersect_list_x[ptrx].first)-1, int(floor(intersect_list_x[ptrx].second))));
                ptrx++;
            }
            else
            {
                if (k > 0)
                {
                    cell_list.push_back(pairii(int(floor(intersect_list_y[ptry].first)), int(intersect_list_y[ptry].second)-1));
                }
                else
                {
                    cell_list.push_back(pairii(int(floor(intersect_list_y[ptry].first)), int(intersect_list_y[ptry].second)));
                }
                ptry++;
            }
        }
        cell_list.push_back(pairii(int(floor(x2)), int(floor(y2))));
        if (inv)
        {
            reverse(cell_list.begin(), cell_list.end());
        }
    }
    return cell_list;
}

VisibleGridNode::VisibleGridNode()
{
    this->is_grid_init_ = false;
    this->is_camera_init_ = false;

    this->camera_name_ = "iris_0::base_link";
    
    this->is_intrinsics_init_ = false;

    this->max_dist_ = 30.0;

    this->gridPub_ = n_.advertise<nav_msgs::OccupancyGrid>("/available_grid", 100);

    this->gridSub_ = n_.subscribe("/occupancy_grid", 1, &VisibleGridNode::occupancygridCallback, this, ros::TransportHints().tcpNoDelay());  
    this->linkposeSub_ = n_.subscribe("/gazebo/link_states", 1, &VisibleGridNode::linkposeCallback, this, ros::TransportHints().tcpNoDelay());  
    this->intrinsicsSub_ = n_.subscribe("iris_0/usb_cam/camera_info", 1, &VisibleGridNode::intrinsicsCallback, this, ros::TransportHints().tcpNoDelay());

    this->publishloop_timer_ = n_.createTimer(ros::Duration(0.5), &VisibleGridNode::publishloopCallback, this);

}

void VisibleGridNode::gridUpdate()
{    
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);

    if (!this->is_camera_init_ || !this->is_intrinsics_init_ || !this->is_grid_init_)
    {
        return;
    }
    Eigen::Vector3d pix_list[4] = {Eigen::Vector3d(1.0, (this->cx_ - 0.0) / this->fx_, (this->cy_ - 0.0) / this->fy_), 
                                   Eigen::Vector3d(1.0, (this->cx_ - 0.0) / this->fx_, (this->cy_ - this->h_) / this->fy_), 
                                   Eigen::Vector3d(1.0, (this->cx_ - this->w_) / this->fx_, (this->cy_ - 0.0) / this->fy_), 
                                   Eigen::Vector3d(1.0, (this->cx_ - this->w_) / this->fx_, (this->cy_ - this->h_) / this->fy_)};
    double yaw_list[4];
    double yaw_max = -10, yaw_min = 10;
    for (int i = 0; i < 4; i++)
    {
        Eigen::Vector3d p_w = this->R_wc_ * pix_list[i];
        yaw_list[i] = atan2(p_w[1], p_w[0]);
        if (yaw_list[i] > yaw_max)
        {
            yaw_max = yaw_list[i];
        }
        if (yaw_list[i] < yaw_min)
        {
            yaw_min = yaw_list[i];
        }
    }
    if (yaw_max - yaw_min > M_PI)
    {
        yaw_max = -10;
        yaw_min = 10;
        for (int i = 0; i < 4; i++)
        {
            if (yaw_list[i] < 0)
            {
                yaw_list[i] += 2*M_PI;
            }
            if (yaw_list[i] > yaw_max)
            {
                yaw_max = yaw_list[i];
            }
            if (yaw_list[i] < yaw_min)
            {
                yaw_min = yaw_list[i];
            }
        }
    }
    //if (this->visible_grid_.size() == 0)
    if (true)
    {
        this->visible_grid_ = vector<signed char>(this->map_width_ * this->map_height_);
    }
    double yaw = yaw_min;
    double yaw_step = this->res_ / this->max_dist_;
    Eigen::Quaterniond R_wc_inv = this->R_wc_.conjugate();
    Eigen::Vector3d t_wc_vec = this->t_wc_;
    while (yaw <= yaw_max)
    {
        Eigen::Vector2d endpoint;
        Eigen::Vector2d t_wc_xy = Eigen::Vector2d(t_wc_vec[0], t_wc_vec[1]);
        endpoint = t_wc_xy + this->max_dist_ * Eigen::Vector2d(cos(yaw), sin(yaw));
        vector<pairii> cell_list = getCellLists(t_wc_xy, endpoint, this->res_, this->origin_);
        for (auto iter = cell_list.begin(); iter != cell_list.end(); iter++)
        {
            if (iter->first < 0 || iter->first >= this->map_width_ || iter->second < 0 || iter->second >= this->map_height_)
            {
                break;
            }
            if (this->grid_msg_.data[iter->second*this->map_width_ + iter->first] > 0)
            {
                break;
            }
            double t_wt_x = this->origin_[0] + (double(iter->first) + 0.5) * this->res_;
            double t_wt_y = this->origin_[1] + (double(iter->second) + 0.5) * this->res_;
            Eigen::Vector3d t_ct = R_wc_inv * (Eigen::Vector3d(t_wt_x, t_wt_y, 0.0) - t_wc_vec);
            double u = this->fx_ * (-t_ct[1]/t_ct[0]) + this->cx_;
            double v = this->fy_ * (-t_ct[2]/t_ct[0]) + this->cy_;

            if (t_ct[0] < 0 || u < 0 || u > double(this->w_) || v < 0 || v > double(this->h_))
            {
                continue;
            }
            this->visible_grid_[iter->second*this->map_width_ + iter->first] = 100;
        }
        yaw += yaw_step;
    }

    gettimeofday(&t2, NULL);
    double deltaT = (t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec;
    std::cout << "time consumed "<< deltaT << " us" <<std::endl;
}

void VisibleGridNode::occupancygridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    if (!this->is_grid_init_)
    {
        this->grid_msg_ = *msg;
        this->res_ = msg->info.resolution;
        this->origin_ = Eigen::Vector2d(msg->info.origin.position.x, msg->info.origin.position.y);
        this->map_width_ = msg->info.width;
        this->map_height_ = msg->info.height;
        this->is_grid_init_ = true;
    }
}

void VisibleGridNode::linkposeCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++)
    {
        if(msg->name[i] == this->camera_name_)
        {
            break;
        }
    }
    if(i < msg->name.size())
    {
        geometry_msgs::Pose pose = msg->pose[i];
        this->t_wc_ << pose.position.x, pose.position.y, pose.position.z;
        this->R_wc_ = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        this->is_camera_init_ = true;
    }
}

void VisibleGridNode::intrinsicsCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    this->fx_ = msg->K[0];
    this->fy_ = msg->K[4];
    this->cx_ = msg->K[2];
    this->cy_ = msg->K[5];
    this->h_ = msg->height;
    this->w_ = msg->width;
    this->is_intrinsics_init_ = true;
}

void VisibleGridNode::publishloopCallback(const ros::TimerEvent& event)
{
    if (this->visible_grid_.size() != 0)
    {
        nav_msgs::OccupancyGrid grid;
        grid.header.stamp = ros::Time::now();
        grid.info.map_load_time = ros::Time::now();
        grid.info.resolution = this->res_;
        grid.info.width = this->map_width_;
        grid.info.height = this->map_height_;
        grid.info.origin.position.x = this->origin_[0];
        grid.info.origin.position.y= this->origin_[1];
        grid.info.origin.position.z = 0;
        grid.info.origin.orientation.x = 0;
        grid.info.origin.orientation.y = 0;
        grid.info.origin.orientation.z = 0;
        grid.info.origin.orientation.w = 1;
        grid.data = this->visible_grid_;
        this->gridPub_.publish(grid);
    }
}

int main(int argc, char **argv)
{   
/*
    Eigen::Vector2d start(600, 500), end(800, 498.1), origin(0, 0);
    vector<pairii> cell_list = getCellLists(start, end, 1, origin);
    for (auto iter = cell_list.begin(); iter != cell_list.end(); iter++)
        cout << "(" << iter->first << "  " << iter->second << ")\n";
*/
    ros::init(argc, argv, "visible_grid_node");

    VisibleGridNode f;

    //ros::Rate loop_rate(100);
    while(ros::ok())
    {
        f.gridUpdate();
        ros::spinOnce();
        //loop_rate.sleep();
    }

    return 0;
}
