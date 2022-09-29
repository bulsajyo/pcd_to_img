#ifndef PRIP_
#define PRIP_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <opencv2/opencv.hpp>
#include <opencv2/cvwimage.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using Point = pcl::PointXYZ;
using Pointcloud = pcl::PointCloud<Point>;

class ProjectionIP{
private:
    //**** ROS node ****//
    ros::NodeHandle _nh;
    
    //*** config ***//
    float k1_vis = 5.0;
    float k2_vis = 95.0;
    std::vector<double> R, T, dF, dC; 
    std::string encoding_ = sensor_msgs::image_encodings::BGR8;

    Eigen::MatrixXf Pr = Eigen::MatrixXf::Zero(3,4); //= Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Rect = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tr = Eigen::Matrix4f::Identity();
    

    //*** camera info ***//
    bool is_camera_info=false;
    ros::Subscriber sub_cam_info;
    std::string topic_cam_info = "/camera/camera_info";
    int img_width = 0;
    int img_height = 0;

    //*** camera ***//
    bool is_cam=false;
    ros::Subscriber sub_img;
    cv::Mat image_;
    std::string topic_cam = "/camera/image_raw";
    ros::Time time_cam;

    //*** lidar ***//
    bool is_lidar=false;
    ros::Subscriber sub_pcd;
    std::string topic_lidar = "/os_cloud_node/points";
    ros::Time time_lidar;
    Pointcloud cloud;

    //*** new image ***//
    ros::Publisher pub_img_;
    std::string topic_newimg = "/image_p";

public:
    ProjectionIP(ros::NodeHandle* nodehandle);
    ~ProjectionIP();
    void get_config(ros::NodeHandle& nodehandle, const std::string& prefix);
    void callback_caminfo(const sensor_msgs::CameraInfo::ConstPtr& msg_img_info);
    void callback_img(const sensor_msgs::Image::ConstPtr& msg_img);
    void callback_lidar(const sensor_msgs::PointCloud2::ConstPtr& msg_lidar);
    void point2img(const Pointcloud&, const cv::Mat&);
    void publish_img(const cv::Mat& img_);
    void starter();
    cv::Vec3b set_pixel_color(float value, float k1, float k2);
};

#endif