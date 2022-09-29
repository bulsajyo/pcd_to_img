#include "pcd_to_img/projection_IP.hpp"

ProjectionIP::ProjectionIP(ros::NodeHandle* nodehandle) : _nh(*nodehandle){
    is_cam = false;
    is_lidar = false;
    get_config(_nh,"/pcd_to_img/");
    sub_cam_info = _nh.subscribe(topic_cam_info, 10, &ProjectionIP::callback_caminfo, this);
    sub_img = _nh.subscribe(topic_cam, 1, &ProjectionIP::callback_img, this);
    sub_pcd = _nh.subscribe(topic_lidar, 1, &ProjectionIP::callback_lidar, this);
    pub_img_ = _nh.advertise<sensor_msgs::Image>(topic_newimg, 20);
}

ProjectionIP::~ProjectionIP(){
    std::cout<< "Projection Ended" << std::endl;
}

void ProjectionIP::get_config(ros::NodeHandle& nh, const std::string& prefix){
    std::string tp_lidar_, tp_cam_, tp_caminfo_, tp_newimg_;
    std::vector<double> _R, _T, _dF, _dC, _P, _Rect; 
    bool use_cam_info_from_yaml;
    double k1_, k2_;
    
    bool res_tp = nh.param<std::string>(prefix + "topics/lidar", tp_lidar_, "") 
        && nh.param<std::string>(prefix + "topics/image", tp_cam_, "")
        && nh.param<std::string>(prefix + "topics/image_pub", tp_newimg_, "");
    bool res_caminfo = nh.param<std::string>(prefix + "topics/image_info", tp_caminfo_, "");
    
    bool res_use_info = nh.param(prefix + "camera_matrix/use_this", use_cam_info_from_yaml, false);
    bool res_cam = nh.param(prefix + "camera_matrix/P", _P, {}) && nh.param(prefix + "camera_matrix/Rect", _Rect, {});
    
    bool res_lidar = nh.param(prefix + "calib_lidar_to_cam/R", _R, {}) && nh.param(prefix + "calib_lidar_to_cam/T", _T, {});
			// && nh.param(prefix + "calib_lidar_to_cam/delta_f", _dF, {}) && nh.param(prefix + "calib_lidar_to_cam/delta_c", _dC, {});
    
    bool res_vis = nh.param(prefix + "visualization/k1", k1_, 0.0) && nh.param(prefix + "visualization/k2", k2_, 0.0);
    
    if (!res_tp || !res_use_info || !res_lidar || !res_vis || (use_cam_info_from_yaml && !res_cam)) {
		ROS_ERROR_STREAM("Failed to load configs");
	}
    else{
        if(!res_caminfo && !use_cam_info_from_yaml){
            ROS_ERROR_STREAM("Use camera_info from topic or config");
        }
        else{
            ROS_INFO("Config values loaded");
            std::cout << "====== Topics ======" << std::endl;
            std::cout << "camera : " << tp_cam_ << std::endl;
            if (res_caminfo) std::cout << "camera info : " << tp_caminfo_ << std::endl;
            std::cout << "lidar : " << tp_lidar_ << std::endl;
            std::cout << "img to be published : " << tp_newimg_ << std::endl;
            
            topic_lidar = tp_lidar_;
            topic_cam = tp_cam_;
            topic_cam_info = tp_caminfo_;
            topic_newimg = tp_newimg_;

            if(use_cam_info_from_yaml){
                for(int i=0;i<12;i++) Pr(i/4,i%4) = _P[i]; 
                for(int j=0;j<9;j++) Rect(j/3,j%3) = _Rect[j];
                
                std::cout << "====== Cam_info ======" << std::endl;
                std::cout << "camera matrix P : \n" << Pr << std::endl;
                std::cout << "camera matrix Rect : \n" << Rect << std::endl;

                is_camera_info = true;
            }

            for(int i=0;i<9;i++) Tr(i/3,i%3) = _R[i]; 
            for(int j=0;j<3;j++) Tr(j,3) = _T[j];
            std::cout << "====== Tr lidar to cam ======\n" << Tr << std::endl;

            std::cout << "====== Constants for visualization ======"<< std::endl;
            std::cout << "k1 : " << k1_ << " , k2 : " << k2_ << std::endl;
            k1_vis = k1_;
            k2_vis = k2_;
        }   
    }    
}

void ProjectionIP::callback_caminfo(const sensor_msgs::CameraInfo::ConstPtr& msg_img_info){
    if(!is_camera_info){
        for(int i=0;i<12;i++) Pr(i/4,i%4) = msg_img_info->P[i];
        for(int i=0;i<9;i++) Rect(i/3,i%3) = msg_img_info->R[i];
        std::cout << "P : \n" << Pr << std::endl;
        std::cout << "Rect : \n" << Rect << std::endl;
        is_camera_info = true;
    }
    return;
}

void ProjectionIP::callback_img(const sensor_msgs::Image::ConstPtr& msg_img){
    if (!is_camera_info){
        ROS_WARN("No camera info yet...");
        return;
    }
    
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg_img, encoding_);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    time_cam = msg_img->header.stamp;
    image_ = cv_ptr->image;
    
    if(!is_cam){
        img_height = image_.size().height;
        img_width = image_.size().width;
        ROS_INFO("Image recieved");
        is_cam = true;
    }
}

void ProjectionIP::callback_lidar(const sensor_msgs::PointCloud2::ConstPtr& msg_lidar){
    if(!is_lidar){
        ROS_INFO("pointcloud recieved");
        is_lidar = true;
    }
    pcl::fromROSMsg(*msg_lidar,cloud);
    time_lidar = msg_lidar->header.stamp;
}

void ProjectionIP::starter(){
    point2img(cloud, image_);
}

void ProjectionIP::point2img(const Pointcloud& cloud_, const cv::Mat& image_){
    if(!is_lidar || !is_cam) return;

    // Eigen::Matrix4f Pnew = Eigen::Matrix4f::Identity();;
    // for(int i=0;i<9;i++) Pnew(i/3,i%3) = Pr(i/3,i%3);
    // Eigen::Matrix4f final_transform = Pnew * Rect * Tr;
    Eigen::Matrix4f final_transform = Pr * Rect * Tr;
    
    Pointcloud::Ptr cloud_ptr(new Pointcloud);
    Pointcloud::Ptr cloud_tf(new Pointcloud), cloud_tf2(new Pointcloud);
    *cloud_ptr = cloud_;

    pcl::PassThrough<Point> pass_filter_x;
    pass_filter_x.setInputCloud(cloud_ptr);
    pass_filter_x.setFilterFieldName("x");
    pass_filter_x.setFilterLimits(0.0, 500.0);
    pass_filter_x.setFilterLimitsNegative(false);
    pass_filter_x.filter(*cloud_tf);
    
    pcl::transformPointCloud(*cloud_tf, *cloud_tf2, final_transform);
    
    cv::Mat img_tmp = image_.clone();
    
    for(int i=0;i<cloud_tf2->points.size();i++){
        float z=0.0;
        int x=0, y=0;
        float dist = 0;
        int r_=255, g_=0, b_=0;

        z = cloud_tf2->points[i].data[2];
        x = cloud_tf2->points[i].data[0]/z;
        
        
        if(x<0 || x>=img_width) continue;

        y = cloud_tf2->points[i].data[1]/z;
        
        if (y<0 || y>=img_height) continue;

        float xc=0,yc=0;
        xc = cloud_tf->points[i].data[0];
        yc = cloud_tf->points[i].data[1];

        img_tmp.at<cv::Vec3b>(cv::Point(x,y)) = set_pixel_color(sqrt(xc*xc+yc*yc), 5.0, 95.0);
    }
    publish_img(img_tmp);
}

cv::Vec3b ProjectionIP::set_pixel_color(float value, float k1, float k2){
    int r_=255, g_=0, b_=0;
    float dist = k1_vis + value / k2_vis;
    float color_k = (dist < 0) ? 0 : dist;
    color_k = (color_k > 1.5) ? 1.5 : color_k;
    
    if (dist<0.5){
        r_ = 255 * (1-2 * color_k);
        g_ = 255 * 2 * color_k;
        b_ = 0;
    }
    else if (dist<1.0){
        g_ = 255 * (1-2 * (color_k-0.5));
        b_ = 255 * 2 * (color_k-0.5);
        r_ = 0;
    }
    else{
        r_ = 255 * (2 * (color_k-1.0));
        b_ = 255 * (1 - 2 * (color_k-1.0));
        g_ = 0;
        
    }
    r_ = (r_ > 255) ? 255 : r_;
    r_ = (r_ < 0) ? 0 : r_;
    g_ = (g_ > 255) ? 255 : g_;
    g_ = (g_ < 0) ? 0 : g_;
    b_ = (b_ > 255) ? 255 : b_;
    b_ = (b_ < 0) ? 0 : b_;
    
    return cv::Vec3b(b_, g_, r_);
}

void ProjectionIP::publish_img(const cv::Mat& img_){
    cv_bridge::CvImage out_img;
    out_img.header.stamp = ros::Time::now();
    out_img.encoding = encoding_;
    out_img.image = img_;
    pub_img_.publish(out_img);
}
