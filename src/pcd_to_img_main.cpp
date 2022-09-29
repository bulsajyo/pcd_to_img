#include <pcd_to_img/projection_IP.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "projection_ip");
    ros::NodeHandle nh;
    ProjectionIP projectionIP(&nh);
    ros::Rate loop_rate(15);
    while (ros::ok()){
        ros::spinOnce();
        projectionIP.starter();
        loop_rate.sleep();
    }
    return 0;
}
