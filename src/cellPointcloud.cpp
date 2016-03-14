#include "../include/cellPointcloud.hpp"
#include <iostream>



CellPointcloud::CellPointcloud(int argc, char **argv):
    init_argc(argc),
    init_argv(argv)
{}

CellPointcloud::~CellPointcloud()
{
    if(ros::isStarted()) {
      ros::shutdown();
      ros::waitForShutdown();
    }
}

bool CellPointcloud::init()
{
    std::cout << "Initing" << std::endl;
    newCloud0 = false;
    newCloud1 = false;
    newCloud2 = false;
    Eigen::Matrix4f cam0 = Eigen::Matrix4f::Identity();
    cameraPositions.push_back(cam0);
    Eigen::Matrix4f cam1 = Eigen::Matrix4f::Identity();
    cam1 << 0.0323759, -0.663456,  0.747553,   -1.7822,
            0.710676,  0.541203,  0.449541, -0.749686,
           -0.702801,  0.516698,  0.489009,  0.612935,
                   0,         0,         0,         1;
    cameraPositions.push_back(cam1);
    Eigen::Matrix4f cam2 = Eigen::Matrix4f::Identity();
    cam2 << 0.0353684,  0.469686, -0.882159,   1.73419,
            -0.702388,  0.639615,  0.312387, -0.672272,
             0.710946,   0.60855,  0.352513,   0.81631,
                    0,         0,         0,         1;
    cameraPositions.push_back(cam2);

    ros::init(init_argc,init_argv,"cellPointCloud");
    if ( ! ros::master::check() ) {
        return false;
    }

pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZRGB>);
cloud0 = tmp1;
cloud1 = tmp2;
cloud2 = tmp3;

    ros::start();

    ros::NodeHandle n;
    subPointCloud1 = n.subscribe<sensor_msgs::PointCloud2, CellPointcloud>("/NUC1/sd/points", 1, &CellPointcloud::cloudCallback1,this);
    subPointCloud2 = n.subscribe<sensor_msgs::PointCloud2, CellPointcloud>("/NUC2/sd/points", 1, &CellPointcloud::cloudCallback2, this);
    subPointCloud0 = n.subscribe<sensor_msgs::PointCloud2, CellPointcloud>("/PC/sd/points", 1, &CellPointcloud::cloudCallback0, this);
    pubPointCloud = n.advertise<sensor_msgs::PointCloud2>("PointCloudMerged", 500);

    return true;
}

void CellPointcloud::run()
{
    std::cout << "Running" << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudMerged = tmp;
    ros::Rate loop_rate(100);
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    
    while ( ros::ok() ) {
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"NUC2_ir_optical_frame", "PointCloudMerged_link"));

        if(newCloud0 && newCloud1 && newCloud2){
            *cloudMerged = *cloud2;
	    *cloudMerged += *cloud1;
            *cloudMerged += *cloud0;
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*cloudMerged, msg);
            pubPointCloud.publish(msg);
            newCloud0 = false;
            newCloud1 = false;
            newCloud2 = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void CellPointcloud::cloudCallback0(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //std::cout << cameraPositions.at(0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *tmp);
    //cloud0 = tmp;
    *cloud0 = *tmp;
    pcl::transformPointCloud(*tmp, *cloud0, cameraPositions.at(2));
    newCloud0 = true;
}

void CellPointcloud::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //std::cout << cameraPositions.at(1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *tmp);
    //cloud1 = tmp;
    *cloud1 = *tmp;    
    pcl::transformPointCloud(*tmp, *cloud1, cameraPositions.at(0));
    newCloud1 = true;
}

void CellPointcloud::cloudCallback2(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //std::cout << cameraPositions.at(2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *tmp);
    //cloud2 = tmp;
    *cloud2 = *tmp;
    pcl::transformPointCloud(*tmp, *cloud2, cameraPositions.at(1));
    newCloud2 = true;

}

int main(int argc, char **argv){
    CellPointcloud cell(argc, argv);
    cell.init();
    cell.run();
}




