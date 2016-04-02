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
    // This is the matrix from NUC2 to table
//    cam1 << 0.0078076 ,-0.659096 , 0.752018  ,-1.76514,
//            0.71265 , 0.531222 , 0.458191, -0.757404,
//          -0.701487 , 0.532347 , 0.473851,  0.634337,
//                  0 ,        0     ,    0  ,      1;

    cam1 << -0.0369295,   -0.99916 , 0.0177825,   0.174928,
            -0.592998, 0.00758757 , -0.805168  , 0.016518,
             0.804357, -0.0402794 ,  -0.59278  , 0.945164,
                    0,          0 ,         0   ,       1;
    cameraPositions.push_back(cam1);


    Eigen::Matrix4f cam2 = Eigen::Matrix4f::Identity();
    // This is the matrix from PC to table
//    cam2 << 0.00652725 , 0.488565 ,-0.872509  , 1.71032,
//            -0.705839 , 0.620315 ,  0.34207 ,-0.721961,
//              0.70836 , 0.613619 , 0.348893 , 0.808552,
//                    0  ,       0  ,       0  ,       1;
    cam2 <<   -0.999718 , -0.0224201 ,-0.00776418 ,   0.604796,
              -0.00404688 ,   0.483571  , -0.875296 ,  -0.256756,
                0.0233787 ,  -0.875018  , -0.483525  ,   2.14104,
                        0 ,          0    ,       0     ,      1;

    cameraPositions.push_back(cam2);



    // This is the matrix from NUC1 to table
    Eigen::Matrix4f cam3 = Eigen::Matrix4f::Identity();
//    cam3 <<    0.99921, 0.00555718 , 0.0393607  , 0.019021,
//               0.0316395  ,-0.710612 , -0.702872  ,  0.14354,
//               0.0240642 ,  0.703562 , -0.710226  ,  1.12045,
//                       0  ,        0 ,         0   ,       1;
    cam3 << 0.0139678,  0.998862, 0.0456047, -0.299209,
            0.707306, 0.0223681, -0.706554, -0.578648,
            -0.70677, 0.0421254, -0.706188,   1.80034,
                   0 ,        0,         0,         1;

    cameraPositions.push_back(cam3);

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
    // PC, CHANGES
    Eigen::Matrix4f tmpMat2 = cameraPositions.at(2);
    Eigen::Matrix4f tmpMat3 = cameraPositions.at(3);
    Eigen::Matrix4f final = tmpMat3*tmpMat2.inverse();

    pcl::transformPointCloud(*tmp, *cloud0, final);
    //pcl::transformPointCloud(*tmp, *cloud0, cameraPositions.at(2));
    newCloud0 = true;
}

void CellPointcloud::cloudCallback1(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    //std::cout << cameraPositions.at(1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*cloud_msg, *tmp);
    //cloud1 = tmp;
    *cloud1 = *tmp;
    // NUC1, NOT CHANGED
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
    // NUC2, CHANGES
    Eigen::Matrix4f tmpMat2 = cameraPositions.at(1);
    Eigen::Matrix4f tmpMat3 = cameraPositions.at(3);
    Eigen::Matrix4f final = tmpMat3*tmpMat2.inverse();
    pcl::transformPointCloud(*tmp, *cloud2, final);
    //pcl::transformPointCloud(*tmp, *cloud2, cameraPositions.at(1));
    newCloud2 = true;

}

int main(int argc, char **argv){
    CellPointcloud cell(argc, argv);
    cell.init();
    cell.run();
}




