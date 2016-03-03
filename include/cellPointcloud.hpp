#include <ros/ros.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/transforms.h>


class CellPointcloud{
public:
    CellPointcloud(int argc, char** argv);
    virtual ~CellPointcloud();
    bool init();
    void run();


    void cloudCallback0(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cloudCallback1(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void cloudCallback2(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
    ros::Publisher pubPointCloud;
    ros::Subscriber subPointCloud0;
    ros::Subscriber subPointCloud1;
    ros::Subscriber subPointCloud2;

    bool newCloud0;
    bool newCloud1;
    bool newCloud2;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMerged;

    std::vector<Eigen::Matrix4f> cameraPositions;

    int init_argc;
    char** init_argv;

};
