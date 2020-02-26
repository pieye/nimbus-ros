#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

void callback(const PointCloud::ConstPtr& msg){
    printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points) 
                    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_driver_io_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::spin();
}