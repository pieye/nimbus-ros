#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());

int counter = 0;

void callback(const PointCloud::ConstPtr& msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    cloud->header = msg->header;
    cloud->height = msg->height;
    cloud->width = msg->width;
    cloud->is_dense = msg->is_dense;
    cloud->points = msg->points;
    counter += 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/kinect2/sd/points", 1, callback);

  while(ros::ok() && counter < 500){
      printf ("Counter: %d\n",counter);
      ros::spinOnce();          // wait for a while
  }
  pcl::io::savePCDFile("cloud.pcd", *cloud);
  ros::shutdown();
}