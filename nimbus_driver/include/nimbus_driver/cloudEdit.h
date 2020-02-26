#ifndef CLOUDEDIT_H
#define CLOUDEDIT_H

#include <pcl/io/impl/synchronized_queue.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

template <class T>
class cloudEdit
{
private:
    typedef pcl::PointCloud<pcl::PointXYZI> Point_Cloud;
public:
    cloudEdit(/* args */);
    ~cloudEdit();

    void meanFilter(pcl::PointCloud<pcl::PointXYZI> &res, int width, int height);
    // Variables
    pcl::SynchronizedQueue<T> cloudQueue;

};

#endif