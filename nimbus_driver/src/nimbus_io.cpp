#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <nimbus_driver/cloudEdit.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud cloud_blob;
bool newCloud = false;
void callback(const PointCloud::ConstPtr& msg){
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    pcl::PointXYZI points;
    // BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points){
    //     points.x = tempC.point[i]x;
    //     points.y = tempC.point[i]y;
    //     points.z = tempC.point[i]z;
    //     points.intensity = tempC.point[i]intensity;
    // }
    cloud_blob.width = msg->width;
    cloud_blob.height = msg->height;
    cloud_blob.header = msg->header;
    cloud_blob.points = msg->points;
    newCloud = true;
}

template <class T>
cloudEdit<T>::cloudEdit(){}
template <class T>
cloudEdit<T>::~cloudEdit(){}

template <class T>
void cloudEdit<T>::meanFilter(pcl::PointCloud<pcl::PointXYZI> &res, int width, int height){
    if(cloudQueue.isEmpty()) return;
    std::vector<float> conf(width*height, 0);
    std::vector<float> mnCounter(width*height, 0);
    std::vector<float> addX(width*height, 0);
    std::vector<float> addY(width*height, 0);
    std::vector<float> addZ(width*height, 0);
    std::vector<float> ampt(width*height, 0);

    //Point_Cloud sum;
    Point_Cloud tempC;
    res.width = width;
    res.height = height;
    while (!cloudQueue.isEmpty()){
        cloudQueue.dequeue(tempC);
        int i = 0;
        for(int i = 0; i < tempC.points.size(); i++){
            if(!std::isnan(tempC.points[i].x)){
                addX[i] += tempC.points[i].x;
                addY[i] += tempC.points[i].y;
                addZ[i] += tempC.points[i].z;
                ampt[i] += tempC.points[i].intensity;
                mnCounter[i] +=1;
                // ROS_INFO("X: %f, Y: %f, Z:%f AMP: %f, Mean Counter: %d", addX[i], addY[i], addZ[i], ampt[i], mnCounter[i]);
            }else{
                conf[i] = 1;
                mnCounter[i] = 0;
            }
        }
    }
    for(int i = 0; i < mnCounter.size(); i++){
        pcl::PointXYZI temPoint;
        if(mnCounter[i] <= 2){
            temPoint.x = NAN;
            temPoint.y = NAN;
            temPoint.z = NAN;
            temPoint.intensity = NAN;
        }else{
            temPoint.x = addX[i] / mnCounter[i];
            temPoint.y = addY[i] / mnCounter[i];
            temPoint.z = addZ[i] / mnCounter[i];
            temPoint.intensity = ampt[i] / mnCounter[i];
        }
        res.points.push_back(temPoint);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "nimbus_driver_io_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/nimbus/pointcloud", 10, callback);
    ros::Publisher pub = nh.advertise<PointCloud>("pointcloud", 5);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    cloudEdit<PointCloud> cE;

    geometry_msgs::TransformStamped cameraPose;
    cameraPose.child_frame_id = "Mcamera";
    cameraPose.header.frame_id = "world";
    cameraPose.transform.translation.x = 0;
    cameraPose.transform.translation.y = 0;
    cameraPose.transform.translation.z = 1;
    cameraPose.transform.rotation.x = -0.7071068;
    cameraPose.transform.rotation.y = 0;
    cameraPose.transform.rotation.z = 0;
    cameraPose.transform.rotation.w = 0.7071068;
    
    while (ros::ok())
    {
        if(newCloud){
            if(cE.cloudQueue.size() < 10){
                cE.cloudQueue.enqueue(cloud_blob);
                newCloud = false;
            }
            else{
                PointCloud::Ptr cloud(new PointCloud);
                cE.meanFilter (*cloud, cloud_blob.width, cloud_blob.height);
                cloud->header.frame_id= "Mcamera";
                pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
                pub.publish(cloud);
                cloud.reset();
            }
            // cloud_blob.points.clear()

        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        ros::spinOnce();
    }
    
    ros::spin();
}