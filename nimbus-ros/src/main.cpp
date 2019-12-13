#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <nimbus_ros_driver/websocket.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nibus_ros");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("point_cloud", 1000);
    static tf2_ros::StaticTransformBroadcaster staticTrans;

    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr preCloud(new PointCloud);
    
    nimbus::WebSocketClient wbClient((unsigned char *)"http://192.168.0.69:8383/jsonrpc", false, 8080, 8383, 3, 5, 3, 10, nh);

    cloud->width = 352 * 286;
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->header.frame_id = "camera";

    geometry_msgs::TransformStamped cameraPose;
    cameraPose.child_frame_id = "camera";
    cameraPose.header.frame_id = "world";
    cameraPose.transform.translation.x = 0;
    cameraPose.transform.translation.y = 0;
    cameraPose.transform.translation.z = 1;
    cameraPose.transform.rotation.x = 0.7071068;
    cameraPose.transform.rotation.y = 0;
    cameraPose.transform.rotation.z = 0;
    cameraPose.transform.rotation.w = 0.7071068;
    //geometry_msgs::Transform p(geometry_msgs::Vector3(0,0,0), geometry_msgs::Quaternion(0,0,0,1));
    
    ros::Rate r(20);
    while (nh.ok())
    {
        PointCloud::Ptr new_cloud(new PointCloud);
        std::vector<std::vector<float>> res = wbClient.getImage();
        if(!res.empty()){
            for(int i = 0; i < res[0].size(); i++)
            {
                pcl::PointXYZ basic_points;
                basic_points.x = res[0][i];
                basic_points.y = res[1][i];
                basic_points.z = res[2][i];
                new_cloud->points.push_back(basic_points);
            }
            cloud->points = new_cloud->points;
        }else{
            cloud = preCloud;
        }
        cameraPose.header.stamp = ros::Time::now();
        staticTrans.sendTransform(cameraPose);
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        pub.publish(cloud);
        ros::spinOnce();
        r.sleep();
        preCloud = cloud;
    }
    return 0;
}