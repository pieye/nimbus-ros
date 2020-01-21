#include "/usr/local/include/nimbusPreprocessInterface.h"

#include <iostream>
#include <thread>

#include <ros/ros.h>
#include <ros/console.h>
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

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#include <stdio.h>
#include <unistd.h>
#include <nimbusPreprocessInterface.h>
#include <random>

#include <cstdlib>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int g_cnt;
int rand();

int _x[352*286];
int _y[352*286];
int _z[352*286];


void imageCallback(void* unused0, void* img, void* unused1) {
    uint16_t* ampl = nimbus_seq_get_amplitude(img);
    int16_t* x = nimbus_seq_get_x(img);
    int16_t* y = nimbus_seq_get_y(img);
    int16_t* z = nimbus_seq_get_z(img);
    uint8_t* conf = nimbus_seq_get_confidence(img);
    ImgHeader_t* header = nimbus_seq_get_header(img);
    //printf("got an image\n");
    for(int i = 0; i < (352 * 286); i++)
        {
            _x[i] = x[i];
            _y[i] = y[i];
            _z[i] = z[i];
        }
    g_cnt++;
    nimbus_seq_del(img); //<- free the image pointer this call is necessary to return img resource to nimbus
}


int i;




int main(int argc, char** argv)
{
    ros::init(argc, argv, "nibus_ros");

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("point_cloud", 1000);
    static tf2_ros::StaticTransformBroadcaster staticTrans;
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr preCloud(new PointCloud);
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

    printf("nimbus-userland version: %s\n", nimbus_get_version());

    if (nimbus_preproc_init()) {
        nimbus_preproc_seq_cb(imageCallback);
        //g_cnt = 0;
        //while (g_cnt < 100) {
        //    usleep(10000);
        //}
    
        ros::Rate r(1);
        while (nh.ok())
        {
            PointCloud::Ptr new_cloud(new PointCloud);
            //std::vector<std::vector<float>> res = wbClient.getImage();
            if(true){
                for(int i = 0; i < (352*286); i++)
                {
                    pcl::PointXYZ basic_points;
                    basic_points.x = (float)_x[i]/1000;
                    basic_points.y = (float)_y[i]/1000;
                    basic_points.z = (float)_z[i]/1000;
                    new_cloud->points.push_back(basic_points);
                }
                cloud->points = new_cloud->points;
            }else{
                cloud = preCloud;
            }
            cameraPose.header.stamp = ros::Time::now();
            staticTrans.sendTransform(cameraPose);
            pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
            printf("publish\n");
            pub.publish(cloud);
            ros::spinOnce();
            r.sleep();
            preCloud = cloud;
        }
    }
    else {
        printf("init nimbus failed!\n");
    }
    return 0;
}
