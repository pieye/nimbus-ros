/**
 * @file main.cpp
 * @brief ROS Driver for the nimbus 3D Time of flight camera.
 *        It requires the updated nimbus-userland and the correctly 
 *        installed library (libnimbus-preprocessing.so).
 *
 * @copyright Copyright 2020 Bjarne Johannsen
 * @author Bjarne Johannsen
 *
 * @license This file is part of Nimbus.
 * 
 * Nimbus is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * Nimbus is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with Nimbus.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <nimbusPreprocessInterface.h>
#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud::Ptr nimbus_cloud(new PointCloud);
PointCloud::Ptr nimbus_cloud_filtered(new PointCloud);

sensor_msgs::Image range_image;
sensor_msgs::Image intensity_image;

const int img_width = 352;
const int img_height = 286;
const int XYZ_to_m = 5000;  //<-- to be determined. Correct value missings

//Callback to get measurement data directly from nimbus
void imageCallback(void* unused0, void* img, void* unused1) {
    uint16_t* ampl = nimbus_seq_get_amplitude(img);
    int16_t* x = nimbus_seq_get_x(img);
    int16_t* y = nimbus_seq_get_y(img);
    int16_t* z = nimbus_seq_get_z(img);
    uint8_t* conf = nimbus_seq_get_confidence(img);
    ImgHeader_t* header = nimbus_seq_get_header(img);

    //Move valid points into the point cloud and the corresponding images
    for(int i = 0; i < (img_width*img_height); i++)
        {
            if(conf[i] == 0){
                nimbus_cloud->points[i].x         = (float)x[i]/XYZ_to_m;
                nimbus_cloud->points[i].y         = (float)y[i]/XYZ_to_m;
                nimbus_cloud->points[i].z         = (float)z[i]/XYZ_to_m;
                nimbus_cloud->points[i].intensity = ampl[i];
                range_image.data[i]               = std::min(std::max(z[i]/200, 0), 255);
                intensity_image.data[i]           = std::min(std::max(50+ampl[i]/40, 0), 255);
            }
            else{
                nimbus_cloud->points[i].x         =  NAN;
                nimbus_cloud->points[i].y         =  NAN;
                nimbus_cloud->points[i].z         =  NAN;
                nimbus_cloud->points[i].intensity =  NAN;
                range_image.data[i]               = 0;
                intensity_image.data[i]           = std::min(std::max(50+ampl[i]/40, 0), 255);
            }
        }
    nimbus_seq_del(img); //<- free the image pointer this call is necessary to return img resource to nimbus
}

int main(int argc, char** argv)
{
    bool downsampling = false;
    bool pub_intes_image = false;
    bool pub_range_image = false;
    float downsampling_voxel_size = 0.05;
    int frame_rate = 10;

    ros::init(argc, argv, "nibus_ros");
    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub = nh.advertise<PointCloud>("pointcloud", 1);
    ros::Publisher range_image_pub = nh.advertise<sensor_msgs::Image>("range_image", 1);
    ros::Publisher intensity_image_pub = nh.advertise<sensor_msgs::Image>("intensity_image", 1);

    //Define point cloud
    nimbus_cloud->points.resize(img_width*img_height);
    nimbus_cloud->width = img_width;
    nimbus_cloud->height = img_height;
    nimbus_cloud->is_dense = false;        //<-- because invalid points have been set to nan
    nimbus_cloud->header.frame_id = "nimbus";

    //Define projected range image
    range_image.width = img_width;
    range_image.height = img_height;
    range_image.encoding = "mono8";
    range_image.data.resize(img_width*img_height);
    
    //Define projected intensity image
    intensity_image.width = img_width;
    intensity_image.height = img_height;
    intensity_image.encoding = "mono8";
    intensity_image.data.resize(img_width*img_height);

    ROS_INFO_STREAM("Nimbus-userland version: " << nimbus_get_version());

    if (nimbus_preproc_init()) {
        nimbus_preproc_seq_cb(imageCallback);
    
        ros::param::get("/nimbus_ros_node/frame_rate", frame_rate);

        ros::Rate r(frame_rate);
        while (nh.ok())
        {
            //get rosparam dyncamically
            ros::param::get("/nimbus_ros_node/intensity_image", pub_intes_image);
            ros::param::get("/nimbus_ros_node/range_image", pub_range_image);
            ros::param::get("/nimbus_ros_node/downsampling", downsampling);
            ros::param::get("/nimbus_ros_node/downsampling_voxel_size", downsampling_voxel_size);

            pcl_conversions::toPCL(ros::Time::now(), nimbus_cloud->header.stamp);

            //Downsampling of Pointcloud if needed
            if(downsampling == false)
                pointcloud_pub.publish(nimbus_cloud);
            else{
                pcl::VoxelGrid<pcl::PointXYZI> sor;
                sor.setInputCloud (nimbus_cloud);
                sor.setLeafSize (downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);
                sor.filter (*nimbus_cloud_filtered);
                pointcloud_pub.publish(nimbus_cloud_filtered);
            }

            //only publish images if required
            if(pub_range_image == true)
                range_image_pub.publish(range_image);
            if(pub_intes_image == true)
                intensity_image_pub.publish(intensity_image);

            ros::spinOnce();
            r.sleep();
        }
    }
    else {
        ROS_FATAL_STREAM("Nimbus init failed!");
    }
    return 0;
}
