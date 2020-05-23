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

/**************************** includes ******************************/
#include <nimbusPreprocessInterface.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "nimbus_3d_driver/nimbus_3d_driver.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nibus_ros");
    ros::NodeHandle nh;

    ros::Publisher pointcloud_pub       = nh.advertise<PointCloud>("pointcloud", 1);
    ros::Publisher range_image_pub      = nh.advertise<sensor_msgs::Image>("range_image", 1);
    ros::Publisher intensity_image_pub  = nh.advertise<sensor_msgs::Image>("intensity_image", 1);
    ros::Publisher info_temp_pub        = nh.advertise<std_msgs::Float32>("info/temperature", 1);
    ros::Publisher info_exposure_pub    = nh.advertise<std_msgs::Float32>("info/exposure", 1);

    //Initialize Pointcloud and Images
    m_nimbus_cloud->points.resize(IMG_WIDTH*IMG_HEIGHT);
    m_nimbus_cloud->width = IMG_WIDTH;
    m_nimbus_cloud->height = IMG_HEIGHT;
    m_nimbus_cloud->is_dense = false;        //<-- because invalid points are being set to NAN
    m_nimbus_cloud->header.frame_id = "nimbus";
    m_range_image.width = IMG_WIDTH;
    m_range_image.height = IMG_HEIGHT;
    m_range_image.encoding = "rgb8";
    m_range_image.data.resize(IMG_WIDTH*IMG_HEIGHT*COLOR_CHANELS);
    m_intensity_image.width = IMG_WIDTH;
    m_intensity_image.height = IMG_HEIGHT;
    m_intensity_image.encoding = "mono8";
    m_intensity_image.data.resize(IMG_WIDTH*IMG_HEIGHT);

    ROS_INFO_STREAM("Nimbus-userland version: " << nimbus_get_version());

    if (nimbus_preproc_init()) {

        nimbus_preproc_seq_cb(imageCallback);

        ros::param::get("/nimbus_3d_driver_node/frame_rate", frame_rate);
        ros::Rate r(frame_rate);
        while (nh.ok())
        {
            if(m_new_image == true){
                //get rosparam dyncamically
                update_params();

                //only publish pointcloud if required
                if(pub_pointcloud == true){
                    //Downsampling of Pointcloud if needed
                    if(downsampling == false)
                        pointcloud_pub.publish(m_nimbus_cloud);
                    else{
                        pcl::VoxelGrid<pcl::PointXYZI> sor;
                        sor.setInputCloud (m_nimbus_cloud);
                        sor.setLeafSize (downsampling_voxel_size, downsampling_voxel_size, downsampling_voxel_size);
                        PointCloud::Ptr m_nimbus_cloud_filtered(new PointCloud);
                        sor.filter (*m_nimbus_cloud_filtered);
                        pointcloud_pub.publish(m_nimbus_cloud_filtered);
                    }
                }
                
                //only publish images if required
                cloud_to_image(m_nimbus_cloud,m_intensity_image,m_range_image);
                if(pub_m_range_image == true)
                    range_image_pub.publish(m_range_image);
                if(pub_intes_image == true)
                    intensity_image_pub.publish(m_intensity_image);

                m_new_image = false;

                info_temp_pub.publish(m_temp);
                info_exposure_pub.publish(m_exposure);

                if(m_temp.data > CRITICAL_TEMP)
                    ROS_WARN_STREAM("Critical temperature of Nimbus: " << m_temp.data << "°C");
                if(m_temp.data == NAN)
                    ROS_FATAL_STREAM("I²C error, check raspi-config!");
                ros::spinOnce();
            }
            r.sleep();
        }
    }
    else {
        ROS_FATAL_STREAM("Nimbus init failed!");
    }
    return 0;
}