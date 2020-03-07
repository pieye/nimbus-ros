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

#include <arm_neon.h>
#include <nimbusPreprocessInterface.h>
#include <unistd.h>
#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Image.h>
#include "nimbus-ros/RosPackageTemplate.hpp"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "nibus_ros");
    ros::NodeHandle nh;

    ros::Publisher pointcloud_pub = nh.advertise<PointCloud>("pointcloud", 1);
    ros::Publisher range_image_pub = nh.advertise<sensor_msgs::Image>("range_image", 1);
    ros::Publisher intensity_image_pub = nh.advertise<sensor_msgs::Image>("intensity_image", 1);

    //Initialize Pointcloud and Images
    m_nimbus_cloud->points.resize(m_img_width*m_img_height);
    m_nimbus_cloud->width = m_img_width;
    m_nimbus_cloud->height = m_img_height;
    m_nimbus_cloud->is_dense = false;        //<-- because invalid points are being set to NAN
    m_nimbus_cloud->header.frame_id = "nimbus";
    m_range_image.width = m_img_width;
    m_range_image.height = m_img_height;
    m_range_image.encoding = "rgb8";
    m_range_image.data.resize(m_img_width*m_img_height*3);
    m_intensity_image.width = m_img_width;
    m_intensity_image.height = m_img_height;
    m_intensity_image.encoding = "rgb8";
    m_intensity_image.data.resize(m_img_width*m_img_height*3);


    ROS_INFO_STREAM("Nimbus-userland version: " << nimbus_get_version());

    if (nimbus_preproc_init()) {
        //nimbus_preproc_set_user_data((void*)nh);
        int frame_rate = 10;
        bool downsampling = false;
        bool pub_intes_image = false;
        bool pub_m_range_image = false;
        float downsampling_voxel_size = 0.05;
        float ampl_single;
        int   max_exposure;
        float hdr_factor;
        float ampl_hdr;
        bool  hdr_mode;

        nimbus_preproc_seq_cb(imageCallback);

        ros::param::get("/nimbus_ros_node/frame_rate", frame_rate);
        ros::Rate r(frame_rate);
        while (nh.ok())
        {
            if(m_new_image == true){
                //get rosparam dyncamically
                ros::param::get("/nimbus_ros_node/intensity_image", pub_intes_image);
                ros::param::get("/nimbus_ros_node/range_image", pub_m_range_image);
                ros::param::get("/nimbus_ros_node/downsampling", downsampling);
                ros::param::get("/nimbus_ros_node/downsampling_voxel_size", downsampling_voxel_size);
                ros::param::get("/nimbus_ros_node/ampl_single", ampl_single);
                ros::param::get("/nimbus_ros_node/max_exposure", max_exposure);
                ros::param::get("/nimbus_ros_node/hdr_factor", hdr_factor);
                ros::param::get("/nimbus_ros_node/ampl_hdr", ampl_hdr);
                ros::param::get("/nimbus_ros_node/hdr_mode", hdr_mode);

                //update exposure parameters if they have changed
                if((float(ampl_single) != m_params.ampl_single) || (int(max_exposure) != m_params.max_exposure) ||  (float(hdr_factor) != m_params.hdr_factor)
                        || (float(ampl_hdr) != m_params.ampl_hdr) || (bool(hdr_mode) != m_params.hdr_mode)){
                    m_auto_exposure_update = true;
                    m_params.ampl_single   = ampl_single; 
                    m_params.max_exposure  = max_exposure;
                    m_params.hdr_factor    = hdr_factor;
                    m_params.ampl_hdr      = ampl_hdr;
                    m_params.hdr_mode      = hdr_mode;
                }
                    
                pcl_conversions::toPCL(ros::Time::now(), m_nimbus_cloud->header.stamp);

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
                    pointcloud_pub.publish(m_nimbus_cloud);

                }
                
                //only publish images if required
                cloud_to_image();
                if(pub_m_range_image == true)
                    range_image_pub.publish(m_range_image);
                if(pub_intes_image == true)
                    intensity_image_pub.publish(m_intensity_image);

                m_new_image = false;
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