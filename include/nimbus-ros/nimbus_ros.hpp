/**
 * @file nimbus_ros.hpp
 * @brief ROS <-> Nimbus Interface 
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
#include "nimbus-ros/Algorithm.hpp"
#include <algorithm>
#include <unistd.h>
#include <std_msgs/Float32.h>

/**************************** defines  ******************************/
#define CRITICAL_TEMP 70.0

/************************* global variables *************************/
/**
 * ROS Images to represent intensity and range values
 */
extern bool m_new_image;
extern bool m_auto_exposure_update;

/**
 * Pointcloud and ROS Images to represent intensity and range values.
 */
extern PointCloud::Ptr m_nimbus_cloud;
extern sensor_msgs::Image m_range_image;
extern sensor_msgs::Image m_intensity_image;
extern std_msgs::Float32  m_temp;
extern std_msgs::Float32  m_exposure;
extern std_msgs::Float32  m_dist;

/**
 * Auto Exposure Parameter to 
 */
extern AutoExposureParams_t m_params;

/**
 * Dynamic parameter set by the parameter server.
 */
extern float m_XYZ_to_m;
extern int   frame_rate;
extern bool  downsampling;
extern bool  pub_pointcloud;
extern bool  pub_intes_image;
extern bool  pub_m_range_image;
extern float downsampling_voxel_size;
extern float ampl_single;
extern int   max_exposure;
extern float hdr_factor;
extern float ampl_hdr;
extern bool  hdr_mode;


/********************** function declarations ***********************/

/**
 * Callback which gets the new Pointcloud when it's available
 * and writes the values into the PCL Pointcloud.
 */
void imageCallback(void* unused0, void* img, void* unused1);

/**
 * Poll new parameter from the ROS paramter server 
 */
void update_params();