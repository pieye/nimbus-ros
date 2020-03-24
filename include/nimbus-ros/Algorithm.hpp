/**
 * @file Algorithm.hpp
 * @brief Basic Algorithms required by the Nimbus 3D ROS driver.

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
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Image.h>


/**************************** defines ******************************/
#define IMG_WIDTH 352
#define IMG_HEIGHT 286
#define COLOR_CHANELS 3

/**************************** typedefs ******************************/

/**
 * Define a PCL XYZI Pointcloud
 */
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

/**
 * Define RgbColor struct to be used int the HsvToRgb
 */
typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;


/********************** function declarations ***********************/

/**
 * Calculates RGB to given HSV values. 
 */
RgbColor HsvToRgb(unsigned char h, unsigned char s, unsigned char v);

/**
 * Takes a XYZI pointcloud and generates HSV range and intensity images.
 */
void cloud_to_image(PointCloud::Ptr cloud_ptr, sensor_msgs::Image &temp_intens, sensor_msgs::Image &temp_range);