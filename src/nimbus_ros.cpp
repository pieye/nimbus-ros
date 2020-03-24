/**
 * @file nimbus_ros.cpp
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
#include "nimbus-ros/nimbus_ros.hpp"


/************************* global variables *************************/
bool m_new_image = false;
bool m_auto_exposure_update = false;

PointCloud::Ptr m_nimbus_cloud(new PointCloud);
sensor_msgs::Image m_range_image;
sensor_msgs::Image m_intensity_image;
std_msgs::Float32  m_temp;
std_msgs::Float32  m_exposure;
std_msgs::Float32  m_dist;

AutoExposureParams_t m_params;

float m_XYZ_to_m              = 0.00009587526; 
int frame_rate                = 10;
bool downsampling             = false;
bool pub_pointcloud           = false;
bool pub_intes_image          = false;
bool pub_m_range_image        = false;
float downsampling_voxel_size = 0.05;
float ampl_single;
int   max_exposure;
float hdr_factor;
float ampl_hdr;
int   exposure_mode;


/********************** function declarations ***********************/

//Callback to get measurement data directly from nimbus
void imageCallback(void* unused0, void* img, void* unused1) {
    if(img != nullptr && ros::ok()){
        ImgHeader_t* header = nimbus_seq_get_header(img);
        m_temp.data         = header->temperature/10;
        m_exposure.data     = header->exposure*0.00001241;
        uint16_t* ampl      = nimbus_seq_get_amplitude(img);
        int16_t* x          = nimbus_seq_get_x(img);
        int16_t* y          = nimbus_seq_get_y(img);
        int16_t* z          = nimbus_seq_get_z(img);
        uint8_t* conf       = nimbus_seq_get_confidence(img);
        m_dist.data         = z[IMG_WIDTH*(IMG_HEIGHT/2) + IMG_WIDTH/2]*m_XYZ_to_m;

        //Move valid points into the point cloud and the corresponding images
        for(int i = 0; i < (IMG_WIDTH*IMG_HEIGHT); i++)
            {
                if(conf[i] == 0){
                    //cast x,y,z to float and multiply by m_XYZ_to_m
                    int16x4_t xyz_vec = {x[i], y[i], z[i], z[i]};
                    float32x4_t result = vmulq_n_f32(vcvtq_f32_s32(vmovl_s16(xyz_vec)), m_XYZ_to_m);
                    m_nimbus_cloud->points[i].x         = result[0];
                    m_nimbus_cloud->points[i].y         = result[1];
                    m_nimbus_cloud->points[i].z         = result[2];
                    m_nimbus_cloud->points[i].intensity = ampl[i];
                }
                else{
                    m_nimbus_cloud->points[i].x         = NAN;
                    m_nimbus_cloud->points[i].y         = NAN;
                    m_nimbus_cloud->points[i].z         = NAN;
                    m_nimbus_cloud->points[i].intensity = NAN;
                }
            }

        if(m_auto_exposure_update){
            bool set_exposure = nimbus_autoexposure_set_params(&m_params);
            m_auto_exposure_update = false;
        }

        nimbus_seq_del(img); //<- free the image pointer this call is necessary to return img resource to nimbus
        m_new_image = true;
    }
    else{
        ROS_FATAL_STREAM("Callback failed!");
    }
}

void update_params(){
    ros::param::get("/nimbus_ros_node/pointcloud", pub_pointcloud);
    ros::param::get("/nimbus_ros_node/intensity_image", pub_intes_image);
    ros::param::get("/nimbus_ros_node/range_image", pub_m_range_image);
    ros::param::get("/nimbus_ros_node/XYZ_to_m", m_XYZ_to_m);
    ros::param::get("/nimbus_ros_node/downsampling", downsampling);
    ros::param::get("/nimbus_ros_node/downsampling_voxel_size", downsampling_voxel_size);
    ros::param::get("/nimbus_ros_node/ampl_single", ampl_single);
    ros::param::get("/nimbus_ros_node/max_exposure", max_exposure);
    ros::param::get("/nimbus_ros_node/hdr_factor", hdr_factor);
    ros::param::get("/nimbus_ros_node/ampl_hdr", ampl_hdr);
    ros::param::get("/nimbus_ros_node/exposure_mode", exposure_mode);

    //update exposure parameters if they have changed
    if((float(ampl_single) != m_params.ampl_single) || (int(max_exposure) != m_params.max_exposure) ||  
        (float(hdr_factor) != m_params.hdr_factor) || (float(ampl_hdr) != m_params.ampl_hdr) ||
        (int(exposure_mode) != m_params.exposure_mode)){
            m_auto_exposure_update = true;
            m_params.ampl_single   = ampl_single; 
            m_params.max_exposure  = max_exposure;
            m_params.hdr_factor    = hdr_factor;
            m_params.ampl_hdr      = ampl_hdr;
            m_params.exposure_mode = exposure_mode;
    }
}