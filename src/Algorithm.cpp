/**
 * @file Algorithm.cpp
 * @brief Basic Algorithms required by the Nimbus 3D ROS driver.
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
#include "nimbus-ros/Algorithm.hpp"


/********************** function declarations ***********************/
RgbColor HsvToRgb(unsigned char h, unsigned char s, unsigned char v)
{
    RgbColor rgb;
    unsigned char region, remainder, p, q, t;

    if (s == 0)
    {
        rgb.r = v;
        rgb.g = v;
        rgb.b = v;
        return rgb;
    }

    region = h / 43;
    remainder = (h - (region * 43)) * 6; 

    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0:
            rgb.r = v; rgb.g = t; rgb.b = p;
            break;
        case 1:
            rgb.r = q; rgb.g = v; rgb.b = p;
            break;
        case 2:
            rgb.r = p; rgb.g = v; rgb.b = t;
            break;
        case 3:
            rgb.r = p; rgb.g = q; rgb.b = v;
            break;
        case 4:
            rgb.r = t; rgb.g = p; rgb.b = v;
            break;
        default:
            rgb.r = v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}

void cloud_to_image(PointCloud::Ptr cloud_ptr, sensor_msgs::Image &temp_intens, sensor_msgs::Image &temp_range){
    for(int i = 0; i < (IMG_WIDTH*IMG_HEIGHT); i++){
        if(cloud_ptr->points[i].intensity > 0){
            RgbColor rgb_range = HsvToRgb(std::min(std::max((cloud_ptr->points[i].z*50), 0.0f), 255.0f),255,255);
            temp_range.data[i*COLOR_CHANELS]          = rgb_range.r;
            temp_range.data[i*COLOR_CHANELS+1]        = rgb_range.g;
            temp_range.data[i*COLOR_CHANELS+2]        = rgb_range.b;
            temp_intens.data[i]                       = std::min(std::max((log(cloud_ptr->points[i].intensity)*30), 0.0f), 255.0f);
        }
        else{
            temp_range.data[i*COLOR_CHANELS]          = 0;
            temp_range.data[i*COLOR_CHANELS+1]        = 0;
            temp_range.data[i*COLOR_CHANELS+2]        = 0;
            temp_intens.data[i]                       = std::min(std::max((log(cloud_ptr->points[i].intensity)*30), 0.0f), 255.0f);
        }
    }
}