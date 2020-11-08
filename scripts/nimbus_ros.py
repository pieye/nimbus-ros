#!/usr/bin/env python

# @file nimbus_ros.py
# @brief Basic Nimbus 3D ROS driver for remote remote access.
#
# @copyright Copyright 2020 Bjarne Johannsen
# @author Bjarne Johannsen
#
# @license This file is part of Nimbus.
# 
# Nimbus is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# Nimbus is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with Nimbus.  If not, see <http://www.gnu.org/licenses/>.

#Python
import os
import rospy
import time
import struct
import numpy as np

#ROS
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage

#CV
import cv2

#Nimbus
from nimbusPython import NimbusClient

def cloud_to_intensity_image(ampl):
    log_intensiy = np.minimum(np.maximum(np.log(ampl.flatten())*30, np.full(np.size(ampl.flatten()), 0)), 255)
    intensity_image = np.reshape(log_intensiy, (-1, 352))
    return intensity_image

def talker():
    rospy.init_node('nimbus_ros', anonymous=True)
    rospy.loginfo("Initializing nimbus-ros publisher node...")

    pcl_pub = rospy.Publisher("/nimbus/pointcloud", PointCloud2, queue_size=10)
    image_pub = rospy.Publisher("~image_raw/compressed", CompressedImage, queue_size=10)

    fields =    [PointField('x', 0, PointField.FLOAT32, 1),
                 PointField('y', 4, PointField.FLOAT32, 1),
                 PointField('z', 8, PointField.FLOAT32, 1),
                 PointField('intensity', 12, PointField.FLOAT32, 1),
                ]

    #connect to nimbus-sever
    cli = NimbusClient.NimbusClient(rospy.get_param('/nimbus_ip', '192.168.0.69'))
    time.sleep(0.5)

    exposure_mode = rospy.get_param('~exposure_mode')
    exposure = rospy.get_param('~exposure')
    
    if(exposure_mode == "AUTO"):
        cli.setExposureMode(NimbusClient.AUTO)
        cli.setAmplitude(exposure)
    elif(exposure_mode == "AUTO_HDR"):
        cli.setExposureMode(NimbusClient.AUTO_HDR)  
        cli.setAmplitude(exposure)
    elif(exposure_mode == "MANUAL"):
        cli.setExposureMode(NimbusClient.MANUAL)  
        cli.setExposure(exposure)
    elif(exposure_mode == "MANUAL_HDR"):
        cli.setExposureMode(NimbusClient.MANUAL_HDR) 
        cli.setExposure(exposure)           

    #main loop
    while not rospy.is_shutdown():
        #get image data from nimbus
        header, (ampl, radial, x, y, z, conf) = cli.getImage(invalidAsNan=True)
        
        #convert pointcloud data for point_cloud2
        if(rospy.get_param('~pointcloud')):
            cloud = np.transpose(np.vstack((x.flatten(), y.flatten(), z.flatten(), ampl.flatten()))).tolist()
            header = Header()
            header.frame_id = "nimbus"
            pc2 = point_cloud2.create_cloud(header, fields, cloud)
            pc2.header.stamp = rospy.Time.now()
            pcl_pub.publish(pc2)

        #create compressed intensity image
        if(rospy.get_param('~intensity_image')):
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', cloud_to_intensity_image(ampl))[1]).tostring()
            image_pub.publish(msg)

        rospy.loginfo("Published Image")


if __name__ == '__main__':
    try:
        talker()
        os._exit(os.EX_OK)
    except rospy.ROSInterruptException:
        pass