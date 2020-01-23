#include <nimbusPreprocessInterface.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Image.h>

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
PointCloud::Ptr nimbus_cloud(new PointCloud);
sensor_msgs::Image range_image;
sensor_msgs::Image intensity_image;

const int img_width = 352;
const int img_height = 286;

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
                nimbus_cloud->points[i].x         = (float)x[i]/5000;
                nimbus_cloud->points[i].y         = (float)y[i]/5000;
                nimbus_cloud->points[i].z         = (float)z[i]/5000;
                nimbus_cloud->points[i].intensity = ampl[i];
                range_image.data[i]               = z[i]/100;
                intensity_image.data[i]           = ampl[i]/20;
            }
            else{
                nimbus_cloud->points[i].x         =  std::numeric_limits<float>::quiet_NaN();
                nimbus_cloud->points[i].y         =  std::numeric_limits<float>::quiet_NaN();
                nimbus_cloud->points[i].z         =  std::numeric_limits<float>::quiet_NaN();
                nimbus_cloud->points[i].intensity =  std::numeric_limits<float>::quiet_NaN();
                range_image.data[i]               = 0;
                intensity_image.data[i]           = ampl[i]/20;
            }
        }
    nimbus_seq_del(img); //<- free the image pointer this call is necessary to return img resource to nimbus
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nibus_ros");

    ros::NodeHandle nh;
    ros::Publisher pointcloud_pub = nh.advertise<PointCloud>("pointcloud", 10);
    ros::Publisher range_image_pub = nh.advertise<sensor_msgs::Image>("range_image", 10);
    ros::Publisher intensity_image_pub = nh.advertise<sensor_msgs::Image>("intensity_image", 10);

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

    //Todo - implement pointcloud downsampling
    int downsampling = nh.param("downsampling", 1);

    ROS_INFO_STREAM("Nimbus-userland version: " << nimbus_get_version());

    if (nimbus_preproc_init()) {
        nimbus_preproc_seq_cb(imageCallback);
    
        ros::Rate r(20);
        while (nh.ok())
        {
            pcl_conversions::toPCL(ros::Time::now(), nimbus_cloud->header.stamp);
            pointcloud_pub.publish(nimbus_cloud);
            range_image_pub.publish(range_image);
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
