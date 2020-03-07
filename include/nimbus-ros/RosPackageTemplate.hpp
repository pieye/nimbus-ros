#include "nimbus-ros/driver.hpp"
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include <algorithm>
#include <unistd.h>
#include <nimbusPreprocessInterface.h>


//Define point cloud and images
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

typedef struct RgbColor
{
    unsigned char r;
    unsigned char g;
    unsigned char b;
} RgbColor;

typedef struct HsvColor
{
    unsigned char h;
    unsigned char s;
    unsigned char v;
} HsvColor;



//Global variables
extern bool m_auto_exposure_update;
extern const int m_img_width;
extern const int m_img_height;
extern const float m_XYZ_to_m;  //<-- to be determined. Correct value missings
extern PointCloud::Ptr m_nimbus_cloud;
extern bool m_new_image;


extern sensor_msgs::Image m_range_image;
extern sensor_msgs::Image m_intensity_image;
extern AutoExposureParams_t m_params;

RgbColor HsvToRgb(unsigned char h, unsigned char s, unsigned char v);
HsvColor RgbToHsv(RgbColor rgb);

void imageCallback(void* unused0, void* img, void* unused1);
void cloud_to_image();