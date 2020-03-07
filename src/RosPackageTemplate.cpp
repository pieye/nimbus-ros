#include "nimbus-ros/RosPackageTemplate.hpp"

bool m_new_image = false;
bool m_auto_exposure_update = false;
const int m_img_width = 352;
const int m_img_height = 286;
const float m_XYZ_to_m = 0.0002; 

PointCloud::Ptr m_nimbus_cloud(new PointCloud);

sensor_msgs::Image m_range_image;
sensor_msgs::Image m_intensity_image;
AutoExposureParams_t m_params;

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

HsvColor RgbToHsv(RgbColor rgb)
{
    HsvColor hsv;
    unsigned char rgbMin, rgbMax;

    rgbMin = rgb.r < rgb.g ? (rgb.r < rgb.b ? rgb.r : rgb.b) : (rgb.g < rgb.b ? rgb.g : rgb.b);
    rgbMax = rgb.r > rgb.g ? (rgb.r > rgb.b ? rgb.r : rgb.b) : (rgb.g > rgb.b ? rgb.g : rgb.b);

    hsv.v = rgbMax;
    if (hsv.v == 0)
    {
        hsv.h = 0;
        hsv.s = 0;
        return hsv;
    }

    hsv.s = 255 * long(rgbMax - rgbMin) / hsv.v;
    if (hsv.s == 0)
    {
        hsv.h = 0;
        return hsv;
    }

    if (rgbMax == rgb.r)
        hsv.h = 0 + 43 * (rgb.g - rgb.b) / (rgbMax - rgbMin);
    else if (rgbMax == rgb.g)
        hsv.h = 85 + 43 * (rgb.b - rgb.r) / (rgbMax - rgbMin);
    else
        hsv.h = 171 + 43 * (rgb.r - rgb.g) / (rgbMax - rgbMin);

    return hsv;
}


//Callback to get measurement data directly from nimbus
void imageCallback(void* unused0, void* img, void* unused1) {
    auto start = std::chrono::steady_clock::now();

    ImgHeader_t* header = nimbus_seq_get_header(img);
    uint16_t* ampl = nimbus_seq_get_amplitude(img);
    int16_t* x = nimbus_seq_get_x(img);
    int16_t* y = nimbus_seq_get_y(img);
    int16_t* z = nimbus_seq_get_z(img);
    uint8_t* conf = nimbus_seq_get_confidence(img);

    //Move valid points into the point cloud and the corresponding images
    for(int i = 0; i < (m_img_width*m_img_height); i++)
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
    auto end = std::chrono::steady_clock::now();
    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " us" << std::endl;

    nimbus_seq_del(img); //<- free the image pointer this call is necessary to return img resource to nimbus
    m_new_image = true;
}

void cloud_to_image(){
    for(int i = 0; i < (m_img_width*m_img_height); i++){
        if(m_nimbus_cloud->points[i].intensity != NAN){
            RgbColor rgb_range = HsvToRgb(std::min(std::max((m_nimbus_cloud->points[i].z*50), 0.0f), 255.0f),255,255);
            m_range_image.data[i*3]             = rgb_range.r;
            m_range_image.data[i*3+1]           = rgb_range.g;
            m_range_image.data[i*3+2]           = rgb_range.b;
            RgbColor rgb_intens = HsvToRgb(std::min(std::max((m_nimbus_cloud->points[i].intensity/10), 0.0f), 255.0f),255,255);
            m_intensity_image.data[i*3]         = rgb_intens.r;
            m_intensity_image.data[i*3+1]       = rgb_intens.g;
            m_intensity_image.data[i*3+2]       = rgb_intens.b;
        }
        else{
            m_range_image.data[i*3]             = 0;
            m_range_image.data[i*3+1]           = 0;
            m_range_image.data[i*3+2]           = 0;
            m_intensity_image.data[i*3]         = 0;
            m_intensity_image.data[i*3+1]       = 0;
            m_intensity_image.data[i*3+2]       = 0;
        }
    }
}