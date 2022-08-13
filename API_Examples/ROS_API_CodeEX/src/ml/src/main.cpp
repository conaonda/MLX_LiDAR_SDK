#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include "libsoslab_ml.h"

typedef pcl::PointXYZRGB PointRGB_T;
typedef pcl::PointCloud<PointRGB_T> PointCloud_T;

int nCols = 0;
int nRows = 0;

static const float soslab_r[] = {0.031373f, 0.035294f, 0.039216f, 0.043137f, 0.043137f, 0.047059f, 0.050980f, 0.050980f, 0.054902f, 0.058824f, 0.062745f, 0.062745f, 0.066667f, 0.070588f, 0.074510f, 0.074510f, 0.078431f, 0.082353f, 0.082353f, 0.086275f, 0.090196f, 0.094118f, 0.094118f, 0.098039f, 0.101961f, 0.101961f, 0.105882f, 0.109804f, 0.113725f, 0.113725f, 0.117647f, 0.121569f, 0.121569f, 0.125490f, 0.129412f, 0.133333f, 0.133333f, 0.137255f, 0.141176f, 0.141176f, 0.145098f, 0.149020f, 0.152941f, 0.152941f, 0.156863f, 0.160784f, 0.164706f, 0.164706f, 0.168627f, 0.172549f, 0.172549f, 0.176471f, 0.180392f, 0.184314f, 0.184314f, 0.188235f, 0.192157f, 0.192157f, 0.196078f, 0.200000f, 0.203922f, 0.203922f, 0.207843f, 0.211765f, 0.215686f, 0.215686f, 0.219608f, 0.223529f, 0.227451f, 0.227451f, 0.231373f, 0.235294f, 0.235294f, 0.239216f, 0.243137f, 0.247059f, 0.247059f, 0.250980f, 0.254902f, 0.254902f, 0.258824f, 0.262745f, 0.266667f, 0.266667f, 0.270588f, 0.274510f, 0.274510f, 0.278431f, 0.282353f, 0.286275f, 0.286275f, 0.290196f, 0.294118f, 0.294118f, 0.298039f, 0.301961f, 0.305882f, 0.305882f, 0.309804f, 0.313725f, 0.317647f, 0.317647f, 0.321569f, 0.325490f, 0.325490f, 0.329412f, 0.333333f, 0.337255f, 0.337255f, 0.341176f, 0.345098f, 0.345098f, 0.349020f, 0.352941f, 0.356863f, 0.356863f, 0.360784f, 0.364706f, 0.364706f, 0.368627f, 0.372549f, 0.376471f, 0.376471f, 0.380392f, 0.384314f, 0.388235f, 0.388235f, 0.392157f, 0.396078f, 0.400000f, 0.403922f, 0.411765f, 0.415686f, 0.419608f, 0.423529f, 0.431373f, 0.435294f, 0.439216f, 0.443137f, 0.447059f, 0.454902f, 0.458824f, 0.462745f, 0.470588f, 0.474510f, 0.478431f, 0.482353f, 0.486275f, 0.494118f, 0.498039f, 0.501961f, 0.505882f, 0.509804f, 0.517647f, 0.521569f, 0.525490f, 0.529412f, 0.533333f, 0.541176f, 0.545098f, 0.549020f, 0.552941f, 0.560784f, 0.564706f, 0.568627f, 0.572549f, 0.576471f, 0.584314f, 0.588235f, 0.592157f, 0.596078f, 0.600000f, 0.607843f, 0.611765f, 0.615686f, 0.619608f, 0.623529f, 0.631373f, 0.635294f, 0.639216f, 0.643137f, 0.650980f, 0.654902f, 0.658824f, 0.662745f, 0.666667f, 0.674510f, 0.678431f, 0.682353f, 0.686275f, 0.690196f, 0.698039f, 0.701961f, 0.705882f, 0.709804f, 0.713725f, 0.721569f, 0.725490f, 0.729412f, 0.737255f, 0.741176f, 0.745098f, 0.749020f, 0.752941f, 0.760784f, 0.764706f, 0.768627f, 0.772549f, 0.780392f, 0.784314f, 0.788235f, 0.792157f, 0.796078f, 0.803922f, 0.807843f, 0.811765f, 0.815686f, 0.819608f, 0.827451f, 0.831373f, 0.835294f, 0.839216f, 0.843137f, 0.850980f, 0.854902f, 0.858824f, 0.862745f, 0.870588f, 0.874510f, 0.878431f, 0.882353f, 0.886275f, 0.894118f, 0.898039f, 0.901961f, 0.905882f, 0.909804f, 0.917647f, 0.921569f, 0.925490f, 0.929412f, 0.937255f, 0.941176f, 0.945098f, 0.949020f, 0.952941f, 0.960784f, 0.964706f, 0.968627f, 0.972549f, 0.976471f, 0.984314f, 0.988235f, 0.992157f, 0.996078f, 0.996078f};
static const float soslab_g[] = {0.125490f, 0.129412f, 0.137255f, 0.145098f, 0.152941f, 0.160784f, 0.168627f, 0.172549f, 0.180392f, 0.188235f, 0.196078f, 0.200000f, 0.207843f, 0.215686f, 0.223529f, 0.227451f, 0.235294f, 0.243137f, 0.250980f, 0.254902f, 0.262745f, 0.270588f, 0.278431f, 0.282353f, 0.290196f, 0.298039f, 0.305882f, 0.309804f, 0.317647f, 0.325490f, 0.333333f, 0.337255f, 0.345098f, 0.352941f, 0.360784f, 0.364706f, 0.372549f, 0.380392f, 0.388235f, 0.392157f, 0.400000f, 0.407843f, 0.415686f, 0.419608f, 0.427451f, 0.435294f, 0.443137f, 0.447059f, 0.454902f, 0.462745f, 0.470588f, 0.474510f, 0.482353f, 0.490196f, 0.498039f, 0.501961f, 0.509804f, 0.517647f, 0.525490f, 0.529412f, 0.541176f, 0.545098f, 0.552941f, 0.560784f, 0.568627f, 0.572549f, 0.580392f, 0.588235f, 0.596078f, 0.600000f, 0.607843f, 0.615686f, 0.623529f, 0.627451f, 0.635294f, 0.643137f, 0.650980f, 0.654902f, 0.662745f, 0.670588f, 0.678431f, 0.682353f, 0.690196f, 0.698039f, 0.705882f, 0.709804f, 0.717647f, 0.725490f, 0.733333f, 0.737255f, 0.745098f, 0.752941f, 0.760784f, 0.764706f, 0.772549f, 0.780392f, 0.788235f, 0.792157f, 0.800000f, 0.807843f, 0.815686f, 0.819608f, 0.827451f, 0.835294f, 0.843137f, 0.847059f, 0.854902f, 0.862745f, 0.870588f, 0.874510f, 0.882353f, 0.890196f, 0.898039f, 0.901961f, 0.909804f, 0.917647f, 0.925490f, 0.929412f, 0.937255f, 0.945098f, 0.952941f, 0.960784f, 0.964706f, 0.972549f, 0.980392f, 0.988235f, 0.992157f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f, 1.000000f};
static const float soslab_b[] = {0.788235f, 0.792157f, 0.792157f, 0.796078f, 0.796078f, 0.796078f, 0.800000f, 0.800000f, 0.803922f, 0.803922f, 0.803922f, 0.807843f, 0.807843f, 0.811765f, 0.811765f, 0.815686f, 0.815686f, 0.815686f, 0.819608f, 0.819608f, 0.823529f, 0.823529f, 0.827451f, 0.827451f, 0.827451f, 0.831373f, 0.831373f, 0.835294f, 0.835294f, 0.835294f, 0.839216f, 0.839216f, 0.843137f, 0.843137f, 0.847059f, 0.847059f, 0.847059f, 0.850980f, 0.850980f, 0.854902f, 0.854902f, 0.858824f, 0.858824f, 0.858824f, 0.862745f, 0.862745f, 0.866667f, 0.866667f, 0.870588f, 0.870588f, 0.870588f, 0.874510f, 0.874510f, 0.878431f, 0.878431f, 0.878431f, 0.882353f, 0.882353f, 0.886275f, 0.886275f, 0.890196f, 0.890196f, 0.894118f, 0.894118f, 0.894118f, 0.898039f, 0.898039f, 0.901961f, 0.901961f, 0.901961f, 0.905882f, 0.905882f, 0.909804f, 0.909804f, 0.913725f, 0.913725f, 0.913725f, 0.917647f, 0.917647f, 0.921569f, 0.921569f, 0.925490f, 0.925490f, 0.925490f, 0.929412f, 0.929412f, 0.933333f, 0.933333f, 0.933333f, 0.937255f, 0.937255f, 0.941176f, 0.941176f, 0.945098f, 0.945098f, 0.945098f, 0.949020f, 0.949020f, 0.952941f, 0.952941f, 0.956863f, 0.956863f, 0.956863f, 0.960784f, 0.960784f, 0.964706f, 0.964706f, 0.968627f, 0.968627f, 0.968627f, 0.972549f, 0.972549f, 0.976471f, 0.976471f, 0.976471f, 0.980392f, 0.980392f, 0.984314f, 0.984314f, 0.988235f, 0.988235f, 0.992157f, 0.992157f, 0.992157f, 0.996078f, 0.996078f, 1.000000f, 1.000000f, 0.992157f, 0.984314f, 0.976471f, 0.968627f, 0.960784f, 0.952941f, 0.945098f, 0.937255f, 0.929412f, 0.921569f, 0.913725f, 0.905882f, 0.898039f, 0.890196f, 0.882353f, 0.874510f, 0.866667f, 0.858824f, 0.850980f, 0.843137f, 0.835294f, 0.827451f, 0.819608f, 0.811765f, 0.803922f, 0.796078f, 0.788235f, 0.780392f, 0.772549f, 0.764706f, 0.756863f, 0.749020f, 0.741176f, 0.733333f, 0.725490f, 0.717647f, 0.709804f, 0.701961f, 0.694118f, 0.686275f, 0.678431f, 0.670588f, 0.662745f, 0.654902f, 0.647059f, 0.639216f, 0.631373f, 0.623529f, 0.615686f, 0.607843f, 0.600000f, 0.592157f, 0.584314f, 0.576471f, 0.568627f, 0.560784f, 0.552941f, 0.545098f, 0.537255f, 0.529412f, 0.521569f, 0.513725f, 0.505882f, 0.498039f, 0.490196f, 0.482353f, 0.474510f, 0.466667f, 0.458824f, 0.450980f, 0.443137f, 0.435294f, 0.427451f, 0.419608f, 0.411765f, 0.403922f, 0.396078f, 0.388235f, 0.380392f, 0.372549f, 0.364706f, 0.356863f, 0.349020f, 0.341176f, 0.333333f, 0.325490f, 0.317647f, 0.309804f, 0.301961f, 0.294118f, 0.286275f, 0.278431f, 0.270588f, 0.262745f, 0.254902f, 0.247059f, 0.239216f, 0.231373f, 0.223529f, 0.215686f, 0.207843f, 0.200000f, 0.192157f, 0.184314f, 0.176471f, 0.168627f, 0.160784f, 0.152941f, 0.145098f, 0.137255f, 0.129412f, 0.121569f, 0.113725f, 0.105882f, 0.098039f, 0.090196f, 0.082353f, 0.074510f, 0.066667f, 0.058824f, 0.050980f, 0.043137f, 0.035294f, 0.027451f, 0.019608f, 0.011765f, 0.003922f, 0.003922f};

static const char* DEFAULT_IP_ADDR_DEVICE          = "192.168.1.10";
static const unsigned short DEFAULT_IP_PORT_DEVICE = 2000;

static const char* DEFAULT_IP_ADDR_PC              = "0.0.0.0";
static const unsigned short DEFAULT_IP_PORT_PC     = 0;

static const char* DEFAULT_PACKAGE_NAME            = "ml";

// Default publisher template
static const char* DEFAULT_FRAME_ID = "map";

cv::Mat colormap(cv::Mat image)
{
    nCols = image.cols;
    nRows = image.rows;
    
    int temp_intensity = 0;

    cv::Mat rgb_image(nRows, nCols, CV_8UC3);

    for (int col = 0; col < nCols; col++){
        for (int row = 0; row < nRows; row++){
            temp_intensity = image.at<uint8_t>(row, col);
            float colormap_val0, colormap_val1, colormap_val2;
            
            colormap_val0 = soslab_r[temp_intensity];
            colormap_val1 = soslab_g[temp_intensity];
            colormap_val2 = soslab_b[temp_intensity];

            rgb_image.at<cv::Vec3b>(row, col)[0] = int(255 * colormap_val0);
            rgb_image.at<cv::Vec3b>(row, col)[1] = int(255 * colormap_val1);
            rgb_image.at<cv::Vec3b>(row, col)[2] = int(255 * colormap_val2);
        }
    }
    return rgb_image;
}

#include<thread>
int main (int argc, char **argv)
{
    bool success;

    /* ROS node init */
    ros::init(argc, argv, DEFAULT_PACKAGE_NAME);
    
    std::string frame_id = DEFAULT_FRAME_ID;

    /* get parameters */
    ros::NodeHandle nh("~");

    /* LidarML 객체를 생성합니다. */
    std::shared_ptr<SOSLAB::LidarML> lidar_ml(new SOSLAB::LidarML);

    /* IP 정보를 이용하여 장치에 연결합니다. */
    SOSLAB::ip_settings_t ip_settings_device;
    SOSLAB::ip_settings_t ip_settings_pc;

    nh.param<std::string>("ip_address_device", ip_settings_device.ip_address, DEFAULT_IP_ADDR_DEVICE);
    nh.param<int>("ip_port_device", ip_settings_device.port_number, DEFAULT_IP_PORT_DEVICE);
    nh.param<std::string>("ip_address_pc", ip_settings_pc.ip_address, DEFAULT_IP_ADDR_PC);
    nh.param<int>("ip_port_pc", ip_settings_pc.port_number, DEFAULT_IP_PORT_PC);

    std::cout << "> ip_address_device: " << ip_settings_device.ip_address << std::endl;
    std::cout << "> ip_port_device: " << ip_settings_device.port_number << std::endl;
    std::cout << "> ip_address_pc: " << ip_settings_pc.ip_address << std::endl;
    std::cout << "> ip_port_pc: " << ip_settings_pc.port_number << std::endl;

    success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
    if (!success) {
        std::cerr << "LiDAR ML :: Connection failed." << std::endl;
        return 0;
    }
    
    /* 데이터 스트리밍을 시작 합니다. */
    success = lidar_ml->run();
    if (!success) {
        std::cerr << "LiDAR ML :: Start failed." << std::endl;
        return 0;
    }

    std::cout << "LiDAR ML :: Streaming started!" << std::endl;

    /* publisher setting */
    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub_depth = it.advertise("depth_color", 1);
    image_transport::Publisher pub_intensity = it.advertise("intensity_color", 1);
    image_transport::Publisher pub_ambient = it.advertise("ambient_color", 1);
    ros::Publisher pub_lidar = nh.advertise<PointCloud_T>("pointcloud", 10);


    sensor_msgs::ImagePtr msg_ambient;
    sensor_msgs::ImagePtr msg_depth;
    sensor_msgs::ImagePtr msg_intensity;

    PointCloud_T::Ptr msg_pointcloud(new PointCloud_T);

    /* publishing start */
    while (ros::ok()) {
        SOSLAB::LidarML::scene_t scene;
        /* Stream FIFO로부터 한 프레임씩 Lidar data를 가져옵니다. */
        if (lidar_ml->get_scene(scene)) {
            std::vector<uint32_t> ambient = scene.ambient_image;
            std::vector<uint16_t> intensity = scene.intensity_image;
            std::vector<float> depth = scene.depth_image;
            std::vector<SOSLAB::point_t> pointcloud = scene.pointcloud;


            std::size_t height = scene.rows;	// Lidar frame의 height 정보입니다.
            std::size_t width = scene.cols;	// Lidar frame의 width 정보입니다.

            /* Ambient Image */
            /* 측정 된 모든 빛을 표현 한 데이터 입니다. */
            cv::Mat ambient_image(height, width, CV_32SC1, ambient.data());
            ambient_image.convertTo(ambient_image, CV_8UC1, (255.0 / (2000 - 0)), 0);
            ambient_image = colormap(ambient_image);
            cv::normalize(ambient_image, ambient_image, 0, 255, cv::NORM_MINMAX);
            if (pub_ambient.getNumSubscribers() > 0) {
                msg_ambient = cv_bridge::CvImage(std_msgs::Header(), "rgb8", ambient_image).toImageMsg();
                pub_ambient.publish(msg_ambient);
            }

            /* Depth Image */
            /* 원점(Lidar)로부터의 거리 정보입니다. (unit: mm) */
            cv::Mat depth_image(height, width, CV_32FC1, depth.data());

            depth_image.convertTo(depth_image, CV_16U);
            depth_image.convertTo(depth_image, CV_8UC1, (255.0 / (80 - 0)), 0);
            // depth_image.convertTo(depth_image, CV_8U, 1.0 / 64.0);
            depth_image = colormap(depth_image);
            cv::normalize(depth_image, depth_image, 0, 255, cv::NORM_MINMAX);
            if(pub_depth.getNumSubscribers() > 0) {
                msg_depth = cv_bridge::CvImage(std_msgs::Header(), "rgb8", depth_image).toImageMsg();
                pub_depth.publish(msg_depth);
            }

            /* Intensity Image */
            cv::Mat intensity_image_raw(height, width, CV_16UC1, intensity.data());
            cv::Mat intensity_image;
            intensity_image_raw.convertTo(intensity_image, CV_8UC1, (255.0 / (2000 - 0)), 0);
            // intensity_image_raw.convertTo(intensity_image, CV_8UC1, 1.0 / 1.0);
            intensity_image = colormap(intensity_image);

            if (pub_intensity.getNumSubscribers() > 0) {
                msg_intensity = cv_bridge::CvImage(std_msgs::Header(), "rgb8", intensity_image).toImageMsg();
                pub_intensity.publish(msg_intensity);
            }

            /* Point Cloud */
            /* normalize (min-max) */
            msg_pointcloud->header.frame_id = frame_id;
            msg_pointcloud->width = width;
            msg_pointcloud->height = height;
            msg_pointcloud->points.resize(pointcloud.size());

            for (int col=0; col < width; col++) {
                for (int row = 0; row < height; row++) {
                    int idx = col + (width * row);

                    //unit : (m)
                    msg_pointcloud->points[idx].x = pointcloud[idx].xyz.x;
                    msg_pointcloud->points[idx].y = pointcloud[idx].xyz.y;
                    msg_pointcloud->points[idx].z = pointcloud[idx].xyz.z;

                    msg_pointcloud->points[idx].r = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[0]);
                    msg_pointcloud->points[idx].g = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[1]);
                    msg_pointcloud->points[idx].b = (uint8_t)(intensity_image.at<cv::Vec3b>(row, col)[2]);
                    // int intensity_val = int(intensity_norm_8b.at<uint8_t>(row, col));
                    // msg_pointcloud->points[idx].r = (uint8_t)(soslab_r[intensity_val] * 256.f);
                    // msg_pointcloud->points[idx].g = (uint8_t)(soslab_g[intensity_val] * 256.f);
                    // msg_pointcloud->points[idx].b = (uint8_t)(soslab_b[intensity_val] * 256.f);
                }
            }

            // publish the pointcloud
            pcl_conversions::toPCL(ros::Time::now(), msg_pointcloud->header.stamp);
            pub_lidar.publish(msg_pointcloud);
        }
        ros::spinOnce();
    }

    /* 스트리밍을 종료 합니다. */
    lidar_ml->stop();
    std::cout << "Streaming stopped!" << std::endl;

    /* 장치 연결을 해제합니다. */
    lidar_ml->disconnect();

    std::cout << "Done." << std::endl;

    return 0;
}
