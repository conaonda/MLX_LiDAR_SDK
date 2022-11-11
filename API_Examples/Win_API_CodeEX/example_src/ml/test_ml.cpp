// test_ml.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <conio.h>
#include "ml/libsoslab_ml.h"
#include "opencv2/opencv.hpp"

cv::Scalar color[] = { {0, 0, 255},  {0, 255, 0},  {255, 0, 0}, {255, 255, 255} }; //BGR Type

std::shared_ptr<SOSLAB::LidarML> lidar_ml;

std::string filename;
uint64_t total_size = 0;

bool click = true;
cv::Point picking_point_(0, 0);
static void onMouse(int event, int x, int y, int, void*)
{
	switch (event) {
	case cv::EVENT_LBUTTONUP:
		picking_point_.x = x;
		picking_point_.y = y;
		click = true;
		break;

	default:
		break;
	}
}

void helper(void) {
	std::cout << std::endl;
	std::cout << "-------------------- Key Input Helper -------------------" << std::endl;
	std::cout << "q/Q: Quit" << std::endl;
	std::cout << "---------------------------------------------------------" << std::endl;
	std::cout << std::endl;
}

bool keyinput_checker(char kb_input)
{
	bool retval = true;

	switch (kb_input) {
	case 'q':
	case 'Q':
		if (!lidar_ml->stop()) {
			std::cerr << "LiDAR HL :: stop failed." << std::endl;
		}
		else {
			std::cout << "LiDAR HL :: stop." << std::endl;
		}
		retval = false;
		break;
	default:
		break;
	}

	return retval;
}

int main()
{
	bool success = false;

	lidar_ml = std::make_shared<SOSLAB::LidarML>();

	std::cout << lidar_ml->api_info() << std::endl;;
	/* 연결 정보를 활용하여 장치에 연결합니다. */
	SOSLAB::ip_settings_t ip_settings_device;
	SOSLAB::ip_settings_t ip_settings_pc;
	ip_settings_pc.ip_address = "0.0.0.0";
	ip_settings_pc.port_number = 0;
	ip_settings_device.ip_address = "192.168.1.10";
	ip_settings_device.port_number = 2000;
		
	success = lidar_ml->connect(ip_settings_device, ip_settings_pc);
	if (!success) {
		std::cerr << "LiDAR ML :: connection failed." << std::endl;
		return 0;
	}

	/* Data Selection */
    lidar_ml->ambient_enable(true);     //Ambient enable (True / False)
    lidar_ml->depth_enable(true);       //Depth enable (True / False)
    lidar_ml->intensity_enable(true);   //Intensity enable (True / False)
    lidar_ml->multi_echo_enable(true);  //Multi Echo enable (True / False)

	success = lidar_ml->run();

	if (!success) {
		std::cerr << "LiDAR ML :: run failed." << std::endl;
	}
	else {
		std::cout << "LiDAR ML :: run." << std::endl;
	}
	
	helper();

	std::string ambient_viz_name = "Ambient Image";
	std::string depth_viz_name = "Depth Image";
	std::string intensity_viz_name = "Intensity Image";

	cv::namedWindow(ambient_viz_name, cv::WINDOW_GUI_NORMAL);
	cv::namedWindow(depth_viz_name, cv::WINDOW_GUI_NORMAL);
	cv::namedWindow(intensity_viz_name, cv::WINDOW_GUI_NORMAL);
	cv::setMouseCallback(depth_viz_name, onMouse, 0);

	int winwidth = 400;
	int winheight = 200;

	cv::resizeWindow(ambient_viz_name, winwidth, winheight);
	cv::resizeWindow(depth_viz_name, winwidth, winheight);
	cv::resizeWindow(intensity_viz_name, winwidth, winheight);

	cv::moveWindow(ambient_viz_name, 100, 100);
	cv::moveWindow(depth_viz_name, 100 + winwidth, 100);
	cv::moveWindow(intensity_viz_name, 100 + (winwidth * 2), 100);

	int idx = 0;
	while (keyinput_checker(cv::waitKey(1))) {
		/* Stream FIFO로부터 한 프레임씩 Lidar data를 가져옵니다. */
		SOSLAB::LidarML::scene_t scene;

		if (lidar_ml->get_scene(scene)) {

			std::vector<uint32_t> ambient = scene.ambient_image;		//Lidar ambient 정보입니다.
			std::vector<uint32_t> depth = scene.depth_image[0];				//Lidar depth 정보입니다.
			std::vector<uint16_t> intensity = scene.intensity_image[0];	//Lidar intensity 정보입니다.
			std::vector<SOSLAB::point_t> pointcloud = scene.pointcloud[0];	//Lidar PointCloud 정보입니다.
			std::size_t height = scene.rows;	// Lidar frame의 height 정보입니다.
			std::size_t width = scene.cols;		// Lidar frame의 width 정보입니다.

			int ambient_max = 2000;
			int depth_max = 10000;
			int intensity_max = 2000;

			/* ambient Image */
			/* 측정 된 모든 빛을 표현 한 데이터 입니다. */
			cv::Mat ambient_image_raw(height, width*3, CV_32SC1, ambient.data());
			cv::Mat ambient_rgb;

			ambient_image_raw.convertTo(ambient_rgb, CV_8UC1, (255.0 / (ambient_max - 0)), 0);
			cv::applyColorMap(ambient_rgb, ambient_rgb, cv::COLORMAP_VIRIDIS);
			cv::imshow(ambient_viz_name, ambient_rgb);

			/* Depth Image */
			/* 원점(Lidar)로부터의 거리 정보입니다. (unit: mm) */
			cv::Mat depth_image_raw(height, width, CV_32SC1, depth.data());
			cv::Mat depth_rgb;

			depth_image_raw.convertTo(depth_rgb, CV_8UC1, (255.0 / (depth_max - 0)), 0);
			cv::applyColorMap(depth_rgb, depth_rgb, cv::COLORMAP_JET);
			cv::imshow(depth_viz_name, depth_rgb);

			/* Intensity Image */
			/* 측정 된 Lidar 신호의 강도 입니다. */
			cv::Mat intensity_image_raw(height, width, CV_16UC1, intensity.data());
			cv::Mat intensity_rgb;

			intensity_image_raw.convertTo(intensity_rgb, CV_8UC1, (255.0 / (intensity_max - 0)), 0);
			cv::applyColorMap(ambient_rgb, ambient_rgb, cv::COLORMAP_VIRIDIS);
			cv::imshow(intensity_viz_name, intensity_rgb);

			if (click) {
				std::size_t col = picking_point_.x;
				std::size_t row = picking_point_.y;

				std::size_t idx = col + (width * row);

				// 단위 : (m)
				std::cout << "[Row,Col][Point cloud data] = [" << row << ", " << col << "] , [x =" << pointcloud[idx].x/1000.0f << "m],[y = " << pointcloud[idx].y / 1000.0f << "m],[z = " << pointcloud[idx].z / 1000.0f << "m]" << std::endl;		// x-axis, y-axis, z-axis 

				click = false;
			}
		}
	}
	
	cv::destroyAllWindows();

	/* 스트리밍을 종료 합니다. */
	lidar_ml->stop();

	std::cout << "Streaming stopped!" << std::endl;

	/* 장치 연결을 해제합니다. */
	lidar_ml->disconnect();

	std::cout << "Done." << std::endl;
}
