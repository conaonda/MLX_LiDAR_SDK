/*
 * libsoslab_ml.h
 *
 *  Created on: Dec 02, 2019
 *      Author: gnohead
 */

#ifndef LIDAR_ML_H_
#define LIDAR_ML_H_

#include "soslab_typedef.h"
#include <array>
#include <fstream>

namespace SOSLAB
{
	class SOSLAB_EXPORTS LidarML
	{
	public:
		typedef struct _ML_SCENE_T {
			std::vector<uint64_t> timestamp;
			uint64_t status;
			uint8_t frame_id;
			uint8_t lidar_id;
			uint16_t rows;
			uint16_t cols;
			std::vector<uint32_t> ambient_image;
			std::vector<std::vector<uint32_t>> depth_image; 
			std::vector<std::vector<uint16_t>> intensity_image;
			std::vector<std::vector<point_t>> pointcloud;
		} scene_t;

		explicit LidarML();
		~LidarML();

		static std::string api_info();

		bool connect(const ip_settings_t ml, const ip_settings_t local);
		void disconnect();

		bool record_start(const std::string filepath);
		void record_stop();

		bool play_start(const std::string filepath);
		void play_stop();
		bool get_file_size(uint64_t& size);
		bool get_number_of_lidar_playmode(size_t& lidar_num);

		bool run();
		bool stop();

		bool emission_inhibit(bool en);
		bool lp_mode(bool en);
		bool hv_ctrl(bool en);
		bool fir_filter(bool en);
		bool depth_completion(bool en);
		bool channelwise_power_control(bool en);

		bool ambient_enalbe(bool en);
		bool depth_enalbe(bool en);
		bool intensity_enalbe(bool en);
		bool multi_echo_enalbe(bool en);

		bool get_scene(scene_t& scene);
		bool get_scene(scene_t& scene, int idx);

	private:
		void* lidar_;
	};

}   /* namespace SOSLAB */
#endif /* LIDAR_ML_H_ */
