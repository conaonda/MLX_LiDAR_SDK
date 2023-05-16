/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#ifndef LIDAR_ML_H_
#define LIDAR_ML_H_

#include "soslab_typedef.h"
#include "core/json.hpp"
#include "core/logger.h"
#include "core/ext_interfaces.h"

namespace SOSLAB
{
	class SOSLAB_EXPORTS LidarML
	{
	public:
		//User
		typedef struct _ML_SCENE_T {
			std::vector<uint64_t> timestamp;
			uint64_t status;
			uint8_t frame_id;
			uint16_t rows;
			uint16_t cols;
			std::vector<uint32_t> ambient_image;
			std::vector<std::vector<uint32_t>> depth_image; 
			std::vector<std::vector<uint16_t>> intensity_image;
			std::vector<std::vector<point_t>> pointcloud;
		} scene_t;

		typedef nlohmann::json json_t;

		explicit LidarML();
		~LidarML();

		static std::string api_info();

		bool connect(const ip_settings_t ml, const ip_settings_t local);
		bool is_connected();
		void disconnect();

		bool run();
		bool stop();

		bool record_start(const std::string filepath);
		void record_stop();

		bool play_start(const std::string filepath);
		void play_stop();
		bool get_file_size(uint64_t& size);

		//True : 10fps, False : 20fps
		bool fps10(bool en);
		bool depth_completion(bool en);
		bool set_flaring_score(float val);

		bool ambient_enable(bool en);
		bool depth_enable(bool en);
		bool intensity_enable(bool en);

		bool get_scene(scene_t& scene);
		bool get_scene(scene_t& scene, int idx);

		bool sync_localtime(void);

	private:
		void* lidar_;

	};

}   /* namespace SOSLAB */



#endif /* LIDAR_ML_H_ */
