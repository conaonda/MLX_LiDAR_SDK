/*
 * libsoslab_ml.h
 */

#ifndef LIDAR_ML_H_
#define LIDAR_ML_H_


#include "soslab_typedef.h"
#include "core/json.hpp"
#include <array>
#include <fstream>

namespace SOSLAB
{
	class SOSLAB_EXPORTS LidarML
	{
	public:
		typedef struct _MLX_SCENE_T {
			std::size_t rows;
			std::size_t cols;
			std::vector<uint32_t> ambient_image;
			std::vector<float> depth_image;
			std::vector<uint16_t> intensity_image;
			pointcloud_t pointcloud;
		} scene_t;

		typedef nlohmann::json json_t;

		explicit LidarML();
		~LidarML();

		static std::string api_info();

		bool connect(const ip_settings_t ml, const ip_settings_t local);
		void disconnect();

		bool run();
		bool stop();

		bool get_scene(scene_t& scene);
		void set_signal_data_processing(bool enable);

		bool record_start(const std::string filepath);
		void record_stop();

		bool play_start(const std::string filepath);
		void play_stop();

		bool get_scene(scene_t& scene, int idx);
		bool get_file_size(uint64_t& size);


		bool send_json(json_t& json_packet);
		bool json_read_from_file(const char* name, json_t& json_arg);
		bool json_write_to_memory(); 
	private:
		void* lidar_;

	};

}   /* namespace SOSLAB */



#endif /* LIDAR_ML_H_ */
