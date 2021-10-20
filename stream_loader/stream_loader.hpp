#pragma once

#include <string>
#include <memory>
#include <unordered_map>

#include <Eigen/Geometry>

#include <filesystem>

#include "csv.h"

#include "stream_loader/Data.hpp"

namespace camvox_loader {
	
class StreamLoader {
public:
	StreamLoader(const std::string &root_dir);

	Data LoadNextData();

private:
	void LoadGtPoses();
	bool CheckExpectPathExit() const;

	std::unordered_map<uint64_t, Eigen::Isometry3d> gt_poses_;

	std::unique_ptr<io::CSVReader<9>> seq_reader_ptr_ = nullptr;

	std::filesystem::path root_;
	std::filesystem::path img_dir_;
	std::filesystem::path lidar_dir_;
	std::filesystem::path gt_file_;
	std::filesystem::path seq_file_;

	// Expected header for groundtruth csv file
	std::string gtheader_tstamp_str_ = std::string("timestamp");

	std::string gtheader_x_str_ = std::string("position_x");
	std::string gtheader_y_str_ = std::string("position_y");
	std::string gtheader_z_str_ = std::string("position_z");

	std::string gtheader_quat_x_str_ = std::string("quat_x");
	std::string gtheader_quat_y_str_ = std::string("quat_y");
	std::string gtheader_quat_z_str_ = std::string("quat_z");
	std::string gtheader_quat_w_str_ = std::string("quat_w");

	// Expected header for sequence csv file
	std::string seqheader_tstamp_str_ = std::string("timestamp");
	std::string seqheader_dtpe_str_ = std::string("data_type");
	std::string seqheader_location_str_ = std::string("location");
	std::string seqheader_wx_str_ = std::string("wx");
	std::string seqheader_wy_str_ = std::string("wy");
	std::string seqheader_wz_str_ = std::string("wz");
	std::string seqheader_ax_str_ = std::string("ax");
	std::string seqheader_ay_str_ = std::string("ay");
	std::string seqheader_az_str_ = std::string("az");
};

}