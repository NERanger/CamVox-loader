#include <iostream>
#include <vector>
#include <unordered_map>

#include <pcl/io/pcd_io.h>

#include "stream_loader/stream_loader.hpp"

using std::cout;
using std::cerr;
using std::endl;

using camvox_loader::StreamLoader;
using camvox_loader::Data;

StreamLoader::StreamLoader(const std::string& root_dir) : root_(root_dir) {
	using std::filesystem::exists;

	img_dir_ = root_ / ("img");
	lidar_dir_ = root_ / ("lidar");
	gt_file_ = root_ / ("groundtruth.csv");
	seq_file_ = root_ / ("sequence.csv");

	cout << "[CamvoxLoader] Expected data path:" << endl
		 << "-- RGB image: " << img_dir_.string() << endl
		 << "-- LiDAR ptcloud: " << lidar_dir_.string() << endl
		 << "-- Groundtruth: " << gt_file_.string() << endl
		 << "-- Sequence: " << seq_file_.string() << endl;

	// Check img and lidar ptcloud
	if (!CheckExpectPathExit()) {
		cerr << "[CamvoxLoader] Dataset not complete, check if the desired data path exists." << endl;
		exit(EXIT_FAILURE);
	}

	LoadGtPoses();

	seq_reader_ptr_.reset(new io::CSVReader<9>(seq_file_.string()));
	seq_reader_ptr_->read_header(io::ignore_extra_column, seqheader_tstamp_str_,
		seqheader_dtpe_str_, seqheader_location_str_,
		seqheader_wx_str_, seqheader_wy_str_, seqheader_wz_str_,
		seqheader_ax_str_, seqheader_ay_str_, seqheader_az_str_);
}

void StreamLoader::LoadGtPoses(){
	using std::vector;
	using std::make_pair;
	using Eigen::Vector3d;
	using Eigen::Matrix3d;
	using Eigen::Isometry3d;
	using Eigen::Quaterniond;

	cout << "Loading groundtruth pose..." << endl;

	io::CSVReader<8> gt_reader(gt_file_.string());
	gt_reader.read_header(io::ignore_extra_column, gtheader_tstamp_str_,
						  gtheader_x_str_, gtheader_y_str_, gtheader_z_str_,
						  gtheader_quat_x_str_, gtheader_quat_y_str_, gtheader_quat_z_str_, gtheader_quat_w_str_);

	uint64_t timestamp;
	double position_x, position_y, position_z;
	double quat_x, quat_y, quat_z, quat_w;

	bool first_gt = true;

	// Use first frame as origin
	Vector3d trans_origin;
	Matrix3d rot_origin;

	// Populate trans and quat vector
	while (gt_reader.read_row(timestamp, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w)) {
		if (first_gt) {
			trans_origin = Vector3d(position_x, position_y, position_z);
			rot_origin = Quaterniond(quat_w, quat_x, quat_y, quat_z).toRotationMatrix();
		}
		
		Vector3d trans(position_x, position_y, position_z);
		Matrix3d rot(Quaterniond(quat_w, quat_x, quat_y, quat_z).toRotationMatrix());

		trans -= trans_origin;
		rot = rot_origin.transpose() * rot;

		Isometry3d Twb(rot);
		Twb.pretranslate(trans);

		gt_poses_.emplace(make_pair(timestamp, Twb));
	}

	cout << "Loading groundtruth pose... Done" << endl;
}

bool StreamLoader::CheckExpectPathExit() const{
	using std::filesystem::exists;

	if (!exists(img_dir_)) {
		return false;
	}
	if (!exists(lidar_dir_)) {
		return false;
	}
	if (!exists(gt_file_)) {
		return false;
	}
	if (!exists(seq_file_)) {
		return false;
	}

	return true;
}

Data StreamLoader::LoadNextData() {
	using std::string;
	using std::stof;
	using std::filesystem::path;
	using camvox_loader::DataType;
	using camvox_loader::ImuData;

	static const string kDTypeImgStr = string("img");
	static const string kDTypeImuStr = string("imu");
	static const string kDTypeLidarStr = string("lidar");

	uint64_t timestamp;
	string d_type, location;
	string wx, wy, wz, ax, ay, az;
	seq_reader_ptr_->read_row(timestamp, d_type, location, wx, wy, wz, ax, ay, az);

	Data d(timestamp);
	if (d_type == kDTypeImgStr) {
		path p = root_ / location;
		d.SetDataType(kDTypeImg);
		d.SetImg(cv::imread(p.string(), cv::IMREAD_GRAYSCALE));
		return d;
	}
	if (d_type == kDTypeImuStr) {
		d.SetDataType(kDTypeImu);
		d.SetImu(ImuData{ stof(ax), stof(ay), stof(az), stof(wx), stof(wy), stof(wz)});
		return d;
	}
	if (d_type == kDTypeLidarStr) {
		path p = root_ / location;
		d.SetDataType(kDTypeLidar);
		pcl::PointCloud<LivoxPoint>::Ptr ptcloud_ptr(new pcl::PointCloud<LivoxPoint>);
		pcl::io::loadPCDFile<LivoxPoint>(p.string(), *ptcloud_ptr);
		d.SetPtcloud(ptcloud_ptr);
		return d;
	}
}