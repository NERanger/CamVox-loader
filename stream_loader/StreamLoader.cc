#include <iostream>
#include <vector>
#include <unordered_map>

#include <pcl/io/pcd_io.h>

#include "stream_loader/StreamLoader.hpp"

using std::cout;
using std::cerr;
using std::endl;

using dataset_loader::StreamLoader;
using dataset_loader::Data;

StreamLoader::StreamLoader(const std::string& root_dir) : root_(root_dir) {
	using std::filesystem::exists;

	img_dir_ = root_ / ("img");
	lidar_dir_ = root_ / ("lidar");
	gt_file_ = root_ / ("groundtruth.csv");
	seq_file_ = root_ / ("sequence.csv");
	config_file_ = root_ / ("config.yaml");

	cout << "[CamvoxLoader] Expected data path:" << endl
		<< "-- RGB image: " << img_dir_.string() << endl
		<< "-- LiDAR ptcloud: " << lidar_dir_.string() << endl
		<< "-- Groundtruth: " << gt_file_.string() << endl
		<< "-- Sequence: " << seq_file_.string() << endl
		<< "-- Config: " << config_file_.string() << endl;

	// Check img and lidar ptcloud
	if (!CheckExpectPathExit()) {
		cerr << "[CamvoxLoader] Dataset not complete, check if the desired data path exists." << endl;
		exit(EXIT_FAILURE);
	}

	LoadGtPoses();
	LoadConfig();

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

			first_gt = false;
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

void StreamLoader::LoadConfig() {
	config_fs_ = cv::FileStorage(config_file_.string(), cv::FileStorage::READ);

	config_.cam_intrisics.fx = (float)config_fs_["Camera.fx"];
	config_.cam_intrisics.fy = (float)config_fs_["Camera.fy"];
	config_.cam_intrisics.cx = (float)config_fs_["Camera.cx"];
	config_.cam_intrisics.cy = (float)config_fs_["Camera.cy"];

	config_.cam_distort_param.k1 = (float)config_fs_["Camera.k1"];
	config_.cam_distort_param.k2 = (float)config_fs_["Camera.k2"];
	config_.cam_distort_param.p1 = (float)config_fs_["Camera.p1"];
	config_.cam_distort_param.p2 = (float)config_fs_["Camera.p2"];

	cv::Mat Tlc;
	config_fs_["Tlc"] >> Tlc;

	config_.Tlc = CvMat4ToEigenIso3d(Tlc);

	cout << "[CamvoxLoader] Config:" << endl
		<< "-- Camera instrinsics: (fx, fy, cx ,cy)" << config_.cam_intrisics.fx << " "
		<< config_.cam_intrisics.fy << " "
		<< config_.cam_intrisics.cx << " "
		<< config_.cam_intrisics.cy << endl
		<< "-- Camera distortion params: " << config_.cam_distort_param.k1 << " "
		<< config_.cam_distort_param.k2 << " "
		<< config_.cam_distort_param.p1 << " "
		<< config_.cam_distort_param.p2 << endl
		<< "-- Tlc: " << endl << config_.Tlc.matrix() << endl;
}

Eigen::Isometry3d StreamLoader::CvMat4ToEigenIso3d(const cv::Mat& mat) const {
	assert(mat.cols == 4 && mat.rows == 4);

	Eigen::Matrix4d m;
	for (int col = 0; col < mat.cols; ++col) {
		for (int row = 0; row < mat.rows; ++row) {
			m(col, row) = mat.at<float>(col, row);
		}
	}

	return Eigen::Isometry3d(m);
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
	using dataset_loader::DataType;
	using dataset_loader::ImuData;

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
		d.SetImg(cv::imread(p.string(), cv::IMREAD_UNCHANGED));
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