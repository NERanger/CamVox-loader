#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "stream_loader/LivoxPoint.hpp"

namespace dataset_loader {

enum DataType {
	kDTypeLidar = 0,
	kDTypeImg,
	kDTypeImu
};

struct ImuData {
	float a_x;
	float a_y;
	float a_z;
	float w_x;
	float w_y;
	float w_z;
};

class Data {
public:
	Data(uint64_t timestamp) : timestamp_(timestamp) {}
	Data(const DataType d_type) : d_type_(d_type) {}

	inline cv::Mat Img() const { return img_; }
	inline pcl::PointCloud<LivoxPoint>::Ptr Ptcloud() const { return ptcloud_ptr_; }
	inline ImuData Imu() const { return imu_; }
	inline uint64_t Timestamp() const { return timestamp_; }

	inline DataType DType() const { return d_type_; }

private:

	inline void SetImg(const cv::Mat& img) { img_ = img; }
	inline void SetPtcloud(const pcl::PointCloud<LivoxPoint>::Ptr ptcloud_ptr) { ptcloud_ptr_ = ptcloud_ptr; }
	inline void SetImu(const ImuData& imu) { imu_ = imu; }
	inline void SetDataType(const DataType& d_type) { d_type_ = d_type; }

	DataType d_type_;

	uint64_t timestamp_;

	cv::Mat img_;
	pcl::PointCloud<LivoxPoint>::Ptr ptcloud_ptr_ = nullptr;
	ImuData imu_;

	friend class StreamLoader;
};

}