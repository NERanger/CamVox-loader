#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "stream_loader/LivoxPoint.hpp"

namespace dataset_loader {

	pcl::PointCloud<pcl::PointXYZ> LivoxPtcloud2XYZ(const pcl::PointCloud<LivoxPoint> &lvx_ptcloud);

	void SimpleCloudVisualization(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

}