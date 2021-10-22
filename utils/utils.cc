#include "utils/utils.hpp"

pcl::PointCloud<pcl::PointXYZ> dataset_loader::LivoxPtcloud2XYZ(const pcl::PointCloud<LivoxPoint> &lvx_ptcloud) {
	using pcl::PointCloud;
	using pcl::PointXYZ;

	PointCloud<PointXYZ> cloud_xyz;
	cloud_xyz.resize(lvx_ptcloud.size());
	for (size_t i = 0; i < lvx_ptcloud.size(); ++i) {
		cloud_xyz.points[i].x = lvx_ptcloud.points[i].x;
		cloud_xyz.points[i].y = lvx_ptcloud.points[i].y;
		cloud_xyz.points[i].z = lvx_ptcloud.points[i].z;
	}

	return cloud_xyz;
}