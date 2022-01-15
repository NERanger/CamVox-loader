#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

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

void dataset_loader::SimpleCloudVisualization(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    viewer->initCameraParameters();

    // ID for viewport 1
    int vp_1 = 0;
    viewer->createViewPort(0.0, 0.0, 1.0, 1.0, vp_1);
    viewer->setBackgroundColor(0.0, 0.0, 0.0, vp_1);
    viewer->addText("Original pointcloud", 10, 10, "vp1_text", vp_1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> vp1_color(cloud, "x");
    viewer->addPointCloud<pcl::PointXYZI>(cloud, vp1_color, "origin_cloud", vp_1);

    viewer->addCoordinateSystem(1.0);

    viewer->spin();
}