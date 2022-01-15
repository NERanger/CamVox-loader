#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

#include <pcl/common/transforms.h>

#include <Eigen/Geometry>

#include "utils/utils.hpp"

#include "stream_loader/StreamLoader.hpp"

using std::string;

using dataset_loader::StreamLoader;
using dataset_loader::Data;
using dataset_loader::DataType;

using dataset_loader::LivoxPtcloud2XYZ;

int main(int argc, char const* argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: ./accu_ptcloud <path-to-dataset>" << std::endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);
    StreamLoader dataset(path);

    // Accumulated point cloud
    pcl::PointCloud<pcl::PointXYZ> accu_cloud;

    unsigned int cnt = 0;
    while (true) {
        Data d = dataset.LoadNextData();
        if (d.DType() == DataType::kDTypeLidar) {
            uint64_t timestamp = d.Timestamp();
            Eigen::Isometry3d gt_pose = dataset.GetGtPose(timestamp);

            //std::cout << gt_pose.matrix() << std::endl << std::endl;

            pcl::PointCloud<pcl::PointXYZ> cloud = LivoxPtcloud2XYZ(*d.Ptcloud());

            pcl::transformPointCloud<pcl::PointXYZ>(cloud, cloud, gt_pose.matrix().cast<float>());

            accu_cloud += cloud;

            ++cnt;
            if (cnt > 5000) {
                break;
            }
        }
    }

    pcl::io::savePCDFileBinaryCompressed(path + "/result.pcd", accu_cloud);
    std::cout << "Saved " << accu_cloud.size() << " data points to pcd file." << std::endl;

    return EXIT_SUCCESS;
}
