#pragma once

#include <string>
#include <memory>

#include <Eigen/Geometry>

#include <boost/filesystem/path.hpp>

#include <opencv2/opencv.hpp>

#include "csv.h"

namespace dataset_loader {

struct CamvoxFrame{
    cv::Mat rgb_img;
    cv::Mat depth_img;
    Eigen::Isometry3d Twc;
};

struct CameraIntrinsics{
    float fx;
    float fy;
    float cx;
    float cy;
};

class CamvoxLoader{
public:
    using PoseVecType = std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>;
    using PosesVecPtr = std::shared_ptr<PoseVecType>;

    CamvoxLoader(const std::string &data_root, bool &if_success);

    inline size_t Size(){return size_;}
    inline CameraIntrinsics GetCamIntrisics(){return cam_intrinsics_;}
    inline float GetDepthFactor(){return depth_factor_;}
    // inline Eigen::Isometry3d GetTbc(){return Tbc_;}

    void OutPutKittiFormat(std::ofstream &ofstream);

    CamvoxFrame operator[](size_t i) const;

private:
    size_t GetFileNumInDir(const boost::filesystem::path &p) const;

    PosesVecPtr LoadPoseInMemory() const;

    // First, transform poses relative to the given (first) pose. Now we have Twb
    void TransformToCamFrame();
    // Then, get Twc = Twb * Tbc
    void TransformPoseRelativeTo(size_t rel);

    void LoadConfig();

    std::shared_ptr<io::CSVReader<8>> csvreader_ptr_ = nullptr;

    PosesVecPtr gt_pose_ptr_ = nullptr;

    // FileStorage for yaml config file
    cv::FileStorage config_fs_;

    // Path
    boost::filesystem::path root_;
    boost::filesystem::path rgb_path_;
    boost::filesystem::path depth_path_;
    boost::filesystem::path gt_path_;
    boost::filesystem::path config_path_;

    CameraIntrinsics cam_intrinsics_;
    Eigen::Isometry3d Tbc_;  // Transformation from camera to body(IMU)
    float depth_factor_;

    size_t size_ = 0;

    // Expected header for input csv file
    std::string header_seq_str_ = std::string("seq");
    
    std::string header_x_str_ = std::string("position_x");
    std::string header_y_str_ = std::string("position_y");
    std::string header_z_str_ = std::string("position_z");

    std::string header_quat_x_str_ = std::string("quat_x");
    std::string header_quat_y_str_ = std::string("quat_y");
    std::string header_quat_z_str_ = std::string("quat_z");
    std::string header_quat_w_str_ = std::string("quat_w");
};

}