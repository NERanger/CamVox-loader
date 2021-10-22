#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <Eigen/Geometry>

#include "camvox_loader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using dataset_loader::CamvoxLoader;
using dataset_loader::CamvoxFrame;

CamvoxLoader::CamvoxLoader(const std::string &data_root, bool &if_success) : root_(data_root){
    using boost::filesystem::exists;

    rgb_path_ = root_ / ("color_img");
    depth_path_ = root_ / ("depth_img");
    gt_path_ = root_ / ("groundtruth.csv");
    config_path_ = root_ / ("config.yaml");

    cout << "[CamvoxLoader] Expected data path:" << endl
         << "-- RGB image: " << rgb_path_.string() << endl
         << "-- Depth image: " << depth_path_.string() << endl
         << "-- Groundtruth: " << gt_path_.string() << endl;

    // Check egb and depth image
    if(!(exists(rgb_path_) && exists(depth_path_))){
        cerr << "[CamvoxLoader] Dataset not complete, check if the desired data path exists." << endl;
        if_success = false;
    }

    // Check & load config file
    if(exists(config_path_)){
        config_fs_ = cv::FileStorage(config_path_.string(), cv::FileStorage::READ);
        LoadConfig();
    }else{
        cerr << "[CamvoxLoader] No configuration file provided" << endl;
        if_success = false;
    }
    
    // Check & load groundtruth
    if(exists(gt_path_)){
        csvreader_ptr_ = std::make_shared<io::CSVReader<8>>(gt_path_.string());
        gt_pose_ptr_ = LoadPoseInMemory();
        TransformToCamFrame();
        // TransformPoseRelativeTo(0);
    }else{
        cerr << "[CamvoxLoader] No groundtruth file provided" << endl;
        if_success = false;
    }

    // Get total number
    size_t rgb_img_num = GetFileNumInDir(rgb_path_);
    size_t depth_img_num = GetFileNumInDir(depth_path_);

    if (! (rgb_img_num == depth_img_num)){
        cerr << "[CamvoxLoader] Dataset not complete, frame number of "
             << "RGB image and depth image not equal" << endl; 
        if_success = false;
    }
    
    size_ = rgb_img_num;
    if_success = true;
}

// Reference: https://stackoverflow.com/questions/41304891/how-to-count-the-number-of-files-in-a-directory-using-standard
size_t CamvoxLoader::GetFileNumInDir(const boost::filesystem::path &p) const{
    using boost::filesystem::directory_iterator;
    return std::distance(directory_iterator(p), directory_iterator{});
}

CamvoxFrame CamvoxLoader::operator[](size_t i) const{
    using boost::format;
    format fmt("%s/%06d.png");

    string rgb_img_path = (fmt % rgb_path_.string() % i).str();
    string depth_img_path = (fmt % depth_path_.string() % i).str();

    CamvoxFrame f;
    f.rgb_img = cv::imread(rgb_img_path);
    f.depth_img = cv::imread(depth_img_path);
    f.Twc = gt_pose_ptr_->at(i);

    return f;
}

void CamvoxLoader::TransformToCamFrame(){
    // Toc: From camera to world (first frame)
    PosesVecPtr Twc_poses = std::make_shared<PoseVecType>();

    // Currently gt_pose_ptr_ holds all Tob (body to origin)
    // Twc = Twb * Tbc
    for(size_t i = 0; i < gt_pose_ptr_->size(); ++i){
        Eigen::Isometry3d Twc = gt_pose_ptr_->at(i) * Tbc_;
        Twc_poses->emplace_back(Twc);
    }

    gt_pose_ptr_ = Twc_poses;
}

void CamvoxLoader::TransformPoseRelativeTo(size_t rel){
    // Twc: "w" means the given pose (usually first pose in the sequence)
    PosesVecPtr Twc_poses = std::make_shared<PoseVecType>();

    Eigen::Isometry3d w_inv = gt_pose_ptr_->at(rel).inverse();

    for(size_t i = 0; i < gt_pose_ptr_->size(); ++i){
        Eigen::Isometry3d Twc = w_inv * gt_pose_ptr_->at(i);
        Twc_poses->emplace_back(Twc);
    }

    gt_pose_ptr_ = Twc_poses;
}

CamvoxLoader::PosesVecPtr CamvoxLoader::LoadPoseInMemory() const{
    if(csvreader_ptr_ == nullptr){
        return nullptr;
    }

    csvreader_ptr_->read_header(io::ignore_extra_column, header_seq_str_, 
                                header_x_str_, header_y_str_, header_z_str_, 
                                header_quat_x_str_, header_quat_y_str_, header_quat_z_str_, header_quat_w_str_);

    int seq;
    double position_x, position_y, position_z;
    double quat_x, quat_y, quat_z, quat_w;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> trans_vec;
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> rot_vec;

    // Populate trans and quat vector
    while(csvreader_ptr_->read_row(seq, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w)){
        trans_vec.emplace_back(position_x, position_y, position_z);
        rot_vec.emplace_back(Eigen::Quaterniond(quat_w, quat_x, quat_y, quat_z).toRotationMatrix());
    }

    // Transform translation and quat to first frame
    Eigen::Vector3d trans_origin = trans_vec[0];
    Eigen::Matrix3d rot_origin = rot_vec[0];
    for(size_t i = 0; i < trans_vec.size(); ++i){
        trans_vec[i] -= trans_origin;
    }
    // o - origin: origin of UTM coordinate
    // w - world: first body frame
    // b - body: body(RTK) frame
    // Rob = Row * Rwb
    // Rwb = Row^T * Rob
    for(size_t i = 0; i < rot_vec.size(); ++i){
        rot_vec[i] = rot_origin.transpose() * rot_vec[i];
    }

    // std::cout << rot_vec[0] << std::endl;

    PosesVecPtr poses_ptr = std::make_shared<PoseVecType>();
    for(size_t i = 0; i < trans_vec.size(); ++i){
        Eigen::Isometry3d Twb(rot_vec[i]);
        Twb.pretranslate(trans_vec[i]);
        poses_ptr->emplace_back(Twb);
    }

    return poses_ptr;

}

void CamvoxLoader::LoadConfig(){
    cam_intrinsics_.fx = (float)config_fs_["Camera.fx"];
    cam_intrinsics_.fy = (float)config_fs_["Camera.fy"];
    cam_intrinsics_.cx = (float)config_fs_["Camera.cx"];
    cam_intrinsics_.cy = (float)config_fs_["Camera.cy"];

    depth_factor_ = (float)config_fs_["DepthMapFactor"];

    cv::Mat Tbc;
    config_fs_["Tbc"] >> Tbc;

    Eigen::Matrix4d m;

    for(int col = 0; col < Tbc.cols; ++col){
        for(int row = 0; row < Tbc.rows; ++row){
            m(col, row) =  Tbc.at<float>(col, row);
        }
    }

    // std::cout << m.matrix() << std::endl;
    Tbc_ = Eigen::Isometry3d(m);
    
}

void CamvoxLoader::OutPutKittiFormat(std::ofstream &ofstream){
    for(auto p : *gt_pose_ptr_){
        ofstream << p(0, 0) << " " << p(0, 1) << " " << p(0, 2) << " " << p(0, 3) << " "
                 << p(1, 0) << " " << p(1, 1) << " " << p(1, 2) << " " << p(1, 3) << " "
                 << p(2, 0) << " " << p(2, 1) << " " << p(2, 2) << " " << p(2, 3) << std::endl;
    }   
}