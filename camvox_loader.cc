#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include "camvox_loader.hpp"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using camvox_loader::CamvoxLoader;
using camvox_loader::CamvoxFrame;

CamvoxLoader::CamvoxLoader(const std::string &data_root, bool &if_success) : root_(data_root){
    using boost::filesystem::exists;

    rgb_path_ = root_ / ("color_img");
    depth_path_ = root_ / ("depth_img");
    gt_path_ = root_ / ("groundtruth.csv");

    cout << "[CamvoxLoader] Expected data path:" << endl
         << "-- RGB image: " << rgb_path_.string() << endl
         << "-- Depth image: " << depth_path_.string() << endl
         << "-- Groundtruth: " << gt_path_.string() << endl;

    if(!(exists(rgb_path_) && exists(depth_path_))){
        cerr << "[CamvoxLoader] Dataset not complete, check if the desired data path exists." << endl;
        if_success = false;
    }

    size_t rgb_img_num = GetFileNumInDir(rgb_path_);
    size_t depth_img_num = GetFileNumInDir(depth_path_);

    if(exists(gt_path_)){
        csvreader_ptr_ = std::make_shared<io::CSVReader<8>>(gt_path_.string());
        gt_pose_ptr_ = LoadPoseInMemory();
        gt_pose_ptr_ = TransformPoseRelativeTo(gt_pose_ptr_, 0);
    }else{
        // No groundtruth is not considered as failure
        cerr << "[CamvoxLoader] No groundtruth file provided" << endl;
    }

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
    f.pose = gt_pose_ptr_->at(i);

    return f;
}

CamvoxLoader::PosesVecPtr CamvoxLoader::TransformPoseRelativeTo(const CamvoxLoader::PosesVecPtr poses_ptr, size_t rel){
    PosesVecPtr transed_poses = std::make_shared<PoseVecType>();

    for(size_t i = 0; i < poses_ptr->size(); ++i){
        Eigen::Isometry3d transed = poses_ptr->at(rel).inverse() * poses_ptr->at(i);
        transed_poses->emplace_back(transed);
    }

    return transed_poses;
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

    PosesVecPtr poses_ptr = std::make_shared<PoseVecType>();
    while(csvreader_ptr_->read_row(seq, position_x, position_y, position_z, quat_x, quat_y, quat_z, quat_w)){
        Eigen::Vector3d trans(position_x, position_y, position_z);
        Eigen::Quaterniond quat(quat_w, quat_x, quat_y, quat_z);

        Eigen::Isometry3d iso(Eigen::Isometry3d::Identity());
        iso.rotate(quat);
        iso.pretranslate(trans);

        poses_ptr->emplace_back(iso);
    }

    return poses_ptr;

}