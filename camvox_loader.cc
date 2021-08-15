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

    cout << "[CamvoxLoader] Desired data path:" << endl
         << "-- RGB image: " << rgb_path_.string() << endl
         << "-- Depth image: " << depth_path_.string() << endl;

    if(!(exists(rgb_path_) && exists(depth_path_))){
        cerr << "[CamvoxLoader] Dataset not complete, check if the desired data path exists." << endl;
        if_success = false;
    }

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

    return f;
}