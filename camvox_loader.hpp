#pragma once

#include <string>
#include <boost/filesystem/path.hpp>

#include <opencv2/opencv.hpp>

namespace camvox_loader{

struct CamvoxFrame{
    cv::Mat rgb_img;
    cv::Mat depth_img;
};

class CamvoxLoader{
public:
    CamvoxLoader(const std::string &data_root, bool &if_success);

    inline size_t Size(){return size_;}

    CamvoxFrame operator[](size_t i) const;

private:
    size_t GetFileNumInDir(const boost::filesystem::path &p) const;

    boost::filesystem::path root_;
    boost::filesystem::path rgb_path_;
    boost::filesystem::path depth_path_;

    size_t size_ = 0;
};

}