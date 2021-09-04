#include <opencv2/opencv.hpp>

#include "camvox_loader.hpp"

using std::string;

using camvox_loader::CamvoxLoader;
using camvox_loader::CamvoxFrame;

int main(int argc, char const *argv[]){
    
    if(argc != 2){
        std::cerr << "Usage: ./load_camvox <path-to-dataset>" << std::endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);
    bool if_success = false;
    CamvoxLoader dataset(path, if_success);

    if(!if_success){
        std::cerr << "Fail to load dataset" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Camera intrinsics: " << std::endl
              << "Fx: " << dataset.GetCamIntrisics().fx << std::endl
              << "Fy: " << dataset.GetCamIntrisics().fy << std::endl
              << "Cx: " << dataset.GetCamIntrisics().cx << std::endl
              << "Cy: " << dataset.GetCamIntrisics().cy << std::endl;

    std::cout << "Depth Factor: " << dataset.GetDepthFactor() << std::endl;

    for(size_t i = 0; i < dataset.Size(); ++i){
        CamvoxFrame f = dataset[i];

        cv::cvtColor(f.rgb_img, f.rgb_img, cv::COLOR_RGB2BGR);

        cv::resize(f.rgb_img, f.rgb_img, cv::Size(), 0.5, 0.5);
        cv::resize(f.depth_img, f.depth_img, cv::Size(), 0.5, 0.5);

        cv::imshow("rgb", f.rgb_img);
        cv::imshow("depth", f.depth_img);

        std::cout << f.Twc.matrix() << std::endl;

        cv::waitKey(1);
    }
    
    return EXIT_SUCCESS;
}
