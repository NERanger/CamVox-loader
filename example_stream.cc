#include <opencv2/opencv.hpp>

#include "stream_loader/stream_loader.hpp"

using std::string;

using camvox_loader::StreamLoader;
using camvox_loader::Data;
using camvox_loader::DataType;

int main(int argc, char const* argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: ./load_camvox <path-to-dataset>" << std::endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);
    StreamLoader dataset(path);

    /*std::cout << "Camera intrinsics: " << std::endl
        << "Fx: " << dataset.GetCamIntrisics().fx << std::endl
        << "Fy: " << dataset.GetCamIntrisics().fy << std::endl
        << "Cx: " << dataset.GetCamIntrisics().cx << std::endl
        << "Cy: " << dataset.GetCamIntrisics().cy << std::endl;*/

    while (true) {
        Data d = dataset.LoadNextData();
        if (d.DType() == DataType::kDTypeImg) {
            cv::imshow("img", d.Img());
            cv::waitKey(0);
        }
    }

    return EXIT_SUCCESS;
}
