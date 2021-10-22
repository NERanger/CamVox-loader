#include <opencv2/opencv.hpp>

#include "stream_loader/StreamLoader.hpp"

using std::string;

using camvox_loader::StreamLoader;
using camvox_loader::Data;
using camvox_loader::DataType;

int main(int argc, char const* argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: ./color_ptcloud <path-to-dataset>" << std::endl;
        return EXIT_FAILURE;
    }

    string path(argv[1]);
    StreamLoader dataset(path);



    while (true) {
        Data d = dataset.LoadNextData();
        if (d.DType() == DataType::kDTypeImg) {
            cv::imshow("img", d.Img());
            cv::waitKey(0);
        }
    }

    return EXIT_SUCCESS;
}
