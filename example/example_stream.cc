#include <opencv2/opencv.hpp>

#include "stream_loader/StreamLoader.hpp"

using std::string;

using dataset_loader::StreamLoader;
using dataset_loader::Data;
using dataset_loader::DataType;

int main(int argc, char const* argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: ./load_camvox <path-to-dataset>" << std::endl;
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
