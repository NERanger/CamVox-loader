# CamVox dataset loader

A CamVox dataset loader for using with [CamVox data extraction](https://github.com/NERanger/CamVox-data-extraction).

## Dependencies

* OpenCV (Tested on OpenCV 3.4)

## Build

```shell
mkdir build
cd build
cmake ..
make
```

## Usage

```cpp
string path(argv[1]);
bool if_success = false;
CamvoxLoader dataset(path, if_success);

if(!if_success){
    std::cerr << "Fail to load dataset" << std::endl;
    return EXIT_FAILURE;
}

for(size_t i = 0; i < dataset.Size(); ++i){
    CamvoxFrame f = dataset[i];
    // Your operation on data
}
```

## Example

An example is given in `example.cc`