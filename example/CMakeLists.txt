add_executable(load_camvox example.cc)
target_link_libraries(
    load_camvox 
        dataset_loader
)

add_executable(load_stream example_stream.cc)
target_link_libraries(
    load_stream 
        dataset_loader
)