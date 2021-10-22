add_executable(accu_ptcloud AccumulatePtcloud.cc)
target_link_libraries(
    accu_ptcloud 
        dataset_loader
)