add_subdirectory(point)
aux_source_directory(. DIR_LIB_SRCS)
add_library (Navigation ${DIR_LIB_SRCS})
target_link_libraries(Navigation Point
        datum
        utm)