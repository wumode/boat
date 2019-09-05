aux_source_directory(. DIR_LIB_SRCS)
add_library (SerialCommunication ${DIR_LIB_SRCS})
target_link_libraries(SerialCommunication
        pthread
        serial)