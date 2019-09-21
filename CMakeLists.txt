cmake_minimum_required(VERSION 2.8.11)

project(carto_server)

SET(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_DEBUG  "-O3" )
#set(CMAKE_CXX_FLAGS_DEBUG)
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules/")
#set(CMAKE_AUTOMOC ON) # For meta object compiler
#set(CMAKE_AUTORCC ON) # Resource files
#set(CMAKE_AUTOUIC ON) # UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#find_package(Qt5 REQUIRED Widgets)
#find_package(VTK REQUIRED)
#find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 find_package(Boost REQUIRED COMPONENTS thread)
 find_package(Ceres  REQUIRED COMPONENTS SuiteSparse)
 find_package(Eigen3  REQUIRED)
find_package(Abseil REQUIRED)
find_package(Lua REQUIRED)
#set(CMAKE_MODULE_PATH "/usr/local/lib/cmake/protobuf/;${CMAKE_MODULE_PATH}")
find_package(Protobuf REQUIRED)
message(STATUS "Using protobuf ${Protobuf_VERSION}")
message(STATUS "protobuf_dir: ${PROTOBUF_INCLUDE_DIR}")
message(STATUS "protobuf_exe: ${Protobuf_PROTOC_EXECUTABLE}")
find_package(async_grpc CONFIG REQUIRED)
 
find_package(roscpp REQUIRED)
find_package(rosconsole REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf REQUIRED)

set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
 find_package(gRPC REQUIRED)
 message(STATUS "Using gRPC ${gRPC_VERSION}")

 set(_GRPC_GRPCPP_UNSECURE gRPC::grpc++_unsecure)
 set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)

 get_filename_component(hw_proto "locationtest.proto" ABSOLUTE)
 get_filename_component(hw_proto_path "${hw_proto}" PATH)

 # Generated sources
 #set(hw_proto_srcs "${CMAKE_CURRENT_SOURCE_DIR}/locationtest.pb.cc")
 #set(hw_proto_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/locationtest.pb.h")
 #set(hw_grpc_srcs "${CMAKE_CURRENT_SOURCE_DIR}/locationtest.grpc.pb.cc")
 #set(hw_grpc_hdrs "${CMAKE_CURRENT_SOURCE_DIR}/locationtest.grpc.pb.h")
 #add_custom_command(
 #      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
 #     COMMAND ${_PROTOBUF_PROTOC}
 #      ARGS --grpc_out "${CMAKE_CURRENT_SOURCE_DIR}"
 #       --cpp_out "${CMAKE_CURRENT_SOURCE_DIR}"
 #        -I "${hw_proto_path}"
 #        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
 #        "${hw_proto}"
 #      DEPENDS "${hw_proto}")
 file(GLOB_RECURSE ALL_PROTOS "cartographer/*.proto")
 message("testing:")
 set(Protobuf_PROTOC_EXECUTABLE /usr/local/bin/protoc)
 set(PROTOBUF_LIBRARY /usr/local/lib/libprotobuf.a)
 set(ALL_PROTO_SRCS)
 set(ALL_PROTO_HDRS)
 include_directories("${CMAKE_CURRENT_SOURCE_DIR}")
   #*********************protobuf *********************
   foreach(ABS_FIL ${ALL_PROTOS})
       file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
       get_filename_component(DIR ${REL_FIL} DIRECTORY)
       get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

       list(APPEND ALL_PROTO_SRCS "${PROJECT_SOURCE_DIR}/${DIR}/${FIL_WE}.pb.cc")
       list(APPEND ALL_PROTO_HDRS "${PROJECT_SOURCE_DIR}/${DIR}/${FIL_WE}.pb.h")
      # message(${ABS_FIL})

       add_custom_command(
         OUTPUT "${PROJECT_SOURCE_DIR}/${DIR}/${FIL_WE}.pb.cc"
                "${PROJECT_SOURCE_DIR}/${DIR}/${FIL_WE}.pb.h"
         COMMAND  ${Protobuf_PROTOC_EXECUTABLE}
         ARGS --cpp_out  ${PROJECT_SOURCE_DIR} -I
           ${PROJECT_SOURCE_DIR} ${ABS_FIL}
         DEPENDS ${ABS_FIL}
         COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
         VERBATIM
       )
    endforeach()
   set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)

   # Targets greeter_[async_](client|server)
#   foreach(_target
#     greeter_client greeter_server
#     greeter_async_client greeter_async_server)
#     add_executable(${_target} "${_target}.cc"
#       ${hw_proto_srcs}
#       ${hw_grpc_srcs})
#     target_link_libraries(${_target}
#       ${_GRPC_GRPCPP_UNSECURE}
#       ${_PROTOBUF_LIBPROTOBUF})
#   endforeach()
link_directories("/usr/local/lib")
link_directories("/usr/lib/x86_64-linux-gnu")
include_directories(${PROJECT_NAME} "/opt/ros/melodic/include")
link_directories("/opt/ros/melodic/lib")
include_directories(${PROJECT_NAME} "${EIGEN3_INCLUDE_DIR}")
include_directories(${PROJECT_NAME} "${LUA_INCLUDE_DIR}")
include_directories(${PROJECT_NAME}  "${Boost_INCLUDE_DIRS}")
include_directories(${PROJECT_NAME} "${PROJECT_SOURCE_DIR}")

file(GLOB_RECURSE ALL_LIBRARY_HDRS "cartographer/*.h" "drives/*.h")
file(GLOB_RECURSE ALL_LIBRARY_SRCS "cartographer/*.cc" "drives/*.cpp" )
file(GLOB common "cartographer/common/*.cc" "cartographer/common/*.h")
file(GLOB mapping "cartographer/mapping/*.cc" "cartographer/mapping/*.h")
file(GLOB scan "cartographer/mapping/scan_mathing/*.cc" "cartographer/mapping/scan_mathing/*.h")
file(GLOB sensor "cartographer/sensor/*.cc" "cartographer/sensor/*.h")
file(GLOB tranform "cartographer/transform/*.cc" "cartographer/transform/*.h")
file(GLOB location "cartographer/location/*.cc" "cartographer/location/*.h")
file(GLOB drive "drives/*.cpp" "drives/*.h")
set(carto_src ${ALL_LIBRARY_SRCS})  #add src files
set(carto_include ${ALL_LIBRARY_HDRS}) #add include files
#add_executable(${PROJECT_NAME} "main.cpp" ${carto_src} ${carto_include} ${hw_proto_srcs} ${hw_grpc_srcs} ${ALL_PROTO_SRCS})
add_executable(${PROJECT_NAME} "main_server.cpp" ${carto_src} ${carto_include} ${hw_proto_srcs} ${hw_grpc_srcs} ${ALL_PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} SDL SDL_image)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${LUA_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})
target_link_libraries(${PROJECT_NAME} protobuf)
target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARY})
target_link_libraries(${PROJECT_NAME}
  ${_GRPC_GRPCPP_UNSECURE}
  ${_PROTOBUF_LIBPROTOBUF})
target_link_libraries(${PROJECT_NAME} glog)
target_link_libraries(${PROJECT_NAME} gflags)
target_link_libraries(${PROJECT_NAME}  grpc++)
 target_link_libraries(${PROJECT_NAME} async_grpc)
#target_link_libraries(${PROJECT_NAME} ${PROTOBUF_LIBRARY} pthread standalone_absl)

target_link_libraries(${PROJECT_NAME} pthread standalone_absl)
#target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} Qt5::Widgets ${VTK_LIBRARIES})

target_link_libraries(${PROJECT_NAME} ${PTHREADS_LIBRARY})

target_link_libraries(${PROJECT_NAME} ${roscpp_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${rosconsole_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${sensor_msgs_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${std_msgs_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${tf_LIBRARIES})
#target_link_libraries(${PROJECT_NAME} unofficial::abseil::hash)
#target_link_libraries(${PROJECT_NAME} ${UNOFICIAL_abseil_LIBRARY})


