find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(Ceres REQUIRED) 
find_package(Sophus REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
file(GLOB_RECURSE SRC_DIR_LIST ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
file(GLOB_RECURSE HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR} ../include/ )	
set(IPOPT_LIBRARY_DIRS /usr/local/lib)
set(IPOPT_INCLUDE_DIRS /usr/local/include/coin-or)
set(Sophus_INCLUDE_DIR /usr/local/include/sophus )
set(GLOB_RECURSE FRICP_INCLUDE_DIR ../Fast-Robust-ICP/*.h)

#--- Nanoflann
find_package(NanoFlann REQUIRED)
include_directories(${NANOFLANN_INCLUDE_DIR})

include_directories(${IPOPT_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include)
#--- OpenMP (cannot work in clang)
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()
message("ipopt binary path is: " ${IPOPT_LIBRARY_DIRS} )
message("ipopt include path is: " ${IPOPT_INCLUDE_DIRS} )
message("ceres include path is: "  ${Ceres_DIR} )
message("Sophus include path is: "  ${Sophus_INCLUDE_DIR} )
message("PCL include path is: "  ${PCL_INCLUDE_DIRS} )
message("FRICP include path is: "  ${FRICP_INCLUDE_DIR})

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${TBB_INCLUDE_DIRS}
        ${HEADER_DIR}
        ${IPOPT_INCLUDE_DIRS}
        ${Ceres_DIR}
        ${Sophus_DIR}
)
link_directories(${IPOPT_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(test_pcl ${SRC_DIR_LIST}  )


target_link_libraries (test_pcl 
        ${PCL_LIBRARIES} 
        TBB::tbb 
        ipopt
        ${CERES_LIBRARIES}
        Sophus::Sophus
)

install(TARGETS test_pcl RUNTIME DESTINATION bin)
