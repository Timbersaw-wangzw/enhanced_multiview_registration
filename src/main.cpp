/*
 * @Author: your name
 * @Date: 2022-03-03 14:48:02
 * @LastEditTime: 2022-03-08 16:15:12
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/src/main.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>
#include "../include/robust_registration.h"
#include "../include/data_prepare.h"
#include "../include/obb_box.h"
#include <Eigen/Core>
using namespace std;
using namespace Ipopt;
void drawPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    const char *name,
                    double *color,
                    double size)
{
    viewer->addPointCloud<pcl::PointXYZ>(cloud, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud, color[0], color[1], color[2]), name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
}
void typeTransfer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, Eigen::Matrix3Xd & out)
{
    int n_vertices = in->points.size();
    out.resize(3,n_vertices);
    for (int i=0;i<n_vertices;i++)
    {
        out(0,i)=in->points[i].x;
        out(1,i)=in->points[i].y;
        out(2,i)=in->points[i].z;
    }
}
void typeTransfer(Eigen::Matrix3Xd in, pcl::PointCloud<pcl::PointXYZ>& out)
{
    int n_vertices=in.cols();
    out.width=n_vertices;
    out.height =1;
    for (int i=0;i<n_vertices;i++)
    {
        pcl::PointXYZ p(in(0,i),in(1,i),in(2,i));
        out.points.push_back(p);
    }
}
void getInitialCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./data/01.pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./data/02.pcd", *target_cloud);
    // pcl::io::loadPCDFile<pcl::PointXYZ>("./data/2-filter.pcd", *source_cloud);
    // pcl::io::loadPCDFile<pcl::PointXYZ>("./data/8-filter.pcd", *target_cloud);
    filter_points::ObbBox source_box(source_cloud);
    filter_points::ObbBox target_box(target_cloud);
    source_box.GenerateObbBox();
    target_box.GenerateObbBox();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // filter_points::filterBoxPoints(source_box, target_box, filter_source_cloud, filter_target_cloud);
    source_box.FilterPoints(target_cloud, filter_target_cloud);
    target_box.FilterPoints(source_cloud, filter_source_cloud);
    pcl::PCDWriter writer;
    writer.write("./data/1+(1-2).pcd", *filter_source_cloud);
    writer.write("./data/2+(1-2).pcd", *filter_target_cloud);
    // initial viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    double source_color[3] = {0, 255, 0};
    double target_color[3] = {255, 0, 0};
    source_box.drawObbBox(viewer, source_color, 0.1, 4, "source_box");
    target_box.drawObbBox(viewer, target_color, 0.1, 4, "target_box");

    drawPointCloud(viewer, filter_source_cloud, "filter_source_point", source_color, 5);
    drawPointCloud(viewer, filter_target_cloud, "filter_targert_point", target_color, 5);
    // viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    // int x = b.test(1);

    auto point = source_cloud->points[0];
    viewer->setCameraPosition(point.x, point.y, point.z, 0, 156, -20, 0, 0, 1, 0); //设置相机位置，焦点，方向
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    system("pause");
}

int main(int argc, char **argv)
{
    // transfer txt to pcd
    // scanFiles("./test_points");
    
    // input data
    std::cout<<argv[1]<<std::endl;
    std::cout<<argv[2]<<std::endl;
    typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../../data/1+(1-2).pcd", *source_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("../../data/2+(1-2).pcd", *target_cloud);
    Vertices *vertices_source=new Vertices;
    Vertices *vertics_target=new Vertices;
    typeTransfer(source_cloud,*vertices_source);
    typeTransfer(target_cloud,*vertics_target);
    // pairwise icp
    //robustOptimize::robustFunc fcn(1, robustOptimize::cauchy);
    //robustOptimize::pairwiseICP::registrationLoop icp(source_cloud, target_cloud, fcn, 50, 1e-4, robustOptimize::pairwiseICP::manual);
    //icp.optimize();
    robustOptimize::pairwiseICP::registrationICP icp(vertices_source,vertics_target,robustOptimize::pairwiseICP::RICP);
    icp.align();
    typeTransfer(*vertices_source,*transformed_cloud);
    std::cout<<"final transformation is "<<std::endl;
    std::cout<<icp.res_trans<<std::endl;
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
            viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(1, 1, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    double source_color[3] = {0, 0, 255};
    double target_color[3] = {255, 0, 0};
    double transfer_color[3] = {0, 255, 0};
    
    drawPointCloud(viewer, source_cloud, "filter_source_point", source_color, 5);
    drawPointCloud(viewer, target_cloud, "filter_targert_point", target_color, 5);
    drawPointCloud(viewer, transformed_cloud, "filter_transfer_point", transfer_color, 5);
    // viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    // int x = b.test(1);

    auto point = source_cloud->points[0];
    viewer->setCameraPosition(point.x, point.y, point.z, 0, 156, -20, 0, 0, 1, 0); //设置相机位置，焦点，方向
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    system("pause");
    return 0;
}
