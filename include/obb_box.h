/*
 * @Author: your name
 * @Date: 2022-03-03 16:11:53
 * @LastEditTime: 2022-03-07 14:49:15
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/obb_box.h
 */
#pragma once

#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

typedef Eigen::Matrix<float, 3, 1> vector3;
typedef Eigen::Matrix<float, 4, 1> vector4;
typedef Eigen::Matrix<float, 3, 3> matrix3;
typedef Eigen::Matrix<float, 4, 4> matrix4;
namespace filter_points
{
        class ObbBox
        {
        public:
                ObbBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
                {
                        this->cloud = cloud;
                };
                void GenerateObbBox();
                bool isInBox(pcl::PointXYZ pt);
                void DividedBox(float f0, float f1, int side, ObbBox &b);
                void FilterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
                matrix4 box_axis;
                float length, width, height;
                void drawObbBox(
                    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                    double *color,
                    double transparent,
                    double line_width,
                    const char *name);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        private:
                pcl::PointXYZ min_point_OBB;
                pcl::PointXYZ max_point_OBB;
                pcl::PointXYZ position_OBB;
                Eigen::Vector3f mass_center;
                Eigen::Matrix3f rotational_matrix_OBB;
        };
        inline bool filterBoxPoints(ObbBox b1, ObbBox b2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
        {

                for (auto point : b1.cloud->points)
                {
                        if (b2.isInBox(point))
                        {
                                cloud1->push_back(point);
                        }
                }

                for (auto point : b2.cloud->points)
                {
                        if (b1.isInBox(point))
                        {
                                cloud2->push_back(point);
                        }
                }
        };
};
// namespace filter_points
// {
//         template <class T>
//         class ObbBox
//         {
//         public:
//                 typedef Eigen::Matrix<T, 3, 1> vector3;
//                 typedef Eigen::Matrix<T, 4, 1> vector4;
//                 typedef Eigen::Matrix<T, 3, 3> matrix3;
//                 typedef Eigen::Matrix<T, 4, 4> matrix4;
//                 ObbBox(){};
//                 ~ObbBox(){};
//                 ObbBox(const std::vector<vector3> &points)
//                 {
//                         this->points = points;
//                 };
//                 ObbBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
//                 {
//                         this->cloud = cloud;
//                         type = 0;
//                 };
//                 void test();
//                 void GenerateObbBox();
//                 bool isInBox(const vector3 &pt);
//                 void DividedBox(T f0, T f1, int side, ObbBox &b);
//                 void FilterPoints(const std::vector<vector3> &in_cloud, std::vector<vector3> &out_cloud);
//                 void ClearAABBBox();

//                 matrix4 box_axis;
//                 T length, width, height;

//         private:
//                 void pca(matrix3 &pca_matrix);
//                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//                 std::vector<vector3> points;
//                 void orthonormalize(matrix3 &ColVecs);
//                 int type = 1;
//         };
//         template <class T>
//         bool filter_box_points(ObbBox<T> b1, ObbBox<T> b2){

//         };
// }
