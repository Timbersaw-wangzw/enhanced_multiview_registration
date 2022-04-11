#include "../include/obb_box.h"

#include <algorithm>
using namespace filter_points;
void ObbBox::drawObbBox(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                        double *color,
                        double transparent,
                        double line_width,
                        const char *name)
{
        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        Eigen::Quaternionf quat(rotational_matrix_OBB);
        viewer->addCube(position, quat,
                        max_point_OBB.x - min_point_OBB.x,
                        max_point_OBB.y - min_point_OBB.y,
                        max_point_OBB.z - min_point_OBB.z, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                            color[0], color[1], color[2], name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
                                            transparent, name);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                            line_width, name);
        Eigen::Vector4f local_x_axis(width / 2, 0, 0, 1);
        Eigen::Vector4f global_x_axis = box_axis * local_x_axis;
        pcl::PointXYZ x_axis(global_x_axis(0), global_x_axis(1), global_x_axis(2));
        std::string str_name(name);
        viewer->addLine(position_OBB, x_axis, 1.0f, 0.0f, 0.0f, "x axis" + str_name);

        Eigen::Vector4f local_y_axis(0, length / 2, 0, 1);
        Eigen::Vector4f global_y_axis = box_axis * local_y_axis;
        pcl::PointXYZ y_axis(global_y_axis(0), global_y_axis(1), global_y_axis(2));
        viewer->addLine(position_OBB, y_axis, 0.0f, 1.0f, 0.0f, "y axis" + str_name);

        Eigen::Vector4f local_z_axis(0, 0, height / 2, 1);
        Eigen::Vector4f global_z_axis = box_axis * local_z_axis;
        pcl::PointXYZ z_axis(global_z_axis(0), global_z_axis(1), global_z_axis(2));
        viewer->addLine(position_OBB, z_axis, 0.0f, 0.0f, 1.0f, "z axis" + str_name);
}
void ObbBox::GenerateObbBox()
{
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(cloud);
        feature_extractor.compute();
        std::vector<float> moment_of_inertia;
        std::vector<float> eccentricity;

        float major_value, middle_value, minor_value;
        Eigen::Vector3f major_vector, middle_vector, minor_vector;

        feature_extractor.getMomentOfInertia(moment_of_inertia);
        feature_extractor.getEccentricity(eccentricity);
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        feature_extractor.getEigenValues(major_value, middle_value, minor_value);
        feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
        feature_extractor.getMassCenter(mass_center);

        width = abs(max_point_OBB.x - min_point_OBB.x);
        length = abs(max_point_OBB.y - min_point_OBB.y);
        height = abs(max_point_OBB.z - min_point_OBB.z);

        Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
        box_axis = matrix4::Identity();
        box_axis.block(0, 0, 3, 3) = rotational_matrix_OBB;
        box_axis.block(0, 3, 3, 1) = position;
        // vector3 mean(0, 0, 0);
        // int s = int(cloud->points.size());
        // float ds = 1.0 / s;
        // for (int i = 0; i < s; i++)
        // {
        //         vector3 pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        //         mean += (pt * ds);
        // }

        // Eigen::MatrixXf Q(3, s);

        // for (unsigned i = 0; i < s; i++)
        // {
        //         vector3 pt(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        //         Q.col(i) = (pt - mean);
        // }

        // matrix3 C = Q * Q.transpose();

        // Eigen::SelfAdjointEigenSolver<matrix3> eigensolver(C);
        // if (eigensolver.info() != Eigen::Success)
        //         throw std::runtime_error("[Math::pca] Can not find eigenvalues.\n");
        // matrix3 eigenvectors;
        // vector3 eigenvalues;

        // // reverse ordering of eigenvalues and eigenvectors
        // for (int i = 0; i < 3; ++i)
        // {
        //         eigenvalues(i) = eigensolver.eigenvalues()(2 - i);
        //         if (i == 2)
        //                 eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
        //         else
        //                 eigenvectors.col(i) = eigensolver.eigenvectors().col(2 - i);
        //         eigenvectors.col(i) = eigenvectors.col(i) / eigenvectors.col(i).norm();
        // }

        // std::cout << eigenvectors << std::endl;
        // std::cout << "pca_matrix:" << std::endl;

        // matrix3 pca_matrix = eigensolver.eigenvectors();
        // std::cout << pca_matrix << std::endl;

        // std::cout << "rotation" << std::endl;
        // std::cout << rotational_matrix_OBB << std::endl;
        // box_axis = matrix4::Identity();
        // box_axis.block(0, 0, 3, 3) = rotational_matrix_OBB;
        // box_axis.block(0, 3, 3, 1) = mean;
        // matrix4 v = box_axis.inverse();
        // std::vector<float> x, y, z;
        // for (unsigned i = 0; i < s; i++)
        // {
        //         vector4 p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1);
        //         vector4 local_Coord = v * p;
        //         x.push_back(local_Coord(0));
        //         y.push_back(local_Coord(1));
        //         z.push_back(local_Coord(2));
        // }
        // float xmin = *std::min_element(x.begin(), x.end());
        // float xmax = *std::max_element(x.begin(), x.end());
        // float ymin = *std::min_element(y.begin(), y.end());
        // float ymax = *std::max_element(y.begin(), y.end());

        // float zmin = *std::min_element(z.begin(), z.end());
        // float zmax = *std::max_element(z.begin(), z.end());

        // width = abs(xmax - xmin);
        // length = abs(ymax - ymin);
        // height = abs(zmax - zmin);
        // vector4 temp1(xmax, ymin, zmin, 1);
        // vector4 origin1 = box_axis * temp1;
        // box_axis.col(3) = origin1;

        // if (type == 1)
        // {
        //
        // }
        // else // pcl obb
        // {

        // }
}

bool ObbBox::isInBox(pcl::PointXYZ pt)
{
        vector4 p(pt.x, pt.y, pt.z, 1);
        vector4 local = box_axis.inverse() * p;

        float x = local(0);
        float y = local(1);
        float z = local(2);
        if (x > width / 2 || x < -1 * width / 2 || y < -1 * length / 2 || y > length / 2 || z < -1 * height / 2 || z > height)
        {
                return false;
        }
        else
        {
                return true;
        }
}

void ObbBox::DividedBox(float f0, float f1, int side, ObbBox &b)
{
        b = *this;
        vector4 origin;
        if (side == 0)
        {
                vector4 new_origin(-1 * width * f0, 0, 0, 1);
                b.width = width * (f1 - f0);
                origin = new_origin;
        }
        else if (side == 1)
        {
                vector4 new_origin(0, length * f0, 0, 1);
                b.length = length * (f1 - f0);
                origin = new_origin;
        }
        else
        {
                vector4 new_origin(0, 0, height * f0, 1);
                b.height = height * (f1 - f0);
                origin = new_origin;
        }
        origin = b.box_axis * origin;
        b.box_axis.col(3) = origin;
}

void ObbBox::FilterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud)
{
        for (auto pt : in_cloud->points)
        {
                if (isInBox(pt))
                {
                        out_cloud->push_back(pt);
                }
        }
}

// template <typename T>
// void ObbBox<T>::ClearAABBBox()
// {
//         points.clear();
// }
// // template <typename T>
// void ObbBox::pca(matrix3 &pca_matrix)
// {
//         vector3 mean(0, 0, 0);
//         unsigned s = unsigned(points.size());
//         float ds = 1.0 / s;

//         for (unsigned i = 0; i < s; i++)
//                 mean += (points[i] * ds);

//         Eigen::MatrixXd Q(3, s);

//         for (unsigned i = 0; i < s; i++)
//                 Q.col(i) = (points[i] - mean);

//         matrix3 C = Q * Q.transpose();

//         Eigen::SelfAdjointEigenSolver<matrix3> eigensolver(C);
//         if (eigensolver.info() != Eigen::Success)
//                 throw std::runtime_error("[Math::pca] Can not find eigenvalues.\n");
//         matrix3 eigenvectors;
//         vector3 eigenvalues;

//         // reverse ordering of eigenvalues and eigenvectors
//         for (int i = 0; i < 3; ++i)
//         {
//                 eigenvalues(i) = eigensolver.eigenvalues()(2 - i);
//                 if (i == 2)
//                         eigenvectors.col(2) = eigenvectors.col(0).cross(eigenvectors.col(1));
//                 else
//                         eigenvectors.col(i) = eigensolver.eigenvectors().col(2 - i);
//                 eigenvectors.col(i) = eigenvectors.col(i) / eigenvectors.col(i).norm();
//         }

//         // float x1 = eigenvectors.col(0).dot(eigenvectors.col(1));
//         // float x2 = eigenvectors.col(0).dot(eigenvectors.col(2));
//         // float x3 = eigenvectors.col(1).dot(eigenvectors.col(2));
//         pca_matrix = eigensolver.eigenvectors();
//         orthonormalize(pca_matrix);
//         // float y1 = test.col(0).dot(test.col(1));
//         // float y2 = test.col(0).dot(test.col(2));
//         // float y3 = test.col(1).dot(test.col(2));
// }
// template <typename T>
// void ObbBox<T>::orthonormalize(matrix3 &ColVecs)
// {
//         ColVecs.col(0).normalize();
//         float temp;
//         for (std::size_t k = 0; k != ColVecs.cols() - 1; ++k)
//         {
//                 for (std::size_t j = 0; j != k + 1; ++j)
//                 {
//                         temp = ColVecs.col(j).transpose() * ColVecs.col(k + 1);
//                         ColVecs.col(k + 1) -= ColVecs.col(j) * temp;
//                 }
//                 ColVecs.col(k + 1).normalize();
//         }
// }