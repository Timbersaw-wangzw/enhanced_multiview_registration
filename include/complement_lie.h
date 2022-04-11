/*
 * @Author: your name
 * @Date: 2022-03-07 19:50:19
 * @LastEditTime: 2022-03-08 16:06:07
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/complement_lie.h
 */
#ifndef __COMPLEMENT_LIE_H__
#define __COMPLEMENT_LIE_H__
#include <iostream>
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
namespace Sophus
{
        inline Eigen::MatrixXf cdot(Eigen::Vector4f p)
        {
                Eigen::MatrixXf M = Eigen::MatrixXf::Zero(4,6);
                M.block(0, 0, 3, 3) = Eigen::Matrix3f::Identity();
                M.block(0, 3, 3, 3) = -1 * Sophus::SO3f::hat(Eigen::Vector3f(p(0), p(1), p(2)));
                // std::cout << p << std::endl;
                // std::cout << "cdot p" << std::endl;
                // std::cout << M << std::endl;
                return M;
        }
}
#endif
