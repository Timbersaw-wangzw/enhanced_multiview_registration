/*
 * @Author: your name
 * @Date: 2022-03-08 15:40:59
 * @LastEditTime: 2022-03-08 15:43:46
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/robust_cost_grad.h
 */
#ifndef __ROBUST_COST_GRAD_H__
#define __ROBUST_COST_GRAD_H__
// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "sophus/se3.hpp"
namespace robustOptimize
{
        static class robustCostAndGradFcn
        {
        public:
                static void robustICP();
        }
}
#endif