/*
 * @Author: your name
 * @Date: 2022-03-07 11:01:01
 * @LastEditTime: 2022-03-08 16:28:44
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: /pcl_test/include/ipopt_registration.h
 */
#ifndef __ROBUST_REGISTRATION_H__
#define __ROBUST_REGISTRATION_H__
// ipopt
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
// include files
#include "complement_lie.h"
// eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Cholesky>
#include "sophus/se3.hpp"

// ceres
#include "ceres/ceres.h"
#include "glog/logging.h"

#include "ICP.h"
#include "FRICP.h"


namespace pairwiseICP
{
    enum solveMethod
    {
        ipopt,
        manual,
        ICP,
        AA_ICP,
        FICP,
        RICP,
        PPL,
        RPPL,
        SparseICP,
        SICPPPL
    };
    class registrationICP
    {
    public:
        registrationICP(Eigen::Matrix<double, 3, Eigen::Dynamic>* source_cloud,
                        Eigen::Matrix<double, 3, Eigen::Dynamic>* target_cloud,
                        solveMethod method)
        {
            this->vertices_source = source_cloud;
            this->vertices_target = target_cloud;
            this->method=method;
        }
        void align();
        MatrixXX res_trans;
    private:
        Eigen::Matrix<double, 3, Eigen::Dynamic> *vertices_source;
        Eigen::Matrix<double, 3, Eigen::Dynamic> *vertices_target;
        Eigen::Matrix<double, 3, Eigen::Dynamic> normal_source;
        Eigen::Matrix<double, 3, Eigen::Dynamic> normal_target;
        solveMethod method;
    };
}
#endif
