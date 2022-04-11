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
#include "robust_function.h"
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

using namespace Ipopt;
namespace robustOptimize
{
    namespace pairwiseICP
    {
    enum solverType
    {
        ipopt,
        cceres,
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
    class reigstraionNLP : public TNLP
    {
    public:
        /** Constructor */
        reigstraionNLP(
                pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                robustOptimize::robustFunc rbfcn,
                bool printiterate = false /**< whether to print the iterate at each iteration */
                );

        /** Destructor */
        virtual ~reigstraionNLP();

        /**@name Overloaded from TNLP */
        //@{
        /** Method to return some info about the NLP */
        virtual bool get_nlp_info(
                Index &n,
                Index &m,
                Index &nnz_jac_g,
                Index &nnz_h_lag,
                IndexStyleEnum &index_style);

        /** Method to return the bounds for my problem */
        virtual bool get_bounds_info(
                Index n,
                Number *x_l,
                Number *x_u,
                Index m,
                Number *g_l,
                Number *g_u);

        /** Method to return the starting point for the algorithm */
        virtual bool get_starting_point(
                Index n,
                bool init_x,
                Number *x,
                bool init_z,
                Number *z_L,
                Number *z_U,
                Index m,
                bool init_lambda,
                Number *lambda);

        /** Method to return the objective value */
        virtual bool eval_f(
                Index n,
                const Number *x,
                bool new_x,
                Number &obj_value);

        /** Method to return the gradient of the objective */
        virtual bool eval_grad_f(
                Index n,
                const Number *x,
                bool new_x,
                Number *grad_f);

        /** Method to return the constraint residuals */
        virtual bool eval_g(
                Index n,
                const Number *x,
                bool new_x,
                Index m,
                Number *g)
        {
            assert(n == 6 && m == 0);
            return true;
        };

        /** Method to return:
                             *   1) The structure of the jacobian (if "values" is NULL)
                             *   2) The values of the jacobian (if "values" is not NULL)
                             */
        virtual bool eval_jac_g(
                Index n,
                const Number *x,
                bool new_x,
                Index m,
                Index nele_jac,
                Index *iRow,
                Index *jCol,
                Number *values)
        {
            assert(n == 6 && m == 0 && nele_jac == 0);
            return true;
        };

        /** Method to return:
                             *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
                             *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
                             */
        virtual bool eval_h(
                Index n,
                const Number *x,
                bool new_x,
                Number obj_factor,
                Index m,
                const Number *lambda,
                bool new_lambda,
                Index nele_hess,
                Index *iRow,
                Index *jCol,
                Number *values)
        {
            return false;
        };
        /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
        virtual void finalize_solution(
                SolverReturn status,
                Index n,
                const Number *x,
                const Number *z_L,
                const Number *z_U,
                Index m,
                const Number *g,
                const Number *lambda,
                Number obj_value,
                const IpoptData *ip_data,
                IpoptCalculatedQuantities *ip_cq);
        //@}

        bool intermediate_callback(
                AlgorithmMode mode,
                Index iter,
                Number obj_value,
                Number inf_pr,
                Number inf_du,
                Number mu,
                Number d_norm,
                Number regularization_size,
                Number alpha_du,
                Number alpha_pr,
                Index ls_trials,
                const IpoptData *ip_data,
                IpoptCalculatedQuantities *ip_cq);
        void getTransformedcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
        {
            transformed_cloud = this->transformed_cloud;
        }

    private:
        /** whether to print iterate to stdout in intermediate_callback() */
        bool printiterate_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        robustOptimize::robustFunc rbfcn;
        /**@name Methods to block default compiler methods.
                             *
                             * The compiler automatically generates the following three methods.
                             *  Since the default compiler implementation is generally not what
                             *  you want (for all but the most simple classes), we usually
                             *  put the declarations of these methods in the private section
                             *  and never implement them. This prevents the compiler from
                             *  implementing an incorrect "default" behavior without us
                             *  knowing. (See Scott Meyers book, "Effective C++")
                             */
        //@{
    };
    class registrationCeres final : public ceres::FirstOrderFunction
    {
    public:
        registrationCeres(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                          robustOptimize::robustFunc rbfcn)
        {
            this->source_cloud = source_cloud;
            this->target_cloud = target_cloud;
            this->rbfcn = rbfcn;
        }
        bool Evaluate(const double *parameters,
                      double *cost,
                      double *gradient) const override
        {
            if (gradient)
            {
                Sophus::Vector6f lie;
                lie << (float)parameters[0],
                        (float)parameters[1],
                        (float)parameters[2],
                        (float)parameters[3],
                        (float)parameters[4],
                        (float)parameters[5];
                Sophus::SE3f transform = Sophus::SE3f::exp(lie);
                int size = source_cloud->points.size();
                //            pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::MatrixXf grad = Eigen::MatrixXf::Zero(6,1);
                Eigen::Matrix<float, 6, 6> J = Sophus::SE3f::leftJacobian(lie);
                double obj_value = 0;
                for (int i = 0; i < size; i++)
                {
                    Eigen::Vector4f p(source_cloud->points[i].x,
                                      source_cloud->points[i].y,
                                      source_cloud->points[i].z,
                                      1);
                    Eigen::Vector4f q(target_cloud->points[i].x,
                                      target_cloud->points[i].y,
                                      target_cloud->points[i].z,
                                      1);
                    Eigen::Vector4f tp=transform.matrix()*p;
                    float norm_x = (tp - q).norm();
                    float w = rbfcn.weightFunction(norm_x);
                    float influence = w / norm_x;
                    Eigen::MatrixXf cdot_tp=Sophus::cdot(tp);
                    grad += influence * ( cdot_tp* J).transpose() * (p+cdot_tp*J*lie - q);
                    obj_value = obj_value + rbfcn.costFunction(norm_x);
                }
                cost[0] = obj_value;
                std::cout<<grad<<std::endl;
                for (int i = 0; i < 6; i++)
                {
                    gradient[i] = grad(i);
                }
            }
            return true;
        };

        int NumParameters() const override { return 2; }

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        robustOptimize::robustFunc  rbfcn;
        int max_iter;
        float threshold;
        float error;
    };
    class registrationManual
    {
    public:
        registrationManual(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                           robustOptimize::robustFunc rbfcn,
                           int max_iter,
                           float threshold)
        {
            this->source_cloud = source_cloud;
            this->target_cloud = target_cloud;
            this->rbfcn = rbfcn;
            this->max_iter=max_iter;
            this->threshold=threshold;
        }
        void optimize();
        void finalError(float &error)
        {
            error=this->error;
        };
        void getTransformation(Eigen::Matrix4f & result)
        {
            result= this->result;
        };
        void getIterativeError()
        {

        };
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        Eigen::Matrix4f result;
        robustOptimize::robustFunc  rbfcn;
        int max_iter;
        float threshold;
        float error;
        std::vector<float> rmse_error_list;
        std::vector<float> transformation_error_list;
    };
    class registrationLoop

    {
    public:
        registrationLoop(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                         robustOptimize::robustFunc rbfcn,
                         int max_iter,
                         float threshold,
                         solverType type)
        {
            this->max_iter = max_iter;
            this->threshold = threshold;
            this->source_cloud = source_cloud;
            this->target_cloud = target_cloud;
            this->rbfcn = rbfcn;
            this->type = type;
        };
        // loop find cloeset points and optimize by IPOPT
        bool optimize();
        ~registrationLoop(){}
        void getTransformation(Eigen::Matrix4f & result)
        {
            result= this->result;
        };
        void getError(float &error)
        {
            error = this->error;
        }

    private:
        bool ipoptSolve(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud);
        bool cereosSolve(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud);
        bool manualOptimize(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr closest_cloud,
                            int inner_iter);
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        robustOptimize::robustFunc rbfcn;
        int max_iter;
        float threshold;
        float error;
        Eigen::Matrix4f  result;
        solverType type;
    };
    class registrationICP
    {
    public:
        registrationICP( Eigen::Matrix<double, 3, Eigen::Dynamic>* source_cloud,
                           Eigen::Matrix<double, 3, Eigen::Dynamic>* target_cloud,
                           solverType method)
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
        Eigen::Matrix<double, 3, Eigen::Dynamic> transformed_cloud;
        Eigen::Matrix<double, 3, Eigen::Dynamic> normal_source;
        Eigen::Matrix<double, 3, Eigen::Dynamic> normal_target;
        solverType method;
    };
    }
}
#endif
