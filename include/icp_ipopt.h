//
// Created by wangziwei on 2022/4/14.
//

#ifndef MULTIVEWREGISTRAION_ICP_IPOPT_H
#define MULTIVEWREGISTRAION_ICP_IPOPT_H
// ipopt
#include "IpTNLP.hpp"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <sophus/se3.hpp>
#include "complement_lie.h"
#include "ICP.h"
using namespace Ipopt;
namespace pairwise{
        class registrationNLP : public TNLP
        {
            public:
            /** Constructor */
            registrationNLP(
                    const pcl::PointCloud<pcl::PointXYZ>& source_cloud,
                    const pcl::PointCloud<pcl::PointXYZ> & target_cloud,
                    ICP::Parameters pars,
                    bool printiterate = false /**< whether to print the iterate at each iteration */
            );

            /** Destructor */
            ~registrationNLP() override;

            /**@name Overloaded from TNLP */
            //@{
            /** Method to return some info about the NLP */
            bool get_nlp_info(
                    Index &n,
                    Index &m,
                    Index &nnz_jac_g,
                    Index &nnz_h_lag,
                    IndexStyleEnum &index_style) override;



            /** Method to return the starting point for the algorithm */
            bool get_starting_point(
                    Index n,
                    bool init_x,
                    Number *x,
                    bool init_z,
                    Number *z_L,
                    Number *z_U,
                    Index m,
                    bool init_lambda,
                    Number *lambda) override;

            /** Method to return the objective value */
            bool eval_f(
                    Index n,
                    const Number *x,
                    bool new_x,
                    Number &obj_value) override;

            /** Method to return the gradient of the objective */
            bool eval_grad_f(
                    Index n,
                    const Number *x,
                    bool new_x,
                    Number *grad_f) override;

            /** Method to return the constraint residuals */
            bool eval_g(
                    Index n,
                    const Number *x,
                    bool new_x,
                    Index m,
                    Number *g) override
            {
                assert(n == 6 && m == 0);
                return true;
            };

            /** Method to return:
                                 *   1) The structure of the jacobian (if "values" is NULL)
                                 *   2) The values of the jacobian (if "values" is not NULL)
                                 */
            bool eval_jac_g(
                    Index n,
                    const Number *x,
                    bool new_x,
                    Index m,
                    Index nele_jac,
                    Index *iRow,
                    Index *jCol,
                    Number *values) override
            {
                assert(n == 6 && m == 0 && nele_jac == 0);
                return true;
            };

            /** Method to return:
                                 *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
                                 *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
                                 */
            bool eval_h(
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
                    Number *values) override
            {
                return false;
            };
            /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
            void finalize_solution(
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
                    IpoptCalculatedQuantities *ip_cq) override;
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
                    IpoptCalculatedQuantities *ip_cq) override;

            private:
            /** whether to print iterate to stdout in intermediate_callback() */
            bool printiterate_;
            pcl::PointCloud<pcl::PointXYZ> source_cloud;
            pcl::PointCloud<pcl::PointXYZ> target_cloud;
            pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
            ICP::Parameters pars;
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
            bool get_bounds_info(Index n, Number *x_l, Number *x_u, Index m, Number *g_l, Number *g_u) override;
        };
}



#endif //MULTIVEWREGISTRAION_ICP_IPOPT_H
