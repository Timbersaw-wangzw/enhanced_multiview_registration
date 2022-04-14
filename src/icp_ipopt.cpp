//
// Created by wangziwei on 2022/4/14.
//

#include "../include/icp_ipopt.h"

#include <utility>
using namespace pairwise;
using namespace Ipopt;
registrationNLP::registrationNLP(
        const pcl::PointCloud<pcl::PointXYZ> & source_cloud,
        const pcl::PointCloud<pcl::PointXYZ> & target_cloud,
        ICP::Parameters pars,
        bool printiterate) : printiterate_(printiterate)
{
    this->source_cloud = source_cloud;
    this->target_cloud = target_cloud;
    this->pars = std::move(pars);
}

// destructor
registrationNLP::~registrationNLP()
= default;

// [TNLP_get_nlp_info]
// returns the size of the problemf5f5
bool registrationNLP::get_nlp_info(
        Index &n,
        Index &m,
        Index &nnz_jac_g,
        Index &nnz_h_lag,
        IndexStyleEnum &index_style)
{
    // The problem described in registrationNLP.hpp has 4 variables, x[0] through x[3]
    n = 6;

    // one equality constraint and one inequality constraint
    m = 0;

    // in this example the jacobian is dense and contains 8 nonzeros
    nnz_jac_g = 0;

    // the Hessian is also dense and has 16 total nonzeros, but we
    // only need the lower left corner (since it is symmetric)
    nnz_h_lag = 21;

    // use the C style indexing (0-based)
    index_style = TNLP::C_STYLE;

    return true;
}
// [TNLP_get_nlp_info]

// [TNLP_get_bounds_info]
// returns the variable bounds
bool registrationNLP::get_bounds_info(
        Index n,
        Number *x_l,
        Number *x_u,
        Index m,
        Number *g_l,
        Number *g_u)
{
    // here, the n and m we gave IPOPT in get_nlp_info are passed back to us.
    // If desired, we could assert to make sure they are what we think they are.
    assert(n == 6);
    assert(m == 0);

    // the variables have lower bounds of 1
    for (int i = 0; i < 2; i++)
    {
        x_l[i] = -1e20;
        x_u[i] = 1e20;
    }
    for (int i = 3; i < 5; i++)
    {
        x_l[i] = -2 * M_PI;
        x_u[i] = 2 * M_PI;
    }
    // the variables have upper bounds of 5
    // for (Index i = 0; i < 4; i++)
    // {
    //         x_u[i] = 5.0;
    // }

    // the first constraint g1 has a lower bound of 25
    // g_l[0] = 25;
    // the first constraint g1 has NO upper bound, here we set it to 2e19.
    // Ipopt interprets any number greater than nlp_upper_bound_inf as
    // infinity. The default value of nlp_upper_bound_inf and nlp_lower_bound_inf
    // is 1e19 and can be changed through ipopt options.
    // g_u[0] = 2e19;

    // the second constraint g2 is an equality constraint, so we set the
    // upper and lower bound to the same value
    // g_l[1] = g_u[1] = 40.0;

    return true;
}
// [TNLP_get_bounds_info]

// [TNLP_get_starting_point]
// returns the initial point for the problem
bool registrationNLP::get_starting_point(
        Index n,
        bool init_x,
        Number *x,
        bool init_z,
        Number *z_L,
        Number *z_U,
        Index m,
        bool init_lambda,
        Number *lambda)
{
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    // initialize to the given starting point
    x[0] = 0;
    x[1] = 0;
    x[2] = 0;
    x[3] = 0;
    x[4] = 0;
    x[5] = 0;

    return true;
}
// [TNLP_get_starting_point]

// [TNLP_eval_f]
// returns the value of the objective function
bool registrationNLP::eval_f(
        Index n,
        const Number *x,
        bool new_x,
        Number &obj_value)
{
    assert(n == 6);
    obj_value = 0;
    Sophus::Vector6f lie;
    lie << (float)x[0], (float)x[1], (float)x[2], (float)x[3], (float)x[4], (float)x[5];
    Sophus::SE3f transform = Sophus::SE3f::exp(lie);
    int size = (int)source_cloud.points.size();
    for (int i = 0; i < size; i++)
    {
        Eigen::Vector4f p(transformed_cloud.points[i].x,
                          transformed_cloud.points[i].y,
                          transformed_cloud.points[i].z,
                          1);
        Eigen::Vector4f q(target_cloud.points[i].x,
                          target_cloud.points[i].y,
                          target_cloud.points[i].z,
                          1);
        Eigen::Vector4f temp =transform.matrix() * p - q;
        Eigen::VectorXd r;
        r<<(double)temp(0),(double)temp(1),(double)temp(2),(double)temp(3);
        obj_value = obj_value + (float)ICP::get_energy(pars.f, r, 1);
    }
    return true;
}
// [TNLP_eval_f]

// [TNLP_eval_grad_f]
// return the gradient of the objective function grad_{x} f(x)
bool registrationNLP::eval_grad_f(
        Index n,
        const Number *x,
        bool new_x,
        Number *grad_f)
{
    assert(n == 6);
    Sophus::Vector6f lie;
    lie << (float)x[0], (float)x[1], (float)x[2], (float)x[3], (float)x[4], (float)x[5];
    Sophus::SE3f transform = Sophus::SE3f::exp(lie);
    Eigen::Matrix<float, 6, 6> J = Sophus::SE3f::leftJacobian(lie);
    int size = (int)source_cloud.points.size();
    // pcl::transformPointCloud(*source_cloud, *transformed_cloud, transform.matrix());
    Eigen::MatrixXf grad = Eigen::MatrixXf::Zero(6,1);
    for (int i = 0; i < size; i++)
    {
        Eigen::Vector4f p(transformed_cloud.points[i].x,
                          transformed_cloud.points[i].y,
                          transformed_cloud.points[i].z,
                          1);
        Eigen::Vector4f tp = transform.matrix() * p;
        Eigen::Vector4f q(target_cloud.points[i].x,
                          target_cloud.points[i].y,
                          target_cloud.points[i].z,
                          1);
        Eigen::Vector4f temp =transform.matrix() * p - q;
        Eigen::VectorXd r;
        r<<(double)temp(0),(double)temp(1),(double)temp(2),(double)temp(3);
        auto r_norm=(float)r.norm();
        ICP::robust_weight(pars.f, r, 1);
        auto w=(float)r.norm();
        float influence = w / r_norm;
        grad += influence * (Sophus::cdot(tp) * J).transpose() * (tp - q);
    }
    for (int i = 0; i < 6; i++)
    {
        grad_f[i] = grad(i);
    }
    return true;
}
// [TNLP_eval_grad_f]

// [TNLP_eval_g]
// return the value of the constraints: g(x)

// [TNLP_eval_g]

// [TNLP_eval_jac_g]
// return the structure or values of the Jacobian
// [TNLP_eval_jac_g]

// [TNLP_eval_h]
// return the structure or values of the Hessian

// [TNLP_eval_h]

// [TNLP_finalize_solution]
void registrationNLP::finalize_solution(
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
        IpoptCalculatedQuantities *ip_cq)
{
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // For this example, we write the solution to the console
    std::cout << std::endl
              << std::endl
              << "Solution of the primal variables, x" << std::endl;
    for (Index i = 0; i < n; i++)
    {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    std::cout << std::endl
              << std::endl
              << "Solution of the bound multipliers, z_L and z_U" << std::endl;
    for (Index i = 0; i < n; i++)
    {
        std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    }
    for (Index i = 0; i < n; i++)
    {
        std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
    }

    std::cout << std::endl
              << std::endl
              << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;

    std::cout << std::endl
              << "Final value of the constraints:" << std::endl;
    for (Index i = 0; i < m; i++)
    {
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
}
// [TNLP_finalize_solution]

// [TNLP_intermediate_callback]
bool registrationNLP::intermediate_callback(
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
        IpoptCalculatedQuantities *ip_cq)
{
    if (!printiterate_)
    {
        return true;
    }

    Number x[4];
    Number x_L_viol[4];
    Number x_U_viol[4];
    Number z_L[4];
    Number z_U[4];
    Number compl_x_L[4];
    Number compl_x_U[4];
    Number grad_lag_x[4];

    Number g[2];
    Number lambda[2];
    Number constraint_violation[2];
    Number compl_g[2];

    bool have_iter = get_curr_iterate(ip_data, ip_cq, false, 4, x, z_L, z_U, 2, g, lambda);
    bool have_viol = get_curr_violations(ip_data, ip_cq, false, 4, x_L_viol, x_U_viol, compl_x_L, compl_x_U, grad_lag_x, 2, constraint_violation, compl_g);

    printf("Current iterate:\n");
    printf("  %-12s %-12s %-12s %-12s %-12s %-12s %-12s\n", "x", "z_L", "z_U", "bound_viol", "compl_x_L", "compl_x_U", "grad_lag_x");
    for (int i = 0; i < 4; ++i)
    {
        if (have_iter)
        {
            printf("  %-12g %-12g %-12g", x[i], z_L[i], z_U[i]);
        }
        else
        {
            printf("  %-12s %-12s %-12s", "n/a", "n/a", "n/a");
        }
        if (have_viol)
        {
            printf(" %-12g %-12g %-12g %-12g\n", x_L_viol[i] > x_U_viol[i] ? x_L_viol[i] : x_U_viol[i], compl_x_L[i], compl_x_U[i], grad_lag_x[i]);
        }
        else
        {
            printf(" %-12s %-12s %-12s %-12s\n", "n/a", "n/a", "n/a", "n/a");
        }
    }

    printf("  %-12s %-12s %-12s %-12s\n", "g(x)", "lambda", "constr_viol", "compl_g");
    for (int i = 0; i < 2; ++i)
    {
        if (have_iter)
        {
            printf("  %-12g %-12g", g[i], lambda[i]);
        }
        else
        {
            printf("  %-12s %-12s", "n/a", "n/a");
        }
        if (have_viol)
        {
            printf(" %-12g %-12g\n", constraint_violation[i], compl_g[i]);
        }
        else
        {
            printf(" %-12s %-12s\n", "n/a", "n/a");
        }
    }

    return true;
}

// [TNLP_intermediate_callback]