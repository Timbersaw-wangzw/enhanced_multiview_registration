#include "icp_ceres.h"
#include <Eigen/Dense>
#include <math.h>
#include <unordered_map>
#include <vector>
#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/types.h>
#include <ceres/rotation.h>
#include <ceres/ceres.h>

#include <ceres/loss_function.h>

namespace ICP_CERES{
    ceres::Solver::Options getOptions(){
        // Set a few options
        ceres::Solver::Options options;
        //options.use_nonmonotonic_steps = true;
        //options.preconditioner_type = ceres::IDENTITY;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;

    //    options.preconditioner_type = ceres::SCHUR_JACOBI;
    //    options.linear_solver_type = ceres::DENSE_SCHUR;
    //    options.use_explicit_schur_complement=true;
    //    options.max_num_iterations = 100;

        cout << "Ceres Solver getOptions()" << endl;
        cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
        cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
        cout << "Ceres linear solver type: " << options.linear_solver_type << endl;
        return options;
    }
    ceres::Solver::Options getOptionsMedium(){
    // Set a few options
    ceres::Solver::Options options;

    #ifdef _WIN32
        options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        options.linear_solver_type = ceres::ITERATIVE_SCHUR;
        options.preconditioner_type = ceres::SCHUR_JACOBI;
    #else
        //options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    #endif // _WIN32

    //If you are solving small to medium sized problems, consider setting Solver::Options::use_explicit_schur_complement to true, it can result in a substantial performance boost.
    options.use_explicit_schur_complement=true;
    options.max_num_iterations = 50;

    cout << "Ceres Solver getOptionsMedium()" << endl;
    cout << "Ceres preconditioner type: " << options.preconditioner_type << endl;
    cout << "Ceres linear algebra type: " << options.sparse_linear_algebra_library_type << endl;
    cout << "Ceres linear solver type: " << options.linear_solver_type << endl;

    return options;
}


void solve(ceres::Problem &problem, bool smallProblem=false){
    ceres::Solver::Summary summary;
    ceres::Solve(smallProblem ? getOptions() : getOptionsMedium(), &problem, &summary);
    if(!smallProblem) std::cout << "Final report:\n" << summary.FullReport();
}
}


Isometry3d sophusToIso(Sophus::SE3d soph)
{
    //    return Isometry3d(soph.matrix());
    Isometry3d poseFinal = Isometry3d::Identity();
    poseFinal.linear() = soph.rotationMatrix();
    poseFinal.translation() = soph.translation();
    return poseFinal;
}

Sophus::SE3d isoToSophus(const Isometry3d& pose){
    return Sophus::SE3d(pose.rotation(),pose.translation());
}
Isometry3d pointToPlane_SophusSE3(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor, bool autodiff){
    Sophus::SE3d soph = isoToSophus(Isometry3d::Identity());

    ceres::Problem problem;

    for (int i = 0; i < src.size(); ++i) {
        // first viewpoint : dstcloud, fixed
        // second viewpoint: srcCloud, moves
        ceres::CostFunction* cost_function = ICPCostFunctions::PointToPlaneError_SophusSE3::Create(dst[i],src[i],nor[i]);
        problem.AddResidualBlock(cost_function, NULL,soph.data());
    }
#ifdef useLocalParam
    ceres::LocalParameterization* param = sophus_se3::getParameterization(autodiff);
    problem.SetParameterization(soph.data(),param);
#endif

   ICP_CERES:: solve(problem);

    return sophusToIso(soph);
}

