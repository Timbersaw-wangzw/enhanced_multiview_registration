#include "icp-ceres.h"
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
    Isometry3d sophusToIso(Sophus::SE3d soph){
    //    return Isometry3d(soph.matrix());
    Isometry3d poseFinal = Isometry3d::Identity();
    poseFinal.linear() = soph.rotationMatrix();
    poseFinal.translation() = soph.translation();
    return poseFinal;
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

    solve(problem);

    return sophusToIso(soph);
}

}
