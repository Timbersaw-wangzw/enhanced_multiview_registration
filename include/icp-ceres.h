#ifndef ICPCERES
#define ICPCERES

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include <ceres/local_parameterization.h>
#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/types.h>
#include "sophus_se3.h"
using namespace Eigen;
using namespace std;
namespace ICP_CERES
{
    Isometry3d pointToPlane_SophusSE3(vector<Vector3d> &src,vector<Vector3d> &dst,vector<Vector3d> &nor,bool automaticDiffLocalParam=true);
    struct PointToPlaneError_SophusSE3{
    const Eigen::Vector3d& p_dst;
    const Eigen::Vector3d& p_src;
    const Eigen::Vector3d& p_nor;


    PointToPlaneError_SophusSE3(const Eigen::Vector3d& dst, const Eigen::Vector3d& src, const Eigen::Vector3d& nor) :
    p_dst(dst), p_src(src), p_nor(nor)
    {
    }


    // Factory to hide the construction of the CostFunction object from the client code.
    static ceres::CostFunction* Create(const Eigen::Vector3d& observed, const Eigen::Vector3d& worldPoint, const Eigen::Vector3d& normal) {
        return (new ceres::AutoDiffCostFunction<PointToPlaneError_SophusSE3, 1, 7>(new PointToPlaneError_SophusSE3(observed, worldPoint,normal)));
    }

    template <typename T>
    bool operator()(const T* const cam1, T* residuals) const {

        // Make sure the Eigen::Vector world point is using the ceres::Jet type as it's Scalar type
        Eigen::Matrix<T,3,1> p; p << T(p_src[0]), T(p_src[1]), T(p_src[2]);
        Eigen::Matrix<T,3,1> point2; point2 << T(p_dst[0]), T(p_dst[1]), T(p_dst[2]);
        Eigen::Matrix<T,3,1> normal; normal << T(p_nor[0]), T(p_nor[1]), T(p_nor[2]);


        // Map the T* array to an Sophus SE3 object (with appropriate Scalar type)
        Sophus::SE3Group<T> q = Eigen::Map< const Sophus::SE3Group<T> >(cam1);

        // Rotate the point using Eigen rotations
        p = q.unit_quaternion() * p + q.translation();

        // The error is the difference between the predicted and observed position projected onto normal
        residuals[0] = (p - point2).dot(normal);

        return true;
    }
};
}
#endif // ICPCERES
