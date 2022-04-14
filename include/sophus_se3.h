#ifndef SOPHUS_SE3
#define SOPHUS_SE3
#include <iostream>
#include <sophus/se3.hpp>
#include <ceres/local_parameterization.h>

namespace sophus_se3 {

////from https://github.com/adrelino/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
struct SophusSE3Plus{
    template<typename T>
    bool operator()(const T* x_raw, const T* delta_raw, T* x_plus_delta_raw) const {
        const Eigen::Map< const Sophus::SE3<T> > x(x_raw);
        const Eigen::Map< const Eigen::Matrix<T,6,1> > delta(delta_raw);
        Eigen::Map< Sophus::SE3<T> > x_plus_delta(x_plus_delta_raw);
        x_plus_delta = Sophus::SE3<T>::exp(delta)*x;
        return true;
      }
};

//https://github.com/strasdat/Sophus/blob/develop/test/ceres/local_parameterization_se3.hpp
class LocalParameterizationSE3 : public ceres::LocalParameterization {
public:
  ~LocalParameterizationSE3() override = default;

  /**
   * \brief SE3 plus operation for Ceres
   *
   * \f$ \exp(\widehat{\delta})\cdot T \f$
   */
  bool Plus(const double * T_raw, const double * delta_raw,
                    double * T_plus_delta_raw) const override {
    const Eigen::Map<const Sophus::SE3d> T(T_raw);
    const Eigen::Map<const Eigen::Matrix<double,6,1> > delta(delta_raw);
    Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
    T_plus_delta =  Sophus::SE3d::exp(delta) * T;
    return true;
  }

  /**
   * \brief Jacobian of SE3 plus operation for Ceres
   *
   * \f$ \frac{\partial}{\partial \delta}\exp(\widehat{\delta})\cdot T|_{\delta=0} f$
   */
  bool ComputeJacobian(const double * T_raw, double * jacobian_raw)
    const override {
   Eigen::Map<Sophus::SE3d const> T(T_raw);
    Eigen::Map<Eigen::Matrix<double, 7, 6,Eigen::RowMajor>> jacobian(jacobian_raw);
    jacobian.setZero();
    jacobian.block<3,3>(4,0) = Eigen::Matrix3d::Identity();
    jacobian.block<3,3>(0,3) = Eigen::Matrix3d::Identity();//注意需要一一对应
    std::cout<<jacobian<<std::endl;
    return true;
  }

  int GlobalSize() const override {
    return Sophus::SE3d::num_parameters;
  }

  int LocalSize() const override {
    return Sophus::SE3d::DoF;
  }
};


//https://groups.google.com/forum/#!topic/ceres-solver/a9JhUIWOn1I
static ceres::LocalParameterization* getParameterization(bool automaticDiff){
    if(automaticDiff){
       std:: cout<<"automatic diff sophusSE3 local parameterization"<<std::endl;
        return new ceres::AutoDiffLocalParameterization<SophusSE3Plus, Sophus::SE3d::num_parameters, Sophus::SE3d::DoF>;
    }
    else{
        std::cout<<"analytic diff sophusSE3 local parameterization"<<std::endl;
        return new LocalParameterizationSE3();
    }
}


} //end ns sophus_se3

#endif // SOPHUS_SE3

