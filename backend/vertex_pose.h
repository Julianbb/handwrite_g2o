#pragma once
#include "vertex.h"
#include "se3quat.h"
#include <Eigen/Eigen>



namespace ZM
{
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;

class VertexPose6Dof : public BaseVertex<6, SE3Quat>
{
public:
    VertexPose6Dof(){}


virtual void Plus(const double* delta)
{
    Eigen::Map<const Vector6d> tmp_delta(delta);
    SetParameters(SE3Quat::exp(tmp_delta)*Parameters());
}

virtual std::string TypeVertex()
{
    return std::string("VertexPose");
}




 void setToOriginImpl()
 {
     m_parameters = SE3Quat();
 }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};






}