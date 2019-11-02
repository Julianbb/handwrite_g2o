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


virtual void Plus(const Vector6d& delta)
{
    SetParameters(SE3Quat::exp(delta)*Parameters());
}


 void setToOriginImpl()
 {
     m_parameters = SE3Quat();
 }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};






}