#pragma once
#include "vertex.h"
#include "se3quat.h"
#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
using namespace std;


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

void read(istream& is) 
{
    double data[7];
    for(int i = 0; i < 7; i++)
        is >> data[i];

    SE3Quat tmp_pose = SE3Quat(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                    Eigen::Vector3d(data[0], data[1], data[2]));

    SetParameters(tmp_pose);
}


void write(ostream & os)
{
    os << Id() << " ";
    Eigen::Quaterniond q = Parameters().Rotation().normalized();
    
    os << Parameters().Translation().transpose() << " ";
    os << q.coeffs()[0] << " "
       << q.coeffs()[1] << " "
       << q.coeffs()[2] << " "
       << q.coeffs()[3] << std::endl;
}

 void setToOriginImpl()
 {
     m_parameters = SE3Quat();
 }

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};






}