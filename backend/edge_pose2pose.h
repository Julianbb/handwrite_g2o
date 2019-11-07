#pragma once

#include "base_binary_edge.h"
#include "vertex_pose.h"
#include <iostream>
#include <fstream>
using namespace std;


namespace ZM
{
using namespace Eigen;
typedef Matrix<double, 6, 6> Matrix6d;
typedef Matrix<double, 6, 1> Vector6d;

class EdgePose2Pose :public BaseBinaryEdge<6, SE3Quat, VertexPose6Dof, VertexPose6Dof>
{

public:
    virtual void ComputeError();
    virtual void ComputeJacobian();

    void read(ifstream& is);
    void write(ofstream& os);

private:
    Matrix6d JRInv(Vector6d error);




public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};



}