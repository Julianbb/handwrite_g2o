#pragma once

#include "base_binary_edge.h"
#include "vertex_point.h"
#include "vertex_pose.h"

namespace ZM
{
using namespace Eigen;


class EdgeSE3ProjectXYZ : public BaseBinaryEdge<2, Vector2d, vertexPoint3D, VertexPose6Dof>
{
public: 
    EdgeSE3ProjectXYZ(const Matrix3d& K);

    virtual void ComputeError();
    virtual void ComputeJacobian();


private:
    double fx;
    double fy;
    double cx; 
    double cy;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}