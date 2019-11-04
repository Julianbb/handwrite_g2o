#pragma once
#include "vertex.h"


namespace ZM
{
using namespace Eigen;

class vertexPoint3D : public BaseVertex<3, Eigen::Vector3d>
{


virtual void Plus(const double* delta)
{
    Eigen::Map<const Eigen::Vector3d> tmp_delta(delta);
    m_parameters += tmp_delta;
}


virtual std::string TypeVertex()
{
    return std::string("VertexPoint");
}

 void setToOriginImpl()
 {
     m_parameters.fill(0.0);
 }

};


}