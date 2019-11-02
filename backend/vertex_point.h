#pragma once
#include "vertex.h"


namespace ZM
{
using namespace Eigen;

class vertexPoint3D : public BaseVertex<3, Eigen::Vector3d>
{


virtual void Plus(const Vector3d& delta)
{
    m_parameters += delta;
}

 void setToOriginImpl()
 {
     m_parameters.fill(0.0);
 }

};


}