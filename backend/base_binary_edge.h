#pragma once
#include <Eigen/Eigen>
#include <assert.h>
#include <memory>
#include <string>
#include "base_edge.h"

namespace ZM
{
using namespace Eigen;

class Vertex;



template <int D, typename type_of_mesurement, typename type_of_vertexi, typename type_of_vertexj>
class BaseBinaryEdge : public BaseEdge<D, type_of_mesurement>
{
public:
    typedef std::vector<Vertex*> VertexContainer;
    BaseBinaryEdge();


    static const int Di = type_of_vertexi::DIMENSION;
    static const int Dj = type_of_vertexj::DIMENSION; // typename::AlignedMapType
    typedef  Matrix<double, D, Di> JacobianXiOplusType;
    typedef  Matrix<double, D, Dj> JacobianXjOplusType;

    virtual void ComputeError() = 0;
    virtual void ComputeJacobian() = 0;

    void SetVertex(size_t i, Vertex* v) {assert(i < m_vertices.size()); m_vertices[i] = v;}
    Vertex* Vertexi(size_t i) { assert(i < m_vertices.size()); return m_vertices[i];}
    VertexContainer  Verteies(){return m_vertices;}


    const JacobianXiOplusType& jacobianOplusXi() const { return m_jacobianOplusXi;}
    const JacobianXjOplusType& jacobianOplusXj() const { return m_jacobianOplusXj;}

protected:
    using BaseEdge<D, type_of_mesurement>::m_vertices;
    JacobianXiOplusType m_jacobianOplusXi;
    JacobianXjOplusType m_jacobianOplusXj;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_binary_edge.hpp"
}