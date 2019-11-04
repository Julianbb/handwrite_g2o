#pragma once
#include <Eigen/Eigen>
#include "vertex.h"


namespace ZM
{
using namespace Eigen;

class Vertex;





// template <int D, typename type_of_mesurement>
class BaseEdge
{
public:

    typedef std::vector<Vertex*> VertexContainer;

    void SetVertex(size_t i, Vertex* v) {assert(i < m_vertices.size()); m_vertices[i] = v;}
    Vertex* Vertexi(size_t i) { assert(i < m_vertices.size()); return m_vertices[i];}
    VertexContainer  Verteies(){return m_vertices;}


    void SetId(const unsigned int& id){m_id = id;}
    unsigned int Id(){return m_id;}

    virtual void ComputeError() = 0;
    virtual void ComputeJacobian() = 0;
    virtual double Chi2()const = 0;

    virtual double* ErrorData() = 0;
    virtual const double* ErrorData()const = 0;
    virtual double* InformationData() = 0;
    virtual const double* InformationData() const  = 0;

    std::vector<MatrixXd> Jacobians() {return m_jacobians;}
    ulong Dimension(){return m_dimension;}

protected:

    unsigned int m_id;
    unsigned int m_dimension; // 测量值的,也是误差
    VertexContainer m_vertices;


    std::vector<MatrixXd> m_jacobians;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



}