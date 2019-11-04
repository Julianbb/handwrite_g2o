
#pragma once
#include <string>
#include <Eigen/Eigen>


namespace ZM
{
using namespace Eigen;


class Vertex
{
public:
    typedef unsigned long ulong;

    void SetFixed(bool fixed = true){m_fixed = fixed;}
    bool IsFixed(void) const {return m_fixed;}

    ulong Id(void) {return m_id;}
    void SetId(ulong id){m_id = id;}
    ulong OrderId(void) {return m_order_id;}
    void SetOrderId(ulong order_id){m_order_id =order_id;}

    virtual std::string TypeVertex() = 0;

    ulong Dimension(void){return m_dimension;}
    virtual void Plus(const double* delta) = 0;

protected:
    bool m_fixed = false;

    ulong m_id = 0;
    ulong m_order_id;  // 每个顶点在hessian矩阵中的 order id
    ulong m_dimension;

};


template<int D, typename T>
class BaseVertex  : public Vertex
{
    typedef T EstimateType;
    typedef unsigned long ulong;
    typedef Matrix<double, D, 1> VecXd;


public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int DIMENSION = D;
    BaseVertex(); 

    EstimateType Parameters() const {return m_parameters;}
    EstimateType& Parameters(){return m_parameters;}
    void SetParameters(const EstimateType& para){m_parameters = para;}

    

    

protected:

    EstimateType m_parameters;
};


#include "vertex.hpp"




}