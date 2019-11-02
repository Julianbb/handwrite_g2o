#pragma once
#include <Eigen/Eigen>
#include <map>
#include <unordered_map>
#include <memory>
#include <vector>
#include "vertex.h"
#include "base_edge.h"


namespace ZM
{
    using namespace std;
    using namespace Eigen;

class Problem
{
public:
    typedef unsigned long ulong;
    typedef map<ulong, shared_ptr<Vertex>> HashVertex;
    typedef unordered_map<ulong, shared_ptr<BaseEdge>> HashEdge;
    typedef unordered_multimap<ulong, shared_ptr<BaseEdge>> HashVertexId2Edge

    Problem();
    
    void AddVertex(shared_ptr<Vertex> v);
    void AddEdge(shared_ptr<BaseEdge> e);
    void RemoveVertex(shared_ptr<Vertex> v);
    void RemoveEdge(shared_ptr<BaseEdge> e);
    vector<shared_ptr<BaseEdge>> GetEdgesConnectToVertex(shared_ptr<Vertex> v);
    shared_ptr<Vertex> GetVertex(ulong i);
    

    void SetOrdering();

    bool Solve(int itrators);
    void MakeHessian();


protected:


    HashVertex m_vertex;  
    HashEdge   m_edge;
    HashVertexId2Edge m_vertex2edge; // 由顶点查询边


    ulong m_total_demension;  //总变量维度
    HashVertex m_order_id_vertex;// <order_id, vertex>

private:
    Matrix<double, Dynamic, Dynamic> m_H;
    Matrix<double, Dynamic, 1>       m_b;
    Matrix<double, Dynamic, 1>       m_deltax;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




}