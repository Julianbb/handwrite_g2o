#include "problem.h"



namespace ZM
{
 Problem::Problem()
 {

 }



void Problem::AddVertex(shared_ptr<Vertex> v)
{
    if(m_vertex.find(v->Id() != m_vertex.end)
        return;
    else
    {
        m_vertex.insert(make_pair(v->Id(), v));
    }
}




void Problem::RemoveVertex(shared_ptr<Vertex> v)
{
    if(m_vertex.find(v->Id() == m_vertex.end))
        return;
    else
    {
        m_vertex.erase(v->Id());
        m_vertex2edge.erase(v->Id());
    }

    //还要　remove vertex 对应的 edge
    vector<shared_ptr<BaseEdge>> edges_to_remove;
    edges_to_remove = GetEdgesConnectToVertex(v);
    for(int i=0; i<edges_to_remove.size(); i++)
    {
        RemoveEdge(edges_to_remove[i]);
    }
}




void Problem::AddEdge(shared_ptr<BaseEdge> e)
{
    if(m_edge.find(e->Id()) != m_edge.end())
        return;
    else
    {
        m_edge.insert(make_pair(e->Id(),e));
    }

    for(auto vertex : e->Verteies())
    {
        m_vertex2edge.insert(make_pair(vertex->Id(), e));
    }
}


void  Problem::RemoveEdge(shared_ptr<BaseEdge> e)
{
    if(m_edge.find(e->Id()) == m_edge.end())
        return;
    else
    {
        m_edge.erase(e->Id());
    }

}


vector<shared_ptr<BaseEdge>> Problem::GetEdgesConnectToVertex(shared_ptr<Vertex> v)
{
    vector<shared_ptr<BaseEdge>> edges;

    //返回所有m_vertex2edge的 key　是　v-Id()　的元素
    pair<HashVertexId2Edge::iterator, HashVertexId2Edge::iterator> equal_pair = m_vertex2edge.equal_range(v->Id());
    for(auto it = equal_pair.first; it != equal_pair.second; it++)
    {
        if(m_edge.find(it->second->Id()) == m_edge.end()) // 这条边已经被删除
            continue;

        edges.emplace_back(it->second);
    }
}

shared_ptr<Vertex> Problem::GetVertex(ulong i)
{
    assert(i<m_vertex.size());
    audo it = m_vertex.find(i);
    if(it!=m_vertex.end())
    {
        return it->second; 
    }
    else
    {
        return nullptr;
    }
}


void Problem::SetOrdering()
{
    unsigned int currentOrder = 0;
    for(auto& v : m_vertex)
    {
        m_total_demension += (v.second)->Dimension(); 
        v.second->SetOrdering(currentOrder);
        m_order_id_vertex.insert(make_pair(currentOrder, v.second));
        currentOrder += (v.second)->Dimension();
    }
}




}