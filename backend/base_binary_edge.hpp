

template <int D, typename type_of_mesurement, typename type_of_vertexi, typename type_of_vertexj>
BaseBinaryEdge<D, type_of_mesurement, type_of_vertexi, type_of_vertexj>::BaseBinaryEdge()
{
    m_vertices.resize(2);
    m_dimension = D;
    m_jacobians.resize(2);

    m_information = InformationType::Identity();
    m_error = ErrorVector::Zero();
}