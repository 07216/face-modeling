#include "KDTreeAdaptor.h"
/*
/// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
/// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
nanoflann::KDTreeAdaptor::KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat)
{
    const size_t dims = mat.rows();
    index = new index_t( dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size, dims ) );
    index->buildIndex();
}

template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
nanoflann::KDTreeAdaptor::~KDTreeAdaptor()
{
    delete index;
}

/// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
const nanoflann::KDTreeAdaptor::self_t & derived() const
{
    return *this;
}

template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
self_t & nanoflann::KDTreeAdaptor::derived()
{
    return *this;
}

/// Optional bounding-box computation: return false to default to a standard bbox computation loop.
template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
template <class BBOX>
bool nanoflann::KDTreeAdaptor::kdtree_get_bbox(BBOX&) const
{
    return false;
}
*/


