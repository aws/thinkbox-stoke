// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/data_types.hpp>
#include <frantic/volumetrics/field_interface.hpp>

#pragma warning( push, 3 )
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#pragma warning( pop )

#pragma warning( push, 3 )
#include <boost/bind.hpp>
#pragma warning( pop )

namespace ember {

inline vec3 integer_to_float_coord( int x, int y, int z, float spacing ) {
    return vec3( ( (float)x + 0.5f ) * spacing, ( (float)y + 0.5f ) * spacing, ( (float)z + 0.5f ) * spacing );
}

inline void float_to_integer_coord( const vec3& p, float spacing, int ( &outIntCoord )[3] ) {
    outIntCoord[0] = (int)std::floor( p.x / spacing - 0.5f );
    outIntCoord[1] = (int)std::floor( p.y / spacing - 0.5f );
    outIntCoord[2] = (int)std::floor( p.z / spacing - 0.5f );
}

inline void float_to_integer_coord_and_deltas( const vec3& p, float spacing, int ( &outIntCoord )[3],
                                               float ( &outDeltas )[3] ) {
    float voxelX = p.x / spacing - 0.5f;
    float floorX = std::floor( voxelX );
    outDeltas[0] = voxelX - floorX;
    outIntCoord[0] = static_cast<int>( floorX );

    float voxelY = p.y / spacing - 0.5f;
    float floorY = std::floor( voxelY );
    outDeltas[1] = voxelY - floorY;
    outIntCoord[1] = static_cast<int>( floorY );

    float voxelZ = p.x / spacing - 0.5f;
    float floorZ = std::floor( voxelZ );
    outDeltas[2] = voxelZ - floorZ;
    outIntCoord[2] = static_cast<int>( floorZ );
}

template <class T>
class grid_helper;

template <class T>
class grid {
  public:
    inline grid();
    inline grid( const int ( &bounds )[6], float spacing );

    inline void swap( grid& rhs );

    inline float get_spacing() const;
    inline int get_size( int axis ) const;
    inline int get_min( int axis ) const;
    inline int get_max( int axis ) const;

    inline const T& get_voxel( int x, int y, int z ) const;
    inline void set_voxel( int x, int y, int z, const T& val );

    template <class T>
    friend class grid_helper;

  private:
    int m_min[3];
    int m_size[3];
    float m_spacing;

    std::vector<T> m_storage;
};

void assign( grid<float>& grid, const frantic::volumetrics::field_interface& field );
void assign( grid<vec3>& grid, const frantic::volumetrics::field_interface& field );

float trilerp( const grid<float>& grid, const vec3& pos );
vec3 trilerp( const grid<vec3>& grid, const vec3& pos );

/**
 * Splats a particle collection onto the grid using a trilinear kernel with radius 1.
 * @tparam ForwardIterator Models the STL forward iterator concept, dereferencing as a void* to the particle's data.
 * @tparam ParticleAccessor A functor which implements 'void operator()( const void* particle, vec3& outPosition, float&
 * outValue ) const;'
 * @param grid The grid to splat on to.
 * @param it An iterator pointing to the first particle in the collection.
 * @param itEnd An iterator pointing to the last particle in the collection.
 * @param pa The functor which will extract the position and splat value for a derefenced particle iterator
 */
template <class T, class ForwardIterator, class ParticleAccessor>
void splat_particles( grid<T>& grid, ForwardIterator it, ForwardIterator itEnd, const ParticleAccessor& pa );

/**
 * @overload Splats a particle collection onto the grid using a trilinear kernel with radius 1. A data channel and the
 * total weighting of each voxel is stored to 'grid' and 'weightGrid' respectively.
 * @tparam ForwardIterator Models the STL forward iterator concept, dereferencing as a void* to the particle's data.
 * @tparam ParticleAccessor A functor which implements 'void operator()( const void* particle, vec3& outPosition, T&
 * outValue, float& outWeight ) const;'
 * @param grid The grid to splat the data value on to.
 * @param weightGrid The grid to accumulate the weights of splatted particles for each voxel. Must have ther same
 * dimensions as 'grid'.
 * @param it An iterator pointing to the first particle in the collection.
 * @param itEnd An iterator pointing to the last particle in the collection.
 * @param pa The functor which will extract the position, weight and splat value for a derefenced particle iterator
 */
template <class T, class ForwardIterator, class ParticleAccessor>
void splat_weighted_particles( grid<T>& theGrid, grid<float>& weightGrid, ForwardIterator it, ForwardIterator itEnd,
                               const ParticleAccessor& pa );

/**
 * Normalizes the values stored on a grid by dividing out the accumulated weights from particle splats. Since splatted
 * particles are added together, we need to divide out the relative amount of particles that were splatted to each grid
 * point, otherwise the data would behave additively.
 * @param[in|out] theGrid The grid to normalize
 * @param weightGrid The grid storing the accumulated weights splatted to each voxel.
 */
template <class T>
void normalize_grid_from_weights( grid<T>& theGrid, const grid<float>& weightGrid );

template <class T>
inline grid<T>::grid() {
    m_min[0] = m_min[1] = m_min[2] = m_size[0] = m_size[1] = m_size[2] = 0;
    m_spacing = 1.f;
}

template <class T>
inline grid<T>::grid( const int ( &bounds )[6], float spacing ) {
    m_min[0] = bounds[0];
    m_min[1] = bounds[2];
    m_min[2] = bounds[4];
    m_size[0] = ( std::max )( 0, bounds[1] - bounds[0] );
    m_size[1] = ( std::max )( 0, bounds[3] - bounds[2] );
    m_size[2] = ( std::max )( 0, bounds[5] - bounds[4] );
    m_spacing = spacing;
    m_storage.resize( ( std::size_t )( m_size[0] * m_size[1] * m_size[2] ) );
}

template <class T>
inline void grid<T>::swap( grid<T>& rhs ) {
    std::swap( m_min[0], rhs.m_min[0] );
    std::swap( m_min[1], rhs.m_min[1] );
    std::swap( m_min[2], rhs.m_min[2] );
    std::swap( m_size[0], rhs.m_size[0] );
    std::swap( m_size[1], rhs.m_size[1] );
    std::swap( m_size[2], rhs.m_size[2] );
    std::swap( m_spacing, rhs.m_spacing );

    m_storage.swap( rhs.m_storage );
}

template <class T>
inline float grid<T>::get_spacing() const {
    return m_spacing;
}

template <class T>
inline int grid<T>::get_size( int axis ) const {
    return m_size[axis];
}

template <class T>
inline int grid<T>::get_min( int axis ) const {
    return m_min[axis];
}

template <class T>
inline int grid<T>::get_max( int axis ) const {
    return get_min( axis ) + get_size( axis );
}

template <class T>
inline const T& grid<T>::get_voxel( int x, int y, int z ) const {
    return m_storage[x + m_size[0] * ( y + m_size[1] * z )];
}

template <class T>
inline void grid<T>::set_voxel( int x, int y, int z, const T& val ) {
    m_storage[x + m_size[0] * ( y + m_size[1] * z )] = val;
}

template <class T>
class discretized_field : public frantic::volumetrics::field_interface {
  public:
    explicit discretized_field( const frantic::tstring& valueName = _T("Value") );

    discretized_field( const int ( &bounds )[6], float spacing, const frantic::tstring& valueName = _T("Value") );

    virtual ~discretized_field();

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    grid<T>& get_grid();

    const grid<T>& get_grid() const;

    void assign( const frantic::volumetrics::field_interface& field );

  private:
    grid<T> m_storage;

    frantic::channels::channel_map m_channelMap;
};

template <class T>
inline discretized_field<T>::discretized_field( const frantic::tstring& valueName ) {
    m_channelMap.define_channel<T>( valueName );
    m_channelMap.end_channel_definition();
}

template <class T>
inline discretized_field<T>::discretized_field( const int ( &bounds )[6], float spacing,
                                                const frantic::tstring& valueName )
    : m_storage( bounds, spacing ) {
    m_channelMap.define_channel<T>( valueName );
    m_channelMap.end_channel_definition();
}

template <class T>
inline discretized_field<T>::~discretized_field() {}

template <class T>
inline const frantic::channels::channel_map& discretized_field<T>::get_channel_map() const {
    return m_channelMap;
}

template <class T>
inline grid<T>& discretized_field<T>::get_grid() {
    return m_storage;
}

template <class T>
inline const grid<T>& discretized_field<T>::get_grid() const {
    return m_storage;
}

template <class T>
inline void discretized_field<T>::assign( const frantic::volumetrics::field_interface& field ) {
    ember::assign( m_storage, field );
}

namespace detail {
typedef tbb::spin_mutex tbb_mutex_type;

void splat_impl( grid<float>& grid, const vec3& p, float splatValue, tbb_mutex_type& theMutex );
void splat_impl( grid<vec3>& grid, const vec3& p, const vec3& splatValue, tbb_mutex_type& theMutex );

void large_splat_impl( grid<float>& grid, const vec3& p, float splatValue, int radius, tbb_mutex_type& theMutex );
void large_splat_impl( grid<vec3>& grid, const vec3& p, const vec3& splatValue, int radius, tbb_mutex_type& theMutex );

template <class T, class ForwardIterator, class ParticleAccessor>
inline void splat_particles_impl( grid<T>& grid, const tbb::blocked_range<ForwardIterator>& range,
                                  const ParticleAccessor& pa, tbb_mutex_type& theMutex ) {
    vec3 p;
    T v;

    for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
        pa( *it, p, v );
        splat_impl( grid, p, v, theMutex );
    }
}

template <class T, class ForwardIterator, class ParticleAccessor>
inline void splat_weighted_particles_impl( grid<T>& theGrid, grid<float>& weightGrid,
                                           const tbb::blocked_range<ForwardIterator>& range, const ParticleAccessor& pa,
                                           tbb_mutex_type& theMutex ) {
    vec3 p;
    T v;
    float weight;

    for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
        pa( *it, p, v, weight );

        if( weight > 1e-5f ) {
            splat_impl( weightGrid, p, weight, theMutex );
            splat_impl( theGrid, p, v * weight, theMutex );
            // large_splat_impl( weightGrid, p, weight, 2, theMutex );
            // large_splat_impl( theGrid, p, v * weight, 2, theMutex );
        }
    }
}

template <class T>
inline void normalize_splats_impl( grid<T>& theGrid, const grid<float>& weightGrid,
                                   const tbb::blocked_range<int>& zRange ) {
    for( int z = zRange.begin(), zEnd = zRange.end(); z < zEnd; ++z ) {
        for( int y = 0, yEnd = theGrid.get_size( 1 ); y < yEnd; ++y ) {
            for( int x = 0, xEnd = theGrid.get_size( 0 ); x < xEnd; ++x ) {
                float weight = weightGrid.get_voxel( x, y, z );
                if( weight > 1e-5f )
                    theGrid.set_voxel( x, y, z, theGrid.get_voxel( x, y, z ) / weight );
            }
        }
    }
}
} // namespace detail

template <class T, class ForwardIterator, class ParticleAccessor>
inline void splat_particles( grid<T>& grid, ForwardIterator it, ForwardIterator itEnd, const ParticleAccessor& pa ) {
    detail::tbb_mutex_type theMutex;

    tbb::parallel_for( tbb::blocked_range<ForwardIterator>( it, itEnd ),
                       boost::bind( &detail::splat_particles_impl<T, ForwardIterator, ParticleAccessor>,
                                    boost::ref( grid ), _1, boost::cref( pa ), boost::ref( theMutex ) ),
                       tbb::auto_partitioner() );
}

template <class T, class ForwardIterator, class ParticleAccessor>
inline void splat_weighted_particles( grid<T>& theGrid, grid<float>& weightGrid, ForwardIterator it,
                                      ForwardIterator itEnd, const ParticleAccessor& pa ) {
    if( theGrid.get_min( 0 ) != weightGrid.get_min( 0 ) || theGrid.get_min( 1 ) != weightGrid.get_min( 1 ) ||
        theGrid.get_min( 2 ) != weightGrid.get_min( 2 ) || theGrid.get_max( 0 ) != weightGrid.get_max( 0 ) ||
        theGrid.get_max( 1 ) != weightGrid.get_max( 1 ) || theGrid.get_max( 2 ) != weightGrid.get_max( 2 ) ) {
        throw std::out_of_range( "splat_particles() - grid bounds mismatch" );
    }

    detail::tbb_mutex_type theMutex;

    tbb::parallel_for( tbb::blocked_range<ForwardIterator>( it, itEnd ),
                       boost::bind( &detail::splat_weighted_particles_impl<T, ForwardIterator, ParticleAccessor>,
                                    boost::ref( theGrid ), boost::ref( weightGrid ), _1, boost::cref( pa ),
                                    boost::ref( theMutex ) ),
                       tbb::auto_partitioner() );
}

template <class T>
inline void normalize_grid_from_weights( grid<T>& theGrid, const grid<float>& weightGrid ) {
    if( theGrid.get_min( 0 ) != weightGrid.get_min( 0 ) || theGrid.get_min( 1 ) != weightGrid.get_min( 1 ) ||
        theGrid.get_min( 2 ) != weightGrid.get_min( 2 ) || theGrid.get_max( 0 ) != weightGrid.get_max( 0 ) ||
        theGrid.get_max( 1 ) != weightGrid.get_max( 1 ) || theGrid.get_max( 2 ) != weightGrid.get_max( 2 ) ) {
        throw std::out_of_range( "splat_particles() - grid bounds mismatch" );
    }

    tbb::parallel_for(
        tbb::blocked_range<int>( 0, theGrid.get_size( 2 ) ),
        boost::bind( &detail::normalize_splats_impl<T>, boost::ref( theGrid ), boost::cref( weightGrid ), _1 ),
        tbb::auto_partitioner() );
}

} // namespace ember
