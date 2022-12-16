// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/data_types.hpp>
#include <frantic/logging/progress_logger.hpp>
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

class staggered_grid_helper;

class staggered_grid {
  public:
    staggered_grid();
    staggered_grid( const int ( &bounds )[6], float spacing );

    void swap( staggered_grid& rhs );

    float get_spacing() const;
    int get_size( int axis ) const;
    int get_min( int axis ) const;
    int get_max( int axis ) const;

    void get_staggered_values( int x, int y, int z, float ( &outValues )[6] ) const;
    // const float (&get_staggered_voxel( int x, int y, int z ) const)[3];
    // float (&get_staggered_voxel( int x, int y, int z ))[3];
    // void set_staggered_voxel( int x, int y, int z, const float (&values)[3] );
    const vec3& get_staggered_voxel( int x, int y, int z ) const;
    vec3& get_staggered_voxel( int x, int y, int z );
    void set_staggered_voxel( int x, int y, int z, const vec3& v );

    const std::vector<vec3>& get_data() const;

    friend class staggered_grid_helper;

  private:
    int m_min[3];
    int m_size[3];
    float m_spacing;

    std::vector<vec3> m_storage;
};

void assign( staggered_grid& grid, const frantic::volumetrics::field_interface& field,
             const frantic::tstring& channelName, frantic::logging::progress_logger& progress );

vec3 trilerp( const staggered_grid& grid, const vec3& pos );

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
template <class ForwardIterator, class ParticleAccessor>
void splat_particles( staggered_grid& grid, ForwardIterator it, ForwardIterator itEnd, const ParticleAccessor& pa );

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
template <class ForwardIterator, class ParticleAccessor>
void splat_weighted_particles( staggered_grid& theGrid, staggered_grid& weightGrid, ForwardIterator it,
                               ForwardIterator itEnd, const ParticleAccessor& pa );

/**
 * Normalizes the values stored on a grid by dividing out the accumulated weights from particle splats. Since splatted
 * particles are added together, we need to divide out the relative amount of particles that were splatted to each grid
 * point, otherwise the data would behave additively.
 * @param[in|out] theGrid The grid to normalize
 * @param weightGrid The grid storing the accumulated weights splatted to each voxel.
 */
void normalize_grid_from_weights( staggered_grid& theGrid, const staggered_grid& weightGrid );

inline staggered_grid::staggered_grid() {
    m_min[0] = m_min[1] = m_min[2] = m_size[0] = m_size[1] = m_size[2] = 0;
    m_spacing = 1.f;
}

inline staggered_grid::staggered_grid( const int ( &bounds )[6], float spacing ) {
    m_min[0] = bounds[0];
    m_min[1] = bounds[2];
    m_min[2] = bounds[4];
    m_size[0] = ( std::max )( 0, bounds[1] - bounds[0] + 1 );
    m_size[1] = ( std::max )( 0, bounds[3] - bounds[2] + 1 );
    m_size[2] = ( std::max )( 0, bounds[5] - bounds[4] + 1 );
    m_spacing = spacing;
    // m_storage.resize( 3 * (std::size_t)( m_size[0] * m_size[1] * m_size[2] ) );
    m_storage.resize( ( std::size_t )( m_size[0] * m_size[1] * m_size[2] ) );
}

inline void staggered_grid::swap( staggered_grid& rhs ) {
    std::swap( m_min[0], rhs.m_min[0] );
    std::swap( m_min[1], rhs.m_min[1] );
    std::swap( m_min[2], rhs.m_min[2] );
    std::swap( m_size[0], rhs.m_size[0] );
    std::swap( m_size[1], rhs.m_size[1] );
    std::swap( m_size[2], rhs.m_size[2] );
    std::swap( m_spacing, rhs.m_spacing );

    m_storage.swap( rhs.m_storage );
}

inline float staggered_grid::get_spacing() const { return m_spacing; }

inline int staggered_grid::get_size( int axis ) const { return m_size[axis]; }

inline int staggered_grid::get_min( int axis ) const { return m_min[axis]; }

inline int staggered_grid::get_max( int axis ) const { return get_min( axis ) + get_size( axis ); }

inline void staggered_grid::get_staggered_values( int x, int y, int z, float ( &outValues )[6] ) const {
    // int index = 3 * (x + m_size[0] * ( y + m_size[1] * z ));

    // outValues[0] = m_storage[ index ];                                      //-x
    // outValues[2] = m_storage[ index + 1 ];                                  //-y
    // outValues[4] = m_storage[ index + 2 ];                                  //-z
    // outValues[1] = m_storage[ index + 3 ];                                  //+x
    // outValues[3] = m_storage[ index + 3 * get_size(0) + 1 ];                //+y
    // outValues[5] = m_storage[ index + 3 * get_size(0) * get_size(1) + 2 ];  //+z
    int index = x + m_size[0] * ( y + m_size[1] * z );

    outValues[0] = m_storage[index].x;                                 //-x
    outValues[2] = m_storage[index].y;                                 //-y
    outValues[4] = m_storage[index].z;                                 //-z
    outValues[1] = m_storage[index + 1].x;                             //+x
    outValues[3] = m_storage[index + get_size( 0 )].y;                 //+y
    outValues[5] = m_storage[index + get_size( 0 ) * get_size( 1 )].z; //+z
}

// inline void staggered_grid::get_staggered_voxel( int x, int y, int z, float (&outValues)[3] ) const {
//	int index = 3 * (x + m_size[0] * ( y + m_size[1] * z ));
//
//	outValues[0] = m_storage[index];
//	outValues[1] = m_storage[index+1];
//	outValues[2] = m_storage[index+2];
// }

// inline const float (&staggered_grid::get_staggered_voxel( int x, int y, int z ) const)[3] {
//	int index = x + m_size[0] * ( y + m_size[1] * z );
//
//	return *reinterpret_cast<const float(*)[3]>( &m_storage[ 3 * index ] );
// }
//
// inline float (&staggered_grid::get_staggered_voxel( int x, int y, int z ))[3] {
//	int index = x + m_size[0] * ( y + m_size[1] * z );
//
//	return *reinterpret_cast<float(*)[3]>( &m_storage[ 3 * index ] );
// }
//
// inline void staggered_grid::set_staggered_voxel( int x, int y, int z, const float (&values)[3] ){
//	int index = 3 * (x + m_size[0] * ( y + m_size[1] * z ));
//
//	m_storage[index] = values[0];
//	m_storage[index+1] = values[1];
//	m_storage[index+2] = values[2];
// }

inline const vec3& staggered_grid::get_staggered_voxel( int x, int y, int z ) const {
    return m_storage[x + get_size( 0 ) * ( y + get_size( 1 ) * z )];
}

inline vec3& staggered_grid::get_staggered_voxel( int x, int y, int z ) {
    return m_storage[x + get_size( 0 ) * ( y + get_size( 1 ) * z )];
}

inline void staggered_grid::set_staggered_voxel( int x, int y, int z, const vec3& v ) {
    m_storage[x + get_size( 0 ) * ( y + get_size( 1 ) * z )] = v;
}

inline const std::vector<vec3>& staggered_grid::get_data() const { return m_storage; }

class staggered_discretized_field : public frantic::volumetrics::field_interface {
  public:
    staggered_discretized_field( const int ( &bounds )[6], float spacing,
                                 const frantic::tstring& valueName = _T("Value") );

    virtual ~staggered_discretized_field();

    virtual const frantic::channels::channel_map& get_channel_map() const;

    virtual bool evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

    inline staggered_grid& get_grid();

    inline const staggered_grid& get_grid() const;

    void assign( const frantic::volumetrics::field_interface& field,
                 frantic::logging::progress_logger* progress = NULL );

    void assign( const frantic::volumetrics::field_interface& field, const frantic::tstring& channelName,
                 frantic::logging::progress_logger* progress = NULL );

  private:
    staggered_grid m_storage;

    frantic::channels::channel_map m_channelMap;
};

inline staggered_grid& staggered_discretized_field::get_grid() { return m_storage; }

inline const staggered_grid& staggered_discretized_field::get_grid() const { return m_storage; }

namespace staggered_grid_detail {
typedef tbb::spin_mutex tbb_mutex_type;

void splat_impl( staggered_grid& grid, const vec3& p, const vec3& splatValue, tbb_mutex_type& theMutex );

template <class ForwardIterator, class ParticleAccessor>
inline void splat_particles_impl( staggered_grid& grid, const tbb::blocked_range<ForwardIterator>& range,
                                  const ParticleAccessor& pa, tbb_mutex_type& theMutex ) {
    vec3 p, v;

    for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
        pa( *it, p, v );
        splat_impl( grid, p, v, theMutex );
    }
}

template <class ForwardIterator, class ParticleAccessor>
inline void splat_weighted_particles_impl( staggered_grid& theGrid, staggered_grid& weightGrid,
                                           const tbb::blocked_range<ForwardIterator>& range, const ParticleAccessor& pa,
                                           tbb_mutex_type& theMutex ) {
    vec3 p, v;
    float weight;

    for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
        pa( *it, p, v, weight );

        if( weight > 1e-5f ) {
            splat_impl( theGrid, p, v * weight, theMutex );
            splat_impl( weightGrid, p, vec3( weight, weight, weight ), theMutex );
        }
    }
}

inline void normalize_splats_impl( staggered_grid& theGrid, const staggered_grid& weightGrid,
                                   const tbb::blocked_range<int>& zRange ) {
    for( int z = zRange.begin(), zEnd = zRange.end(); z < zEnd; ++z ) {
        for( int y = 0, yEnd = theGrid.get_size( 1 ); y < yEnd; ++y ) {
            for( int x = 0, xEnd = theGrid.get_size( 0 ); x < xEnd; ++x ) {
                const vec3& weight = weightGrid.get_staggered_voxel( x, y, z );

                vec3 newVal = theGrid.get_staggered_voxel( x, y, z );
                if( weight.x > 1e-5f )
                    newVal.x /= weight.x;
                else
                    newVal.x = 0.f;

                if( weight.y > 1e-5f )
                    newVal.y /= weight.y;
                else
                    newVal.y = 0.f;

                if( weight.z > 1e-5f )
                    newVal.z /= weight.z;
                else
                    newVal.z = 0.f;

                theGrid.set_staggered_voxel( x, y, z, newVal );
            }
        }
    }
}
} // namespace staggered_grid_detail

template <class ForwardIterator, class ParticleAccessor>
inline void splat_particles( staggered_grid& grid, ForwardIterator it, ForwardIterator itEnd,
                             const ParticleAccessor& pa ) {
    detail::tbb_mutex_type theMutex;

    tbb::parallel_for( tbb::blocked_range<ForwardIterator>( it, itEnd ),
                       boost::bind( &staggered_grid_detail::splat_particles_impl<ForwardIterator, ParticleAccessor>,
                                    boost::ref( grid ), _1, boost::cref( pa ), boost::ref( theMutex ) ),
                       tbb::auto_partitioner() );
}

template <class ForwardIterator, class ParticleAccessor>
inline void splat_weighted_particles( staggered_grid& theGrid, staggered_grid& weightGrid, ForwardIterator it,
                                      ForwardIterator itEnd, const ParticleAccessor& pa ) {
    if( theGrid.get_min( 0 ) != weightGrid.get_min( 0 ) || theGrid.get_min( 1 ) != weightGrid.get_min( 1 ) ||
        theGrid.get_min( 2 ) != weightGrid.get_min( 2 ) || theGrid.get_max( 0 ) != weightGrid.get_max( 0 ) ||
        theGrid.get_max( 1 ) != weightGrid.get_max( 1 ) || theGrid.get_max( 2 ) != weightGrid.get_max( 2 ) ) {
        throw std::out_of_range( "splat_weighted_particles() - grid bounds mismatch" );
    }

    detail::tbb_mutex_type theMutex;

    tbb::parallel_for(
        tbb::blocked_range<ForwardIterator>( it, itEnd ),
        boost::bind( &staggered_grid_detail::splat_weighted_particles_impl<ForwardIterator, ParticleAccessor>,
                     boost::ref( theGrid ), boost::ref( weightGrid ), _1, boost::cref( pa ), boost::ref( theMutex ) ),
        tbb::auto_partitioner() );
}

inline void normalize_grid_from_weights( staggered_grid& theGrid, const staggered_grid& weightGrid ) {
    if( theGrid.get_min( 0 ) != weightGrid.get_min( 0 ) || theGrid.get_min( 1 ) != weightGrid.get_min( 1 ) ||
        theGrid.get_min( 2 ) != weightGrid.get_min( 2 ) || theGrid.get_max( 0 ) != weightGrid.get_max( 0 ) ||
        theGrid.get_max( 1 ) != weightGrid.get_max( 1 ) || theGrid.get_max( 2 ) != weightGrid.get_max( 2 ) ) {
        throw std::out_of_range( "normalize_grid_from_weights() - grid bounds mismatch" );
    }

    tbb::parallel_for( tbb::blocked_range<int>( 0, theGrid.get_size( 2 ) ),
                       boost::bind( &staggered_grid_detail::normalize_splats_impl, boost::ref( theGrid ),
                                    boost::cref( weightGrid ), _1 ),
                       tbb::auto_partitioner() );
}

} // namespace ember
