// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/grid.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 4100 )
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#pragma warning( pop )

#include <boost/bind.hpp>
#include <boost/call_traits.hpp>

using frantic::volumetrics::field_interface;

namespace ember {

template <class T>
class grid_helper {
  public:
    inline static T trilerp( const grid<T>& grid, const vec3& pos );

    template <class MutexType>
    inline static void trilerp_splat( grid<T>& grid, const vec3& pos,
                                      typename boost::call_traits<T>::param_type splatValue, MutexType& theMutex );

    template <class MutexType>
    inline static void trilerp_large_splat( grid<T>& grid, const vec3& pos,
                                            typename boost::call_traits<T>::param_type splatValue, int radius,
                                            MutexType& theMutex );

    inline static void assign_range( const tbb::blocked_range<int>& range, grid<T>& grid,
                                     const field_interface& field );
};

template <class T>
T grid_helper<T>::trilerp( const grid<T>& grid, const vec3& pos ) {
    float vpos[] = { pos.x / grid.get_spacing() - 0.5f, pos.y / grid.get_spacing() - 0.5f,
                     pos.z / grid.get_spacing() - 0.5f };
    float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };
    float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };
    int ipos[] = { (int)fpos[0] - grid.get_min( 0 ), (int)fpos[1] - grid.get_min( 1 ),
                   (int)fpos[2] - grid.get_min( 2 ) };

    int index = ipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * ipos[2] );

    T result( 0.f );
    if( static_cast<unsigned>( ipos[2] ) < static_cast<unsigned>( grid.get_size( 2 ) ) ) {
        float zWeight = 1.f - alpha[2];
        if( static_cast<unsigned>( ipos[1] ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * ( 1.f - alpha[1] );
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index];
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += alpha[0] * yzWeight * grid.m_storage[index + 1];
        }
        if( static_cast<unsigned>( ipos[1] + 1 ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * alpha[1];
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )];
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1];
        }
    }
    if( static_cast<unsigned>( ipos[2] + 1 ) < static_cast<unsigned>( grid.get_size( 2 ) ) ) {
        index += grid.get_size( 0 ) * grid.get_size( 1 );

        float zWeight = alpha[2];
        if( static_cast<unsigned>( ipos[1] ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * ( 1.f - alpha[1] );
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index];
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += alpha[0] * yzWeight * grid.m_storage[index + 1];
        }
        if( static_cast<unsigned>( ipos[1] + 1 ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * alpha[1];
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )];
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                result += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1];
        }
    }

    return result;
}

template <class T>
void grid_helper<T>::assign_range( const tbb::blocked_range<int>& range, grid<T>& grid, const field_interface& field ) {
    std::vector<T>::iterator it = grid.m_storage.begin() + range.begin() * grid.get_size( 0 ) * grid.get_size( 1 );

    vec3 p;

    for( int z = range.begin(); z < range.end(); ++z ) {
        p.z = ( (float)( z + grid.get_min( 2 ) ) + 0.5f ) * grid.get_spacing();

        for( int y = 0, yEnd = grid.get_size( 1 ); y < yEnd; ++y ) {
            p.y = ( (float)( y + grid.get_min( 1 ) ) + 0.5f ) * grid.get_spacing();

            for( int x = 0, xEnd = grid.get_size( 0 ); x < xEnd; ++x, ++it ) {
                p.x = ( (float)( x + grid.get_min( 0 ) ) + 0.5f ) * grid.get_spacing();

                field.evaluate_field( &*it, p );
            }
        }
    }
}

template <class T>
template <class MutexType>
inline void grid_helper<T>::trilerp_splat( grid<T>& grid, const vec3& pos,
                                           typename boost::call_traits<T>::param_type splatValue,
                                           MutexType& theMutex ) {
    float vpos[] = { pos.x / grid.get_spacing() - 0.5f, pos.y / grid.get_spacing() - 0.5f,
                     pos.z / grid.get_spacing() - 0.5f };

    float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };

    float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };

    int ipos[] = { static_cast<int>( fpos[0] ) - grid.get_min( 0 ), static_cast<int>( fpos[1] ) - grid.get_min( 1 ),
                   static_cast<int>( fpos[2] ) - grid.get_min( 2 ) };

    int index = ipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * ipos[2] );

    typename MutexType::scoped_lock theLock( theMutex );

    if( static_cast<unsigned>( ipos[2] ) < static_cast<unsigned>( grid.get_size( 2 ) ) ) {
        float zWeight = 1.f - alpha[2];
        if( static_cast<unsigned>( ipos[1] ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * ( 1.f - alpha[1] );
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index] += ( 1.f - alpha[0] ) * yzWeight * splatValue;
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + 1] += ( alpha[0] ) * yzWeight * splatValue;
        }
        if( static_cast<unsigned>( ipos[1] + 1 ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * alpha[1];
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + grid.get_size( 0 )] += ( 1.f - alpha[0] ) * yzWeight * splatValue;
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + grid.get_size( 0 ) + 1] += ( alpha[0] ) * yzWeight * splatValue;
        }
    }
    if( static_cast<unsigned>( ipos[2] + 1 ) < static_cast<unsigned>( grid.get_size( 2 ) ) ) {
        index += grid.get_size( 0 ) * grid.get_size( 1 );

        float zWeight = alpha[2];
        if( static_cast<unsigned>( ipos[1] ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * ( 1.f - alpha[1] );
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index] += ( 1.f - alpha[0] ) * yzWeight * splatValue;
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + 1] += ( alpha[0] ) * yzWeight * splatValue;
        }
        if( static_cast<unsigned>( ipos[1] + 1 ) < static_cast<unsigned>( grid.get_size( 1 ) ) ) {
            float yzWeight = zWeight * alpha[1];
            if( static_cast<unsigned>( ipos[0] ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + grid.get_size( 0 )] += ( 1.f - alpha[0] ) * yzWeight * splatValue;
            if( static_cast<unsigned>( ipos[0] + 1 ) < static_cast<unsigned>( grid.get_size( 0 ) ) )
                grid.m_storage[index + grid.get_size( 0 ) + 1] += ( alpha[0] ) * yzWeight * splatValue;
        }
    }
}

// Not inline on purpose, because it makes use of alloca().
template <class T>
template <class MutexType>
void grid_helper<T>::trilerp_large_splat( grid<T>& grid, const vec3& pos,
                                          typename boost::call_traits<T>::param_type splatValue, int radius,
                                          MutexType& theMutex ) {
    float vpos[] = { pos.x / grid.get_spacing() - 0.5f, pos.y / grid.get_spacing() - 0.5f,
                     pos.z / grid.get_spacing() - 0.5f };

    float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };

    float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };

    int ipos[] = { static_cast<int>( fpos[0] ) - grid.get_min( 0 ), static_cast<int>( fpos[1] ) - grid.get_min( 1 ),
                   static_cast<int>( fpos[2] ) - grid.get_min( 2 ) };

    int index = ipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * ipos[2] );

    float fradius = static_cast<float>( radius );

    // Compensate to ensure total weight == 1.0
    // int(-r,r){r - |x-a|}dx = int(-r,0){r+x-a}dx  +  int(0,r){r-x+a}dx
    //                        = (rx+xx/2-ax)|-r,0   +  (rx-xx/2+ax)|0,r
    //                        = rr-rr/2-ar          +  rr-rr/2+ar
    //                        = rr
    float denom = fradius * fradius;
    denom = denom * denom * denom;

    std::size_t bufferSize = static_cast<std::size_t>( 2 * radius );
    float* xWeights = static_cast<float*>(
        alloca( 2 * sizeof( float ) * bufferSize ) ); // Double the size and store the yWeights after the xWeights.
    float* yWeights = xWeights + bufferSize;

    // Precompute the x & y weights, since they are reused many times
    // TODO: This seems like a candidate for SSE.
    // TODO: We could delay computing these until we find out that we actually are splatting into the valid grid region.
    for( int x = -radius + 1, i = 0; x <= radius; ++x, ++i ) {
        xWeights[i] = ( fradius - fabsf( static_cast<float>( x ) - alpha[0] ) );
        yWeights[i] = ( fradius - fabsf( static_cast<float>( x ) - alpha[1] ) );
    }

    for( int z = -radius + 1; z <= radius; ++z ) {
        if( static_cast<unsigned>( ipos[2] + z ) >= static_cast<unsigned>( grid.get_size( 2 ) ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );
            continue;
        }

        float zWeight = ( fradius - fabsf( static_cast<float>( z ) - alpha[2] ) );

        float* pWeights = yWeights;
        for( int y = -radius + 1; y <= radius; ++y, ++pWeights ) {
            if( static_cast<unsigned>( ipos[1] + y ) >= static_cast<unsigned>( grid.get_size( 1 ) ) ) {
                index += grid.get_size( 0 );
                continue;
            }

            float yzWeight = ( *pWeights ) * zWeight;

            float* pWeights = xWeights;
            for( int x = -radius + 1; x <= radius; ++x, ++pWeights, ++index ) {
                if( static_cast<unsigned>( ipos[0] + x ) < static_cast<unsigned>( grid.get_size( 0 ) ) ) {
                    float weight = ( ( *pWeights ) * yzWeight ) / denom;
                    T weightedValue = weight * splatValue;

                    // This is a spin_mutex, so it should be cheap to acquire.
                    typename MutexType::scoped_lock theLock( theMutex );

                    grid.m_storage[index] += weightedValue;
                }
            } // for( x )
        }     // for( y )
    }         // for( z )
}

void assign( grid<float>& grid, const frantic::volumetrics::field_interface& field ) {
    const frantic::channels::channel_map& fieldMap = field.get_channel_map();

    if( fieldMap.channel_count() != 1 )
        throw std::runtime_error( "Unable to assign grid<float> from field_interface due to multiple channels" );

    if( fieldMap[0].arity() != 1 || fieldMap[0].data_type() != frantic::channels::data_type_float32 )
        throw std::runtime_error(
            "Unable to assign grid<float> from field_interface due to conflicting types. Found: " +
            frantic::strings::to_string(
                frantic::channels::channel_data_type_str( fieldMap[0].arity(), fieldMap[0].data_type() ) ) +
            ", expected: " +
            frantic::strings::to_string( frantic::channels::channel_data_type_traits<float>::type_str() ) );

    tbb::parallel_for( tbb::blocked_range<int>( 0, grid.get_size( 2 ), 10 ),
                       boost::bind( &grid_helper<float>::assign_range, _1, boost::ref( grid ), boost::cref( field ) ),
                       tbb::auto_partitioner() );
}

void assign( grid<vec3>& grid, const frantic::volumetrics::field_interface& field ) {
    const frantic::channels::channel_map& fieldMap = field.get_channel_map();

    if( fieldMap.channel_count() != 1 )
        throw std::runtime_error( "Unable to assign grid<vec3> from field_interface due to multiple channels" );

    if( fieldMap[0].arity() != 3 || fieldMap[0].data_type() != frantic::channels::data_type_float32 )
        throw std::runtime_error(
            "Unable to assign grid<vec3> from field_interface due to conflicting types. Found: " +
            frantic::strings::to_string(
                frantic::channels::channel_data_type_str( fieldMap[0].arity(), fieldMap[0].data_type() ) ) +
            ", expected: " +
            frantic::strings::to_string( frantic::channels::channel_data_type_traits<vec3>::type_str() ) );

    tbb::parallel_for( tbb::blocked_range<int>( 0, grid.get_size( 2 ), 10 ),
                       boost::bind( &grid_helper<vec3>::assign_range, _1, boost::ref( grid ), boost::cref( field ) ),
                       tbb::auto_partitioner() );
}

float trilerp( const grid<float>& grid, const vec3& pos ) { return grid_helper<float>::trilerp( grid, pos ); }

vec3 trilerp( const grid<vec3>& grid, const vec3& pos ) { return grid_helper<vec3>::trilerp( grid, pos ); }

namespace detail {
void splat_impl( grid<float>& grid, const vec3& p, float splatValue, tbb_mutex_type& theMutex ) {
    grid_helper<float>::trilerp_splat( grid, p, splatValue, theMutex );
}

void splat_impl( grid<vec3>& grid, const vec3& p, const vec3& splatValue, tbb_mutex_type& theMutex ) {
    grid_helper<vec3>::trilerp_splat( grid, p, splatValue, theMutex );
}

void large_splat_impl( grid<float>& grid, const vec3& p, float splatValue, int radius, tbb_mutex_type& theMutex ) {
    grid_helper<float>::trilerp_large_splat( grid, p, splatValue, radius, theMutex );
}

void large_splat_impl( grid<vec3>& grid, const vec3& p, const vec3& splatValue, int radius, tbb_mutex_type& theMutex ) {
    grid_helper<vec3>::trilerp_large_splat( grid, p, splatValue, radius, theMutex );
}
} // namespace detail

template <class T>
bool discretized_field<T>::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    *reinterpret_cast<T*>( dest ) = grid_helper<T>::trilerp( m_storage, pos );
    return true;
}

template bool discretized_field<float>::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;
template bool discretized_field<vec3>::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const;

//
//
//
//
// struct staggered_grid_helper{
//	static void sample( staggered_grid& grid, const field_interface& field, const tbb::blocked_range<int>& range );
//};
//
// void staggered_grid::sample( const field_interface& field ){
//	if( !field.is_vector() )
//		return;
//
//	tbb::parallel_for(
//		tbb::blocked_range<int>( 0, get_size(2), 10 ),
//		boost::bind( &staggered_grid_helper::sample, boost::ref( *this ), boost::ref( field ), _1 ),
//		tbb::auto_partitioner() );
//}
//
// void staggered_grid_helper::sample( staggered_grid& grid, const field_interface& field, const
// tbb::blocked_range<int>& range ){ 	int index = grid.get_voxel_index( 0, 0, range.begin() );
//
//	float off = 0.5f * grid.get_spacing();
//
//	for( int z = range.begin(); z < range.end(); ++z ){
//		float pz = ((float)(z + grid.get_min(2)) + 0.5f) * grid.get_spacing();
//
//		for( int y = 0, yEnd = grid.get_size(1); y < yEnd; ++y ){
//			float py = ((float)(y + grid.get_min(1)) + 0.5f) * grid.get_spacing();
//
//			for( int x = 0, xEnd = grid.get_size(0); x < xEnd; ++x, ++index ){
//				float px = ((float)(x + grid.get_min(0)) + 0.5f) * grid.get_spacing();
//
//				float fx = field.get_vector( vec3(px - off, py, pz) ).x;
//				float fy = field.get_vector( vec3(px, py - off, pz) ).y;
//				float fz = field.get_vector( vec3(px, py, pz - off) ).z;
//
//				grid[index] = vec3( fx, fy, fz );
//			}
//		}
//	}
//}
//
// template <class T>
// struct regular_grid_helper{
//	static void sample( grid_cache<T>& grid, const field_interface& field, const tbb::blocked_range<int>& range);
//};
//
// template <class T>
// void grid_cache<T>::sample( const field_interface& field ){
//	tbb::parallel_for(
//		tbb::blocked_range<int>( 0, get_size(2), 10 ),
//		boost::bind( &regular_grid_helper<T>::sample, boost::ref( *this ), boost::ref( field ), _1 ),
//		tbb::auto_partitioner() );
//}
//
////Explicit instantiation of sample routines.
// template void grid_cache<float>::sample( const field_interface& );
// template void grid_cache<vec3>::sample( const field_interface& );
//
// template <class T>
// void regular_grid_helper<T>::sample( grid_cache<T>& grid, const field_interface& field, const
// tbb::blocked_range<int>& range)
//{
//		grid_cache<T>::iterator it = grid.begin() + range.begin() * grid.get_size(0) * grid.get_size(1);
//
//		frantic::graphics::vector3f p;
//
//		for( int z = range.begin(); z < range.end(); ++z ){
//			p.z = ((float)(z + grid.get_min(2)) + 0.5f) * grid.get_spacing();
//			for( int y = 0; y < grid.get_size(1); ++y ){
//				p.y = ((float)(y + grid.get_min(1)) + 0.5f) * grid.get_spacing();
//				for( int x = 0; x < grid.get_size(0); ++x, ++it ){
//					p.x = ((float)(x + grid.get_min(0)) + 0.5f) * grid.get_spacing();
//
//					*it = detail::grid_cache_helper<T>::do_get( field, p );
//				}
//			}
//		}
//	}

} // namespace ember
