// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/staggered_grid.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 4100 )
#include <tbb/atomic.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#pragma warning( pop )

#include <boost/bind.hpp>

namespace ember {

namespace {
struct progress_accumulator {
    tbb::atomic<std::size_t> m_accum;
    std::size_t m_progressTotal;
    std::size_t m_progressStep;

    frantic::logging::progress_logger* m_pLogger;

    void add_progress( std::size_t progressAmount ) {
        std::size_t oldVal = m_accum.fetch_and_add( progressAmount );

        m_pLogger->update_progress( static_cast<long long>( oldVal + progressAmount ),
                                    static_cast<long long>( m_progressTotal ) );
    }
};
} // namespace

staggered_discretized_field::staggered_discretized_field( const int ( &bounds )[6], float spacing,
                                                          const frantic::tstring& valueName )
    : m_storage( bounds, spacing ) {
    m_channelMap.define_channel<vec3>( valueName );
    m_channelMap.end_channel_definition();
}

staggered_discretized_field::~staggered_discretized_field() {}

const frantic::channels::channel_map& staggered_discretized_field::get_channel_map() const { return m_channelMap; }

class staggered_grid_helper {
  public:
    static vec3 trilerp( const staggered_grid& grid, const vec3& pos );

    template <class MutexType>
    static void trilerp_splat( staggered_grid& grid, const vec3& pos, const vec3& splatValue, MutexType& theMutex );

    static void assign_range( const tbb::blocked_range<int>& range, staggered_grid& grid,
                              const frantic::volumetrics::field_interface& field,
                              const frantic::channels::channel_accessor<vec3>& chAcc, progress_accumulator& progress );
};

// inline float get_staggered_value( const std::vector<float>& storage, int index, int axis ){
//	return storage[3 * index + axis];
// }
//
// vec3 staggered_grid_helper::trilerp( const staggered_grid& grid, const vec3& pos ){
//	float result[] = { 0, 0, 0 };
//
//	float svpos[] = { pos.x / grid.get_spacing(), pos.y / grid.get_spacing(), pos.z / grid.get_spacing() };
//	float sfpos[] = { std::floor( svpos[0] ), std::floor( svpos[1] ), std::floor( svpos[2] ) };
//	float salpha[] = { svpos[0] - sfpos[0], svpos[1] - sfpos[1], svpos[2] - sfpos[2] };
//	int sipos[] = { (int)sfpos[0] - grid.get_min(0), (int)sfpos[1] - grid.get_min(1), (int)sfpos[2] -
//grid.get_min(2) };
//
//	float vpos[] = { svpos[0] - 0.5f, svpos[1] - 0.5f, svpos[2] - 0.5f };
//	float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };
//	float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };
//	int ipos[] = { (int)fpos[0] - grid.get_min(0), (int)fpos[1] - grid.get_min(1), (int)fpos[2] - grid.get_min(2) };
//
//	{ //x
//		int index = sipos[0] + grid.get_size(0) * ( ipos[1] + grid.get_size(1) * ipos[2] );
//
//		if( ipos[2] >= 0 && ipos[2] < grid.get_size(2) ){
//			float zWeight = 1.f - alpha[2];
//			if( ipos[1] >= 0 && ipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - alpha[1]);
//				if( sipos[0] >= 0 && sipos[0] < grid.get_size(0) )
//					result[0] += (1.f - salpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index,
//0 ); 				if( sipos[0]+1 >= 0 && sipos[0]+1 < grid.get_size(0) ) 					result[0] += salpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 0 );
//			}
//			if( ipos[1]+1 >= 0 && ipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * alpha[1];
//				if( sipos[0] >= 0 && sipos[0] < grid.get_size(0) )
//					result[0] += (1.f - salpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 0 ); 				if( sipos[0]+1 >= 0 && sipos[0]+1 < grid.get_size(0) ) 					result[0] += salpha[0] * yzWeight
//* get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 0 );
//			}
//		}
//		if( ipos[2]+1 >= 0 && ipos[2]+1 < grid.get_size(2) ){
//			index += grid.get_size(0)*grid.get_size(1);
//
//			float zWeight = alpha[2];
//			if( ipos[1] >= 0 && ipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - alpha[1]);
//				if( sipos[0] >= 0 && sipos[0] < grid.get_size(0) )
//					result[0] += (1.f - salpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index,
//0 ); 				if( sipos[0]+1 >= 0 && sipos[0]+1 < grid.get_size(0) ) 					result[0] += salpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 0 );
//			}
//			if( ipos[1]+1 >= 0 && ipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * alpha[1];
//				if( sipos[0] >= 0 && sipos[0] < grid.get_size(0) )
//					result[0] += (1.f - salpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 0 ); 				if( sipos[0]+1 >= 0 && sipos[0]+1 < grid.get_size(0) ) 					result[0] += salpha[0] * yzWeight
//* get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 0 );
//			}
//		}
//	}
//
//	{ //y
//		int index = ipos[0] + grid.get_size(0) * ( sipos[1] + grid.get_size(1) * ipos[2] );
//
//		if( ipos[2] >= 0 && ipos[2] < grid.get_size(2) ){
//			float zWeight = 1.f - alpha[2];
//			if( sipos[1] >= 0 && sipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - salpha[1]);
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[1] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index, 1
//); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[1] += alpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 1 );
//			}
//			if( sipos[1]+1 >= 0 && sipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * salpha[1];
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[1] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 1 ); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[1] += alpha[0] * yzWeight *
//get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 1 );
//			}
//		}
//		if( ipos[2]+1 >= 0 && ipos[2]+1 < grid.get_size(2) ){
//			index += grid.get_size(0)*grid.get_size(1);
//
//			float zWeight = alpha[2];
//			if( sipos[1] >= 0 && sipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - salpha[1]);
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[1] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index, 1
//); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[1] += alpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 1 );
//			}
//			if( sipos[1]+1 >= 0 && sipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * salpha[1];
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[1] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 1 ); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[1] += alpha[0] * yzWeight *
//get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 1 );
//			}
//		}
//	}
//
//	{ //z
//		int index = ipos[0] + grid.get_size(0) * ( ipos[1] + grid.get_size(1) * sipos[2] );
//
//		if( sipos[2] >= 0 && sipos[2] < grid.get_size(2) ){
//			float zWeight = 1.f - salpha[2];
//			if( ipos[1] >= 0 && ipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - alpha[1]);
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[2] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index, 2
//); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[2] += alpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 2 );
//			}
//			if( ipos[1]+1 >= 0 && ipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * alpha[1];
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[2] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 2 ); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[2] += alpha[0] * yzWeight *
//get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 2 );
//			}
//		}
//		if( sipos[2]+1 >= 0 && sipos[2]+1 < grid.get_size(2) ){
//			index += grid.get_size(0)*grid.get_size(1);
//
//			float zWeight = salpha[2];
//			if( ipos[1] >= 0 && ipos[1] < grid.get_size(1) ){
//				float yzWeight = zWeight * (1.f - alpha[1]);
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[2] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage, index, 2
//); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[2] += alpha[0] * yzWeight * get_staggered_value(
//grid.m_storage, index+1, 2 );
//			}
//			if( ipos[1]+1 >= 0 && ipos[1]+1 < grid.get_size(1) ){
//				float yzWeight = zWeight * alpha[1];
//				if( ipos[0] >= 0 && ipos[0] < grid.get_size(0) )
//					result[2] += (1.f - alpha[0]) * yzWeight * get_staggered_value( grid.m_storage,
//index+grid.get_size(0), 2 ); 				if( ipos[0]+1 >= 0 && ipos[0]+1 < grid.get_size(0) ) 					result[2] += alpha[0] * yzWeight *
//get_staggered_value( grid.m_storage, index+grid.get_size(0)+1, 2 );
//			}
//		}
//	}
//
//	return frantic::graphics::vector3f( result[0], result[1], result[2] );
// }
//
// void staggered_grid_helper::assign_range( const tbb::blocked_range<int>& range, staggered_grid& grid, const
// frantic::volumetrics::field_interface& field ){ 	int index = range.begin() * grid.get_size(0) * grid.get_size(1);
//
//	float off = 0.5f * grid.get_spacing();
//	float temp[3][3];
//
//	for( int z = range.begin(); z < range.end(); ++z ){
//		float pz = ((float)(z + grid.get_min(2)) + 0.5f) * grid.get_spacing();
//
//		for( int y = 0, yEnd = grid.get_size(1); y < yEnd; ++y ){
//			float py = ((float)(y + grid.get_min(1)) + 0.5f) * grid.get_spacing();
//
//			for( int x = 0, xEnd = grid.get_size(0); x < xEnd; ++x, index += 3 ){
//				float px = ((float)(x + grid.get_min(0)) + 0.5f) * grid.get_spacing();
//
//				field.evaluate_field( temp[0], vec3(px - off, py, pz) );
//				field.evaluate_field( temp[1], vec3(px, py - off, pz) );
//				field.evaluate_field( temp[2], vec3(px - off, py, pz - off) );
//
//				grid.m_storage[index] = temp[0][0];
//				grid.m_storage[index+1] = temp[1][1];
//				grid.m_storage[index+2] = temp[2][2];
//			}
//		}
//	}
// }

vec3 staggered_grid_helper::trilerp( const staggered_grid& grid, const vec3& pos ) {
    float result[] = { 0, 0, 0 };

    float svpos[] = { pos.x / grid.get_spacing(), pos.y / grid.get_spacing(), pos.z / grid.get_spacing() };
    float sfpos[] = { std::floor( svpos[0] ), std::floor( svpos[1] ), std::floor( svpos[2] ) };
    float salpha[] = { svpos[0] - sfpos[0], svpos[1] - sfpos[1], svpos[2] - sfpos[2] };
    int sipos[] = { (int)sfpos[0] - grid.get_min( 0 ), (int)sfpos[1] - grid.get_min( 1 ),
                    (int)sfpos[2] - grid.get_min( 2 ) };

    float vpos[] = { svpos[0] - 0.5f, svpos[1] - 0.5f, svpos[2] - 0.5f };
    float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };
    float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };
    int ipos[] = { (int)fpos[0] - grid.get_min( 0 ), (int)fpos[1] - grid.get_min( 1 ),
                   (int)fpos[2] - grid.get_min( 2 ) };

    { // x
        int index = sipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * ipos[2] );

        if( ipos[2] >= 0 && ipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - alpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    result[0] += ( 1.f - salpha[0] ) * yzWeight * grid.m_storage[index].x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    result[0] += salpha[0] * yzWeight * grid.m_storage[index + 1].x;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    result[0] += ( 1.f - salpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    result[0] += salpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].x;
            }
        }
        if( ipos[2] + 1 >= 0 && ipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = alpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    result[0] += ( 1.f - salpha[0] ) * yzWeight * grid.m_storage[index].x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    result[0] += salpha[0] * yzWeight * grid.m_storage[index + 1].x;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    result[0] += ( 1.f - salpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    result[0] += salpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].x;
            }
        }
    }

    { // y
        int index = ipos[0] + grid.get_size( 0 ) * ( sipos[1] + grid.get_size( 1 ) * ipos[2] );

        if( ipos[2] >= 0 && ipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - alpha[2];
            if( sipos[1] >= 0 && sipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - salpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[1] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index].y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[1] += alpha[0] * yzWeight * grid.m_storage[index + 1].y;
            }
            if( sipos[1] + 1 >= 0 && sipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * salpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[1] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[1] += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].y;
            }
        }
        if( ipos[2] + 1 >= 0 && ipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = alpha[2];
            if( sipos[1] >= 0 && sipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - salpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[1] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index].y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[1] += alpha[0] * yzWeight * grid.m_storage[index + 1].y;
            }
            if( sipos[1] + 1 >= 0 && sipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * salpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[1] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[1] += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].y;
            }
        }
    }

    { // z
        int index = ipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * sipos[2] );

        if( sipos[2] >= 0 && sipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - salpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[2] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index].z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[2] += alpha[0] * yzWeight * grid.m_storage[index + 1].z;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[2] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[2] += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].z;
            }
        }
        if( sipos[2] + 1 >= 0 && sipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = salpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[2] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index].z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[2] += alpha[0] * yzWeight * grid.m_storage[index + 1].z;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    result[2] += ( 1.f - alpha[0] ) * yzWeight * grid.m_storage[index + grid.get_size( 0 )].z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    result[2] += alpha[0] * yzWeight * grid.m_storage[index + grid.get_size( 0 ) + 1].z;
            }
        }
    }

    return frantic::graphics::vector3f( result[0], result[1], result[2] );
}

template <class MutexType>
void staggered_grid_helper::trilerp_splat( staggered_grid& grid, const vec3& pos, const vec3& splatValue,
                                           MutexType& theMutex ) {
    float svpos[] = { pos.x / grid.get_spacing(), pos.y / grid.get_spacing(), pos.z / grid.get_spacing() };
    float sfpos[] = { std::floor( svpos[0] ), std::floor( svpos[1] ), std::floor( svpos[2] ) };
    float salpha[] = { svpos[0] - sfpos[0], svpos[1] - sfpos[1], svpos[2] - sfpos[2] };
    int sipos[] = { (int)sfpos[0] - grid.get_min( 0 ), (int)sfpos[1] - grid.get_min( 1 ),
                    (int)sfpos[2] - grid.get_min( 2 ) };

    float vpos[] = { svpos[0] - 0.5f, svpos[1] - 0.5f, svpos[2] - 0.5f };
    float fpos[] = { std::floor( vpos[0] ), std::floor( vpos[1] ), std::floor( vpos[2] ) };
    float alpha[] = { vpos[0] - fpos[0], vpos[1] - fpos[1], vpos[2] - fpos[2] };
    int ipos[] = { (int)fpos[0] - grid.get_min( 0 ), (int)fpos[1] - grid.get_min( 1 ),
                   (int)fpos[2] - grid.get_min( 2 ) };

    typename MutexType::scoped_lock theLock( theMutex );

    { // x
        int index = sipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * ipos[2] );

        if( ipos[2] >= 0 && ipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - alpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].x += ( 1.f - salpha[0] ) * yzWeight * splatValue.x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].x += ( salpha[0] ) * yzWeight * splatValue.x;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].x += ( 1.f - salpha[0] ) * yzWeight * splatValue.x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].x += ( salpha[0] ) * yzWeight * splatValue.x;
            }
        }
        if( ipos[2] + 1 >= 0 && ipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = alpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].x += ( 1.f - salpha[0] ) * yzWeight * splatValue.x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].x += ( salpha[0] ) * yzWeight * splatValue.x;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( sipos[0] >= 0 && sipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].x += ( 1.f - salpha[0] ) * yzWeight * splatValue.x;
                if( sipos[0] + 1 >= 0 && sipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].x += ( salpha[0] ) * yzWeight * splatValue.x;
            }
        }
    }

    { // y
        int index = ipos[0] + grid.get_size( 0 ) * ( sipos[1] + grid.get_size( 1 ) * ipos[2] );

        if( ipos[2] >= 0 && ipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - alpha[2];
            if( sipos[1] >= 0 && sipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - salpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].y += ( 1.f - salpha[0] ) * yzWeight * splatValue.y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].y += ( salpha[0] ) * yzWeight * splatValue.y;
            }
            if( sipos[1] + 1 >= 0 && sipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * salpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].y += ( 1.f - alpha[0] ) * yzWeight * splatValue.y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].y += ( alpha[0] ) * yzWeight * splatValue.y;
            }
        }
        if( ipos[2] + 1 >= 0 && ipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = alpha[2];
            if( sipos[1] >= 0 && sipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - salpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].y += ( 1.f - salpha[0] ) * yzWeight * splatValue.y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].y += ( salpha[0] ) * yzWeight * splatValue.y;
            }
            if( sipos[1] + 1 >= 0 && sipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * salpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].y += ( 1.f - alpha[0] ) * yzWeight * splatValue.y;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].y += ( alpha[0] ) * yzWeight * splatValue.y;
            }
        }
    }

    { // z
        int index = ipos[0] + grid.get_size( 0 ) * ( ipos[1] + grid.get_size( 1 ) * sipos[2] );

        if( sipos[2] >= 0 && sipos[2] < grid.get_size( 2 ) ) {
            float zWeight = 1.f - salpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].z += ( 1.f - alpha[0] ) * yzWeight * splatValue.z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].z += ( alpha[0] ) * yzWeight * splatValue.z;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].z += ( 1.f - alpha[0] ) * yzWeight * splatValue.z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].z += ( alpha[0] ) * yzWeight * splatValue.z;
            }
        }
        if( sipos[2] + 1 >= 0 && sipos[2] + 1 < grid.get_size( 2 ) ) {
            index += grid.get_size( 0 ) * grid.get_size( 1 );

            float zWeight = salpha[2];
            if( ipos[1] >= 0 && ipos[1] < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * ( 1.f - alpha[1] );
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index].z += ( 1.f - alpha[0] ) * yzWeight * splatValue.z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + 1].z += ( alpha[0] ) * yzWeight * splatValue.z;
            }
            if( ipos[1] + 1 >= 0 && ipos[1] + 1 < grid.get_size( 1 ) ) {
                float yzWeight = zWeight * alpha[1];
                if( ipos[0] >= 0 && ipos[0] < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 )].z += ( 1.f - alpha[0] ) * yzWeight * splatValue.z;
                if( ipos[0] + 1 >= 0 && ipos[0] + 1 < grid.get_size( 0 ) )
                    grid.m_storage[index + grid.get_size( 0 ) + 1].z += ( alpha[0] ) * yzWeight * splatValue.z;
            }
        }
    }
}

void staggered_grid_helper::assign_range( const tbb::blocked_range<int>& range, staggered_grid& grid,
                                          const frantic::volumetrics::field_interface& field,
                                          const frantic::channels::channel_accessor<vec3>& chAcc,
                                          progress_accumulator& progress ) {
    int index = range.begin() * grid.get_size( 0 ) * grid.get_size( 1 );

    std::size_t counter = progress.m_progressStep;

    float off = 0.5f * grid.get_spacing();

    std::size_t structSize = field.get_channel_map().structure_size();

    char* temp = static_cast<char*>( alloca( structSize /* * 3*/ ) );

    for( int z = range.begin(); z < range.end(); ++z ) {
        float pz = ( (float)( z + grid.get_min( 2 ) ) + 0.5f ) * grid.get_spacing();

        for( int y = 0, yEnd = grid.get_size( 1 ); y < yEnd; ++y ) {
            float py = ( (float)( y + grid.get_min( 1 ) ) + 0.5f ) * grid.get_spacing();

            for( int x = 0, xEnd = grid.get_size( 0 ); x < xEnd; ++x, ++index ) {
                float px = ( (float)( x + grid.get_min( 0 ) ) + 0.5f ) * grid.get_spacing();

                if( field.evaluate_field( temp, vec3( px - off, py, pz ) ) )
                    grid.m_storage[index].x = chAcc.get( temp ).x;
                else
                    grid.m_storage[index].x = 0.f;

                if( field.evaluate_field( temp, vec3( px, py - off, pz ) ) )
                    grid.m_storage[index].y = chAcc.get( temp ).y;
                else
                    grid.m_storage[index].y = 0.f;

                if( field.evaluate_field( temp, vec3( px, py, pz - off ) ) )
                    grid.m_storage[index].z = chAcc.get( temp ).z;
                else
                    grid.m_storage[index].z = 0.f;
            }
        }

        if( --counter == 0 ) {
            counter = progress.m_progressStep;
            progress.add_progress( progress.m_progressStep );
        }
    }

    progress.add_progress( progress.m_progressStep - counter );
}

void assign( staggered_grid& grid, const frantic::volumetrics::field_interface& field,
             const frantic::tstring& channelName, frantic::logging::progress_logger& progress ) {
    const frantic::channels::channel_map& fieldMap = field.get_channel_map();

    const frantic::channels::channel& ch = fieldMap[channelName];

    if( ch.arity() != 3 || ch.data_type() != frantic::channels::data_type_float32 )
        throw std::runtime_error(
            "Unable to assign staggered_grid from field_interface due to conflicting types. Found: " +
            frantic::strings::to_string(
                frantic::channels::channel_data_type_str( fieldMap[0].arity(), fieldMap[0].data_type() ) ) +
            ", expected: " +
            frantic::strings::to_string( frantic::channels::channel_data_type_traits<vec3>::type_str() ) );

    progress_accumulator prgAccum;
    prgAccum.m_accum = 0;
    prgAccum.m_progressTotal = grid.get_size( 2 );
    prgAccum.m_progressStep = std::max<std::size_t>( 1, prgAccum.m_progressTotal / 10 );
    prgAccum.m_pLogger = &progress;

    tbb::parallel_for( tbb::blocked_range<int>( 0, grid.get_size( 2 ), 10 ),
                       boost::bind( &staggered_grid_helper::assign_range, _1, boost::ref( grid ), boost::cref( field ),
                                    fieldMap.get_accessor<vec3>( channelName ), boost::ref( prgAccum ) ),
                       tbb::auto_partitioner() );
}

vec3 trilerp( const staggered_grid& grid, const vec3& pos ) { return staggered_grid_helper::trilerp( grid, pos ); }

namespace staggered_grid_detail {
void splat_impl( staggered_grid& grid, const vec3& p, const vec3& splatValue, tbb_mutex_type& theMutex ) {
    staggered_grid_helper::trilerp_splat( grid, p, splatValue, theMutex );
}
} // namespace staggered_grid_detail

bool staggered_discretized_field::evaluate_field( void* dest, const frantic::graphics::vector3f& pos ) const {
    *reinterpret_cast<vec3*>( dest ) = staggered_grid_helper::trilerp( m_storage, pos );
    return true;
}

void staggered_discretized_field::assign( const field_interface& field, frantic::logging::progress_logger* progress ) {
    if( field.get_channel_map().channel_count() != 1 )
        throw std::runtime_error( "Unable to assign staggered_grid from field_interface due to multiple channels" );

    frantic::logging::null_progress_logger nullLogger;
    if( !progress )
        progress = &nullLogger;

    ember::assign( m_storage, field, field.get_channel_map()[0].name(), *progress );
}

void staggered_discretized_field::assign( const frantic::volumetrics::field_interface& field,
                                          const frantic::tstring& channelName,
                                          frantic::logging::progress_logger* progress ) {
    frantic::logging::null_progress_logger nullLogger;
    if( !progress )
        progress = &nullLogger;

    ember::assign( m_storage, field, channelName, *progress );
}

} // namespace ember
