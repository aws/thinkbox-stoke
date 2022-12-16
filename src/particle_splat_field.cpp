// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/particle_splat_field.hpp>

#include <ember/grid.hpp>

#include <frantic/logging/progress_logger.hpp>
#include <frantic/particles/particle_array.hpp>
#include <frantic/particles/particle_istream_iterator.hpp>
#include <frantic/particles/streams/particle_container_particle_istream.hpp>

namespace ember {
void do_poisson_solve( staggered_grid& velocityGrid, float stepInSeconds, const char boundaryTypes[] = "DDDDDD",
                       frantic::logging::progress_logger* progress = NULL );
}

namespace stoke {

particle_splat_field::particle_splat_field( float spacing, const int ( *bounds )[6], int boundsPadding,
                                            bool removeDivergence ) {
    m_spacing = std::max( 0.f, spacing );
    m_useAutoBounds = true;

    if( bounds ) {
        m_useAutoBounds = false;
        ember::staggered_grid( *bounds, spacing ).swap( m_gridStorage );
    }

    m_boundsPadding = std::max( 0, boundsPadding );
    m_removeDivergence = removeDivergence;
    m_velocityScale = 1.f;
}

particle_splat_field::~particle_splat_field() {}

float particle_splat_field::get_velocity_scale() { return m_velocityScale; }

void particle_splat_field::set_velocity_scale( float newScale ) { m_velocityScale = newScale; }

// void particle_splat_field::reset_simulation(){
//	this->update( this->reset_simulation_impl() );
// }
//
// void particle_splat_field::advance_simulation(){
//	this->update( this->advance_simulation_impl() );
// }

namespace {
template <class T>
class particle_accessor {
  public:
    particle_accessor( const frantic::channels::channel_accessor<vec3>& posAccessor,
                       const frantic::channels::channel_accessor<T>& dataAccessor,
                       const frantic::channels::channel_accessor<float>& densityAccessor );

    void operator()( const char* pParticle, vec3& outPos, T& outValue, float& outWeight ) const;

  private:
    frantic::channels::channel_accessor<vec3> m_posAccessor;
    frantic::channels::channel_accessor<T> m_dataAccessor;
    frantic::channels::channel_accessor<float> m_densityAccessor;
};

template <class T>
inline particle_accessor<T>::particle_accessor( const frantic::channels::channel_accessor<vec3>& posAccessor,
                                                const frantic::channels::channel_accessor<T>& dataAccessor,
                                                const frantic::channels::channel_accessor<float>& densityAccessor )
    : m_posAccessor( posAccessor )
    , m_dataAccessor( dataAccessor )
    , m_densityAccessor( densityAccessor ) {}

template <class T>
inline void particle_accessor<T>::operator()( const char* pParticle, vec3& outPos, T& outValue,
                                              float& outWeight ) const {
    outPos = m_posAccessor.get( pParticle );
    outValue = m_dataAccessor.get( pParticle );
    outWeight = m_densityAccessor.get( pParticle );
}
} // namespace

void particle_splat_field::set_particles( frantic::particles::particle_istream_ptr pStream ) {
    frantic::channels::channel_map tempMap;
    tempMap.define_channel<vec3>( _T("Position") );
    tempMap.define_channel<float>( _T("Density") );
    tempMap.define_channel<vec3>( _T("Velocity") );
    tempMap.end_channel_definition();

    char* tempBuffer = reinterpret_cast<char*>( alloca( tempMap.structure_size() ) );

    tempMap.construct_structure( tempBuffer );
    tempMap.get_accessor<float>( _T("Density") ).get( tempBuffer ) = 1.f;

    pStream->set_channel_map( tempMap );
    pStream->set_default_particle( tempBuffer );

    if( m_useAutoBounds ) {
        frantic::particles::particle_array tempParticles( tempMap );
        tempParticles.insert_particles( pStream );

        frantic::channels::channel_accessor<vec3> posAccessor =
            tempParticles.get_channel_map().get_accessor<vec3>( _T("Position") );

        int bounds[] = { INT_MAX, INT_MIN, INT_MAX, INT_MIN, INT_MAX, INT_MIN };
        int voxelCoord[3];

        for( frantic::particles::particle_array::const_iterator it = tempParticles.begin(), itEnd = tempParticles.end();
             it != itEnd; ++it ) {
            ember::float_to_integer_coord( posAccessor.get( *it ), m_spacing, voxelCoord );

            if( voxelCoord[0] < bounds[0] )
                bounds[0] = voxelCoord[0];
            if( voxelCoord[0] >= bounds[1] )
                bounds[1] = voxelCoord[0];

            if( voxelCoord[1] < bounds[2] )
                bounds[2] = voxelCoord[1];
            if( voxelCoord[1] >= bounds[3] )
                bounds[3] = voxelCoord[1];

            if( voxelCoord[2] < bounds[4] )
                bounds[4] = voxelCoord[2];
            if( voxelCoord[2] >= bounds[5] )
                bounds[5] = voxelCoord[2];
        }

        const int radius = 1; // If this isn't constant, we could use the max splat radius.

        bounds[0] -=
            radius - 1 + m_boundsPadding; // -1 because we left justify the int coordinate (ie. 5.345 becomes 5)
        bounds[2] -= radius - 1 + m_boundsPadding;
        bounds[4] -= radius - 1 + m_boundsPadding;

        bounds[1] += radius + 1 + m_boundsPadding; // +1 beccause we use half-open intervals.
        bounds[3] += radius + 1 + m_boundsPadding;
        bounds[5] += radius + 1 + m_boundsPadding;

        frantic::channels::channel_accessor<vec3> velocityAccessor =
            tempParticles.get_channel_map().get_accessor<vec3>( _T("Velocity") );
        frantic::channels::channel_accessor<float> densityAccessor =
            tempParticles.get_channel_map().get_accessor<float>( _T("Density") );

        ember::staggered_grid( bounds, m_spacing ).swap( m_gridStorage );
        ember::staggered_grid densityGrid( bounds, m_spacing );

        ember::splat_weighted_particles( m_gridStorage, densityGrid, tempParticles.begin(), tempParticles.end(),
                                         particle_accessor<vec3>( posAccessor, velocityAccessor, densityAccessor ) );
        ember::normalize_grid_from_weights( m_gridStorage, densityGrid );
    } else {
        frantic::channels::channel_accessor<vec3> posAccessor =
            pStream->get_channel_map().get_accessor<vec3>( _T("Position") );
        frantic::channels::channel_accessor<vec3> velocityAccessor =
            pStream->get_channel_map().get_accessor<vec3>( _T("Velocity") );
        frantic::channels::channel_accessor<float> densityAccessor =
            pStream->get_channel_map().get_accessor<float>( _T("Density") );

        particle_accessor<vec3> particleAccessor( posAccessor, velocityAccessor, densityAccessor );

        int bounds[] = { m_gridStorage.get_min( 0 ), m_gridStorage.get_max( 0 ), m_gridStorage.get_min( 1 ),
                         m_gridStorage.get_max( 1 ), m_gridStorage.get_min( 2 ), m_gridStorage.get_max( 2 ) };

        // Allocate a temporary grid for storing the accumulated weights of the particles. This is necessary so that the
        // weighted velocity is computed instead of the sum of all splatted particles. If we did it differently the
        // velocity magnitude would be higher where there were more particles.
        ember::staggered_grid densityGrid( bounds, m_spacing );

        frantic::particles::particle_istream_block_iterator blockIt( pStream );

        // Read all the particles from the stream in chunks until we've splatted them all.
        while( blockIt.valid() ) {
            ember::splat_weighted_particles( m_gridStorage, densityGrid, blockIt.begin_buffer(), blockIt.end_buffer(),
                                             particleAccessor );

            blockIt.advance();
        }

        ember::normalize_grid_from_weights( m_gridStorage, densityGrid );
    }

    if( m_removeDivergence ) {
        ember::do_poisson_solve( m_gridStorage, 1.f / 30.f ); // TODO: Don't hardcode the timestep!
    }
}

vec3 particle_splat_field::evaluate_velocity( const vec3& p ) const {
    return m_velocityScale * ember::trilerp( m_gridStorage, p );
}

} // namespace stoke
