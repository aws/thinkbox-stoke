// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <boost/scoped_array.hpp>

#include <stoke/duplicating_particle_istream.hpp>
#include <stoke/id_generator.hpp>
#include <stoke/particle_generator.hpp>
#include <stoke/rk2_advector.hpp>

#include <frantic/particles/streams/culling_particle_istream.hpp>
#include <frantic/particles/streams/empty_particle_istream.hpp>

#define CHUNK_SIZE 10000

namespace stoke {

particle_generator::particle_generator()
    : m_randEng( 1234 ) {
    m_seed = 1234;
    m_rate = 1000;
    m_useRate = true;
    m_ignoreIDs = true;

    m_lifespanMin = 1000.f;
    m_lifespanMax = 1000.f;
    m_jitterRadius = 0.f;

    m_idAllocator.reset( new id_generator );
    m_advector.reset( new rk2_advector );
}

particle_generator::~particle_generator() {}

void particle_generator::set_random_seed( boost::uint32_t seed ) {
    m_seed = seed;
    m_randEng.seed( seed );
}

boost::uint32_t particle_generator::get_random_seed() const { return m_seed; }

bool particle_generator::is_generator_rate_enabled() const { return m_useRate; }

void particle_generator::set_generator_rate_enabled( bool enabled ) { m_useRate = enabled; }

boost::int64_t particle_generator::get_generator_rate() { return m_useRate ? m_rate : -1i64; }

void particle_generator::set_generator_rate( boost::int64_t numPerFrame ) {
    m_useRate = true;
    m_rate = numPerFrame;
}

field_interface_ptr particle_generator::get_initial_velocity_field() { return m_velocityField; }

void particle_generator::set_initial_velocity_field( field_interface_ptr pVelocityField ) {
    m_velocityField = pVelocityField;
}

float particle_generator::get_initial_lifespan_min() { return m_lifespanMin; }

float particle_generator::get_initial_lifespan_max() { return m_lifespanMax; }

void particle_generator::set_initial_lifespan_min( float minSeconds ) { m_lifespanMin = minSeconds; }

void particle_generator::set_initial_lifespan_max( float maxSeconds ) { m_lifespanMax = maxSeconds; }

float particle_generator::get_diffusion_constant() const { return 0.f; }

void particle_generator::set_diffusion_constant( float /*kDiffuse*/ ) {}

id_generator_interface_ptr particle_generator::get_id_allocator() const { return m_idAllocator; }

void particle_generator::set_id_allocator( id_generator_interface_ptr pIDAllocator ) { m_idAllocator = pIDAllocator; }

void particle_generator::get_generator_channels( frantic::channels::channel_map& outMap, bool defineObjectID,
                                                 bool defineNodeHandle ) {
    frantic::channels::channel_map defaultMap;
    defaultMap.define_channel<vec3>( _T("Position") );
    defaultMap.define_channel<vec3>( _T("Velocity") );
    if( defineObjectID ) {
        outMap.define_channel<boost::uint16_t>( _T( "ObjectID" ) );
    }
    if( defineNodeHandle ) {
        outMap.define_channel<boost::uint64_t>( _T( "NodeHandle" ) );
    }
    defaultMap.end_channel_definition();

    particle_istream_ptr pStreamImpl = this->generate_next_particles_impl( defaultMap );

    outMap = pStreamImpl->get_native_channel_map();
}

void particle_generator::generate_next_particles( particle_set_interface_ptr pParticleSet, float timeStepSeconds ) {
    if( m_useRate && m_rate <= 0 )
        return;

    if( timeStepSeconds <= 0 )
        throw std::invalid_argument( "Invalid, non-positive timestep" );

    frantic::channels::channel_map seedMap;
    seedMap.define_channel<vec3>( _T("Position") );

    if( !this->get_ignore_ids() )
        seedMap.define_channel<boost::int64_t>( _T("ID") );

    for( std::size_t i = 0, iEnd = pParticleSet->get_num_channels(); i < iEnd; ++i ) {
        frantic::tstring channelName;
        std::pair<frantic::channels::data_type_t, std::size_t> channelType;

        pParticleSet->get_particle_channel( i, &channelName, &channelType );

        seedMap.define_channel( channelName, channelType.second, channelType.first );
    }

    seedMap.end_channel_definition();

    particle_istream_ptr pStreamImpl;

    try {
        pStreamImpl = this->generate_next_particles_impl( seedMap );
    } catch( const std::exception& e ) {
        FF_LOG( error ) << e.what() << std::endl;

        // TODO: We really ought to do something more drastic about this exception...

        pStreamImpl.reset( new frantic::particles::streams::empty_particle_istream( seedMap ) );
    }

    if( m_useRate && pStreamImpl->particle_count() != m_rate )
        pStreamImpl.reset(
            new frantic::particles::streams::duplicated_particle_istream( pStreamImpl, m_rate, m_seed ) );

    frantic::channels::channel_accessor<vec3> posAccessor = seedMap.get_accessor<vec3>( _T("Position") );
    std::vector<std::pair<particle_set_interface::channel_id, frantic::channels::channel_general_accessor>>
        extraChannels;

    for( std::size_t i = 0, iEnd = pParticleSet->get_num_channels(); i < iEnd; ++i ) {
        frantic::tstring channelName;
        std::pair<frantic::channels::data_type_t, std::size_t> channelType;

        particle_set_interface::channel_id channelID =
            pParticleSet->get_particle_channel( i, &channelName, &channelType );

        extraChannels.push_back( std::make_pair( channelID, seedMap.get_general_accessor( channelName ) ) );
    }

    frantic::channels::channel_accessor<boost::int64_t> idAccessor;
    if( seedMap.has_channel( _T("ID") ) && pStreamImpl->get_native_channel_map().has_channel( _T("ID") ) )
        idAccessor = seedMap.get_accessor<boost::int64_t>( _T("ID") );

    // float timeStepSeconds = static_cast<float>( this->get_timestep_seconds() );
    float minLifespan = std::max( 0.f, m_lifespanMin );
    float maxLifespan = std::max( minLifespan, m_lifespanMax );
    bool hasLifespan = maxLifespan > minLifespan;

    // Jittering should be enabled for streams that cannot support generating truely random positions (ex. seeding from
    // a PRT file).
    bool applyJitter = m_jitterRadius > 0.f;

    boost::variate_generator<boost::mt19937&, boost::uniform_real<float>> rndTime(
        m_randEng, boost::uniform_real<float>( 0, timeStepSeconds ) );
    boost::variate_generator<boost::mt19937&, boost::uniform_real<float>> rndLifespan(
        m_randEng, boost::uniform_real<float>( minLifespan, maxLifespan ) );

    particle_set_interface::index_type indexBase = pParticleSet->get_count();

    boost::scoped_array<char> pBuffer( new char[CHUNK_SIZE * seedMap.structure_size()] );

    bool eos;

    do {
        std::size_t numParticles = CHUNK_SIZE;

        eos = !pStreamImpl->get_particles( pBuffer.get(), numParticles );

        id_generator_interface::id_type baseID = m_idAllocator->allocate_ids( numParticles );

        char* pParticle = pBuffer.get();
        for( std::size_t i = 0; i < numParticles; ++i, pParticle += seedMap.structure_size() ) {
            vec3& p = posAccessor.get( pParticle );
            vec3 v;
            float t = rndTime();
            float l = hasLifespan ? rndLifespan() : minLifespan;
            id_generator_interface::id_type id = baseID + i;

            // We are generating a random age, so we advect the particle forward to the end of the timestep.
            /*if( m_velocityField ){
                    v = m_velocityField->evaluate_velocity( p );
                    p = m_advector->advect_particle( p, v, *m_velocityField, t );
            }*/

            // If jittering is enabled, we can add a random offset to the location
            if( applyJitter )
                p += frantic::graphics::vector3f::from_random_in_sphere_rejection( m_randEng, m_jitterRadius );

            // TOOD: Get advectionStrength from the birth Magma.
            pParticleSet->create_particle( p, v, t, l, id, vec3( 0.0f ) );

            // Copy all the channels we want to forward.
            for( std::vector<std::pair<particle_set_interface::channel_id,
                                       frantic::channels::channel_general_accessor>>::const_iterator
                     it = extraChannels.begin(),
                     itEnd = extraChannels.end();
                 it != itEnd; ++it )
                it->second.copy_primitive_from_channel(
                    reinterpret_cast<char*>( pParticleSet->get_particle_data_ptr( indexBase + i, it->first ) ),
                    pParticle );

            // If we are tracking the IDs of source particles, we mark that particle as "seen" so we don't re-seed from
            // it on the next frame.
            if( idAccessor.is_valid() )
                this->mark_particle_id( idAccessor.get( pParticle ) );
        }

        indexBase += static_cast<particle_set_interface::index_type>( numParticles );
    } while( !eos );

    // this->advance_simulation_impl();
}

void particle_generator::mark_particle_id( boost::int64_t ) {}

} // namespace stoke
