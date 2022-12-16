// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/particle_set_istream.hpp>

#include <frantic/channels/channel_map_adaptor.hpp>

namespace stoke {

particle_set_istream::particle_set_istream( particle_set_interface_ptr pParticleSet )
    : m_pParticleSet( pParticleSet )
    , m_deriveNormalizedAge( false ) {
    m_particleCount = pParticleSet->get_count();
    m_particleIndex = -1;

    m_nativeMap.define_channel<vec3>( _T("Position") );
    m_nativeMap.define_channel<vec3>( _T("Velocity") );
    m_nativeMap.define_channel<float>( _T("Age") );
    m_nativeMap.define_channel<float>( _T("LifeSpan") );
    m_nativeMap.define_channel<boost::int64_t>( _T("ID") );
    m_nativeMap.define_channel<vec3>( _T( "AdvectionOffset" ) );
    m_nativeMap.define_channel<vec3>( _T( "FieldVelocity" ) );

    for( std::size_t i = 0, iEnd = pParticleSet->get_num_channels(); i < iEnd; ++i ) {
        frantic::tstring channelName;
        std::pair<frantic::channels::data_type_t, std::size_t> channelType;

        pParticleSet->get_particle_channel( i, &channelName, &channelType );

        m_nativeMap.define_channel( channelName, channelType.second, channelType.first );
    }

    m_nativeMap.end_channel_definition();

    frantic::channels::channel_map defaultMap = m_nativeMap;

    // We may already have NormalizedAge as an "extra channel" if this particle_set was deserialized from disk.
    if( !m_nativeMap.has_channel( _T("NormalizedAge") ) ) {
        m_deriveNormalizedAge = true;
        m_nativeMap.append_channel<float>( _T("NormalizedAge") ); // A derived channel: Age / LifeSpan
    }

    this->set_channel_map( defaultMap );
}

particle_set_istream::~particle_set_istream() {}

void particle_set_istream::close() {}

frantic::tstring particle_set_istream::name() const { return _T("particle_set_istream"); }

std::size_t particle_set_istream::particle_size() const { return m_outMap.structure_size(); }

boost::int64_t particle_set_istream::particle_count() const { return m_particleCount; }

boost::int64_t particle_set_istream::particle_index() const { return m_particleIndex; }

boost::int64_t particle_set_istream::particle_count_left() const { return m_particleCount - m_particleIndex - 1; }

boost::int64_t particle_set_istream::particle_progress_count() const { return m_particleCount; }

boost::int64_t particle_set_istream::particle_progress_index() const { return m_particleIndex; }

void particle_set_istream::set_channel_map( const frantic::channels::channel_map& particleChannelMap ) {
    boost::scoped_array<char> newDefaultParticle( new char[particleChannelMap.structure_size()] );

    particleChannelMap.construct_structure( newDefaultParticle.get() );

    if( m_defaultParticle ) {
        frantic::channels::channel_map_adaptor tempAdaptor( particleChannelMap, m_outMap );

        tempAdaptor.copy_structure( newDefaultParticle.get(), m_defaultParticle.get() );
    }

    m_defaultParticle.swap( newDefaultParticle );

    m_velAccessor.reset();
    m_ageAccessor.reset();
    m_lifespanAccessor.reset();
    m_normalizedAgeAccessor.reset();
    m_idAccessor.reset();
    m_advectionOffsetAccessor.reset();
    m_fieldVelocityAccessor.reset();
    m_extraChannels.clear();

    m_outMap = particleChannelMap;

    m_posAccessor = particleChannelMap.get_accessor<vec3>( _T("Position") );

    if( m_outMap.has_channel( _T("Velocity") ) )
        m_velAccessor = m_outMap.get_cvt_accessor<vec3>( _T("Velocity") );

    if( m_outMap.has_channel( _T("Age") ) )
        m_ageAccessor = m_outMap.get_cvt_accessor<float>( _T("Age") );

    if( m_outMap.has_channel( _T("LifeSpan") ) )
        m_lifespanAccessor = m_outMap.get_cvt_accessor<float>( _T("LifeSpan") );

    if( m_outMap.has_channel( _T("NormalizedAge") ) && m_deriveNormalizedAge )
        m_normalizedAgeAccessor = m_outMap.get_cvt_accessor<float>( _T("NormalizedAge") );

    if( m_outMap.has_channel( _T("ID") ) )
        m_idAccessor = m_outMap.get_cvt_accessor<boost::int64_t>( _T("ID") );

    if( m_outMap.has_channel( _T( "AdvectionOffset" ) ) )
        m_advectionOffsetAccessor = m_outMap.get_cvt_accessor<vec3>( _T( "AdvectionOffset" ) );

    if( m_outMap.has_channel( _T( "FieldVelocity" ) ) )
        m_fieldVelocityAccessor = m_outMap.get_cvt_accessor<vec3>( _T( "FieldVelocity" ) );

    for( std::size_t i = 0, iEnd = m_pParticleSet->get_num_channels(); i < iEnd; ++i ) {
        frantic::tstring channelName;
        std::pair<frantic::channels::data_type_t, std::size_t> channelType;

        particle_set_interface::channel_id channelID =
            m_pParticleSet->get_particle_channel( i, &channelName, &channelType );

        if( m_outMap.has_channel( channelName ) ) {
            frantic::channels::channel_general_accessor acc = m_outMap.get_general_accessor( channelName );
            frantic::channels::channel_type_convertor_function_t convertFn =
                frantic::channels::get_channel_type_convertor_function( channelType.first, acc.data_type(),
                                                                        channelName );

            // NOTE: Silently ignores the channel if arity doesn't match.
            if( channelType.second == acc.arity() )
                m_extraChannels.push_back( boost::make_tuple( channelID, acc, convertFn ) );
        }
    }
}

const frantic::channels::channel_map& particle_set_istream::get_channel_map() const { return m_outMap; }

const frantic::channels::channel_map& particle_set_istream::get_native_channel_map() const { return m_nativeMap; }

void particle_set_istream::set_default_particle( char* rawParticleBuffer ) {
    m_defaultParticle.reset( new char[m_outMap.structure_size()] );
    m_outMap.copy_structure( m_defaultParticle.get(), rawParticleBuffer );
}

bool particle_set_istream::get_particle( char* rawParticleBuffer ) {
    if( ++m_particleIndex >= m_particleCount )
        return false;

    m_outMap.copy_structure( rawParticleBuffer, m_defaultParticle.get() );

    m_posAccessor.get( rawParticleBuffer ) = m_pParticleSet->get_position( m_particleIndex );

    if( m_velAccessor.is_valid() )
        m_velAccessor.set( rawParticleBuffer, m_pParticleSet->get_velocity( m_particleIndex ) );

    if( m_ageAccessor.is_valid() )
        m_ageAccessor.set( rawParticleBuffer, m_pParticleSet->get_age( m_particleIndex ) );

    if( m_lifespanAccessor.is_valid() )
        m_lifespanAccessor.set( rawParticleBuffer, m_pParticleSet->get_lifespan( m_particleIndex ) );

    if( m_normalizedAgeAccessor.is_valid() )
        m_normalizedAgeAccessor.set( rawParticleBuffer, m_pParticleSet->get_age( m_particleIndex ) /
                                                            m_pParticleSet->get_lifespan( m_particleIndex ) );

    if( m_idAccessor.is_valid() )
        m_idAccessor.set( rawParticleBuffer, m_pParticleSet->get_id( m_particleIndex ) );

    if( m_advectionOffsetAccessor.is_valid() ) {
        m_advectionOffsetAccessor.set( rawParticleBuffer, m_pParticleSet->get_advection_offset( m_particleIndex ) );
    }

    if( m_fieldVelocityAccessor.is_valid() ) {
        m_fieldVelocityAccessor.set( rawParticleBuffer, m_pParticleSet->get_field_velocity( m_particleIndex ) );
    }

    for( std::vector<extra_channel_type>::const_iterator it = m_extraChannels.begin(), itEnd = m_extraChannels.end();
         it != itEnd; ++it )
        it->get<2>()(
            it->get<1>().get_channel_data_pointer( rawParticleBuffer ),
            reinterpret_cast<const char*>( m_pParticleSet->get_particle_data_ptr( m_particleIndex, it->get<0>() ) ),
            it->get<1>().arity() );

    return true;
}

bool particle_set_istream::get_particles( char* rawParticleBuffer, std::size_t& numParticles ) {
    for( std::size_t i = 0, iEnd = numParticles; i < iEnd; ++i, rawParticleBuffer += m_outMap.structure_size() ) {
        if( !this->get_particle( rawParticleBuffer ) ) {
            numParticles = i;
            return false;
        }
    }
    return true;
}

} // namespace stoke
