// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/advector_interface.hpp>
#include <stoke/field_interface.hpp>
#include <stoke/particle_set.hpp>
#include <stoke/particle_set_istream.hpp>

#include <frantic/particles/particle_istream_iterator.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4512 4100 )
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#pragma warning( pop )

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <boost/smart_ptr/make_shared.hpp>

namespace stoke {

particle_set::particle_set( const frantic::channels::channel_map& extraChannels, const char* pDefaultValues )
    : m_particleChannelData( extraChannels ) {
    if( extraChannels.channel_count() > 0 ) {
        m_defaultChannelData.reset( new char[extraChannels.structure_size()] );

        if( pDefaultValues ) {
            extraChannels.copy_structure( m_defaultChannelData.get(), pDefaultValues );
        } else {
            extraChannels.construct_structure( m_defaultChannelData.get() );
        }
    }
}

particle_set::~particle_set() {}

void particle_set::create_particle( const vec3& p, const vec3& v, float age, float lifespan, id_type id,
                                    const vec3& advectionOffset ) {
    particle_t newParticle = { p, v, age, lifespan, id, advectionOffset, v };

    m_particleData.push_back( newParticle );

    if( m_particleChannelData.get_channel_map().channel_count() > 0 )
        m_particleChannelData.push_back( m_defaultChannelData.get() );
}

namespace {
void null_deleter( void* ) {}
} // namespace

void particle_set::insert_particles( frantic::particles::streams::particle_istream& stream ) {
    const frantic::channels::channel_map& streamMap = stream.get_channel_map();

    frantic::channels::channel_accessor<vec3> posAccessor = streamMap.get_accessor<vec3>( _T("Position") );
    frantic::channels::channel_cvt_accessor<boost::int64_t> idAccessor =
        streamMap.get_cvt_accessor<boost::int64_t>( _T("ID") );

    frantic::channels::channel_cvt_accessor<vec3> velAccessor( vec3( 0, 0, 0 ) );
    frantic::channels::channel_cvt_accessor<float> ageAccessor( 0.f );
    frantic::channels::channel_cvt_accessor<float> lifespanAccessor( std::numeric_limits<float>::infinity() );
    frantic::channels::channel_cvt_accessor<vec3> advectionOffsetAccessor( vec3( 0.0f ) );
    frantic::channels::channel_cvt_accessor<vec3> fieldVelocityAccessor( vec3( 0.0f ) );

    if( streamMap.has_channel( _T("Velocity") ) )
        velAccessor = streamMap.get_cvt_accessor<vec3>( _T("Velocity") );
    if( streamMap.has_channel( _T("Age") ) )
        ageAccessor = streamMap.get_cvt_accessor<float>( _T("Age") );
    if( streamMap.has_channel( _T("LifeSpan") ) )
        lifespanAccessor = streamMap.get_cvt_accessor<float>( _T("LifeSpan") );
    if( streamMap.has_channel( _T( "AdvectionOffset" ) ) )
        advectionOffsetAccessor = streamMap.get_cvt_accessor<vec3>( _T( "AdvectionOffset" ) );
    if( streamMap.has_channel( _T( "FieldVelocity" ) ) )
        fieldVelocityAccessor = streamMap.get_cvt_accessor<vec3>( _T( "FieldVelocity" ) );

    // TODO: If NormalizedAge & LifeSpan are available, we should be able to recover Age.

    frantic::channels::channel_map_adaptor extraChannelsAdaptor;

    if( stream.particle_count() > 0 ) {
        m_particleData.reserve( static_cast<std::size_t>( stream.particle_count() ) );
        m_particleChannelData.reserve( static_cast<std::size_t>( stream.particle_count() ) );
    }

    frantic::particles::particle_istream_ptr pStream( &stream, &null_deleter );

    particle_t tempParticle;
    char* tempExtraChannels = NULL;

    if( m_particleChannelData.get_channel_map().channel_count() > 0 ) {
        extraChannelsAdaptor.set( m_particleChannelData.get_channel_map(), streamMap );

        tempExtraChannels =
            reinterpret_cast<char*>( alloca( m_particleChannelData.get_channel_map().structure_size() ) );
    }

    for( frantic::particles::particle_istream_block_iterator it( pStream ); it.valid(); it.advance() ) {
        for( frantic::particles::particle_istream_block_iterator::buffer_iterator itBuffer = it.begin_buffer(),
                                                                                  itBufferEnd = it.end_buffer();
             itBuffer != itBufferEnd; ++itBuffer ) {
            tempParticle.p = posAccessor.get( *itBuffer );
            tempParticle.v = velAccessor.get( *itBuffer );
            tempParticle.id = idAccessor.get( *itBuffer ); // TODO: It might make sense to have ID reassignment.
            tempParticle.age = ageAccessor.get( *itBuffer );
            tempParticle.lifespan = lifespanAccessor.get( *itBuffer );
            tempParticle.advectionOffset = advectionOffsetAccessor.get( *itBuffer );
            tempParticle.fieldVelocity = fieldVelocityAccessor.get( *itBuffer );

            m_particleData.push_back( tempParticle );

            if( tempExtraChannels ) {
                extraChannelsAdaptor.copy_structure( tempExtraChannels, *itBuffer );

                m_particleChannelData.push_back( tempExtraChannels );
            }
        }
    }
}

particle_set::index_type particle_set::get_count() const { return static_cast<index_type>( m_particleData.size() ); }

vec3 particle_set::get_position( index_type i ) const { return m_particleData[static_cast<std::size_t>( i )].p; }

vec3 particle_set::get_velocity( index_type i ) const { return m_particleData[static_cast<std::size_t>( i )].v; }

void particle_set::set_position( index_type i, const vec3& p ) { m_particleData[static_cast<std::size_t>( i )].p = p; }

void particle_set::set_velocity( index_type i, const vec3& v ) { m_particleData[static_cast<std::size_t>( i )].v = v; }

float particle_set::get_age( index_type i ) const { return m_particleData[static_cast<std::size_t>( i )].age; }

void particle_set::set_age( index_type i, float a ) { m_particleData[static_cast<std::size_t>( i )].age = a; }

float particle_set::get_lifespan( index_type i ) const {
    return m_particleData[static_cast<std::size_t>( i )].lifespan;
}

void particle_set::set_lifespan( index_type i, float lifespan ) {
    m_particleData[static_cast<std::size_t>( i )].lifespan = lifespan;
}

particle_set::id_type particle_set::get_id( index_type i ) const {
    return m_particleData[static_cast<std::size_t>( i )].id;
}

vec3 particle_set::get_advection_offset( index_type i ) const {
    return m_particleData[static_cast<std::size_t>( i )].advectionOffset;
}

void particle_set::set_advection_offset( index_type i, const vec3& advectionOffset ) {
    m_particleData[static_cast<std::size_t>( i )].advectionOffset = advectionOffset;
}

vec3 particle_set::get_field_velocity( index_type i ) const {
    return m_particleData[static_cast<std::size_t>( i )].fieldVelocity;
}

void particle_set::delete_particles_if( const predicate& predicate ) {
    bool hasExtraChannels = ( m_particleChannelData.get_channel_map().channel_count() > 0 );

    for( std::size_t i = 0, iEnd = m_particleData.size(); i < iEnd; ) {
        if( predicate.evaluate( *this, static_cast<index_type>( i ) ) ) {
            if( i != iEnd ) {
                // The current particle is "deleted" so we copy the particle from the end ontop of the current one,
                // effectively replacing it. First we copy the simulation channels.
                memcpy( &m_particleData[i], &m_particleData.back(), sizeof( particle_t ) );

                // Then we copy any extra data that might be associated with the particle.
                if( hasExtraChannels )
                    m_particleChannelData.get_channel_map().copy_structure( m_particleChannelData.at( i ),
                                                                            m_particleChannelData.at( iEnd - 1 ) );
            }
            --iEnd;

            // Remove the last particle from the collection, since we copied it over the "current" one.
            m_particleData.pop_back();

            if( hasExtraChannels )
                m_particleChannelData.pop_back();
        } else {
            ++i;
        }
    }
}

class particle_set_impl {
  public:
    template <class ForwardIterator>
    static void advect_particles( const tbb::blocked_range<ForwardIterator>& range, const advector_interface& advector,
                                  const field_interface& velocityField, float timeStepSeconds ) {
        for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
            it->p = advector.advect_particle( it->p, it->v, velocityField, timeStepSeconds );
            it->v = velocityField.evaluate_velocity( it->p );
            it->age += timeStepSeconds;
        }
    }

    template <class ForwardIterator>
    static void update_age( const tbb::blocked_range<ForwardIterator>& range, float timeStepSeconds ) {
        for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
            it->age += timeStepSeconds;
        }
    }

    template <class ForwardIterator>
    static void update_advection_offset( const tbb::blocked_range<ForwardIterator>& range,
                                         const advector_interface& advector, const field_interface& velocityField,
                                         float timeStepSeconds ) {
        for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
            it->v = it->fieldVelocity;
            float actualTimeStep = std::min( it->age, timeStepSeconds );
            it->advectionOffset = advector.get_offset( it->p, it->v, velocityField, actualTimeStep );
            it->v = velocityField.evaluate_velocity( it->p + it->advectionOffset );
            it->fieldVelocity = it->v;
        }
    }

    template <class ForwardIterator>
    static void apply_advection_offset( const tbb::blocked_range<ForwardIterator>& range ) {
        for( ForwardIterator it = range.begin(), itEnd = range.end(); it != itEnd; ++it ) {
            it->p = it->p + it->advectionOffset;
        }
    }
};

void particle_set::advect_particles( const advector_interface& advector, const field_interface& velocityField,
                                     float timeStepSeconds ) {
    typedef std::vector<particle_t>::iterator iterator;

    tbb::parallel_for( tbb::blocked_range<iterator>( m_particleData.begin(), m_particleData.end() ),
                       boost::bind( &particle_set_impl::advect_particles<iterator>, _1, boost::ref( advector ),
                                    boost::ref( velocityField ), timeStepSeconds ),
                       tbb::auto_partitioner() );
}

void particle_set::update_age( float timeStepSeconds ) {
    typedef std::vector<particle_t>::iterator iterator;

    tbb::parallel_for( tbb::blocked_range<iterator>( m_particleData.begin(), m_particleData.end() ),
                       boost::bind( &particle_set_impl::update_age<iterator>, _1, timeStepSeconds ),
                       tbb::auto_partitioner() );
}

void particle_set::update_advection_offset( const advector_interface& advector, const field_interface& velocityField,
                                            float timeStepSeconds ) {
    typedef std::vector<particle_t>::iterator iterator;

    tbb::parallel_for( tbb::blocked_range<iterator>( m_particleData.begin(), m_particleData.end() ),
                       boost::bind( &particle_set_impl::update_advection_offset<iterator>, _1, boost::ref( advector ),
                                    boost::ref( velocityField ), timeStepSeconds ),
                       tbb::auto_partitioner() );
}

void particle_set::apply_advection_offset() {
    typedef std::vector<particle_t>::iterator iterator;

    tbb::parallel_for( tbb::blocked_range<iterator>( m_particleData.begin(), m_particleData.end() ),
                       boost::bind( &particle_set_impl::apply_advection_offset<iterator>, _1 ),
                       tbb::auto_partitioner() );
}

std::size_t particle_set::get_num_channels() const { return m_particleChannelData.get_channel_map().channel_count(); }

particle_set::channel_id
particle_set::get_particle_channel( const frantic::tstring& channelName,
                                    std::pair<frantic::channels::data_type_t, std::size_t>* pOutType ) {
    for( std::size_t i = 0, iEnd = m_particleChannelData.get_channel_map().channel_count(); i < iEnd; ++i ) {
        const frantic::channels::channel& ch = m_particleChannelData.get_channel_map()[i];

        if( channelName == ch.name() ) {
            if( pOutType != NULL ) {
                pOutType->first = ch.data_type();
                pOutType->second = ch.arity();
            }

            return static_cast<channel_id>( i );
        }
    }

    return INVALID_CHANNEL;
}

particle_set::channel_id
particle_set::get_particle_channel( std::size_t channelIndex, frantic::tstring* pOutChannelName,
                                    std::pair<frantic::channels::data_type_t, std::size_t>* pOutType ) {
    if( channelIndex >= m_particleChannelData.get_channel_map().channel_count() )
        return INVALID_CHANNEL;

    if( pOutChannelName )
        *pOutChannelName = m_particleChannelData.get_channel_map()[channelIndex].name();

    if( pOutType ) {
        pOutType->first = m_particleChannelData.get_channel_map()[channelIndex].data_type();
        pOutType->second = m_particleChannelData.get_channel_map()[channelIndex].arity();
    }

    return static_cast<channel_id>( channelIndex );
}

void* particle_set::get_particle_data_ptr( index_type i, channel_id channelID ) {
    return m_particleChannelData.get_channel_map()[channelID].get_channel_data_pointer(
        m_particleChannelData.at( static_cast<std::size_t>( i ) ) );
}

particle_set_interface_ptr particle_set::clone() const { return boost::make_shared<particle_set>( *this ); }

std::size_t particle_set::get_memory_usage() const {
    return m_particleData.capacity() * sizeof( particle_t ) + m_particleChannelData.size_in_memory();
}

void write_particle_set( particle_set_interface& pset, frantic::particles::streams::particle_ostream& streamSink ) {
    // These interfaces are supposed to be passed around by shared_ptr, so I need to wrap them up.
    boost::shared_ptr<particle_set_interface> pSetWrapper( &pset, &null_deleter );
    boost::shared_ptr<frantic::particles::streams::particle_ostream> pOutWrapper( &streamSink, &null_deleter );

    boost::shared_ptr<particle_set_istream> pSourceStream = boost::make_shared<particle_set_istream>( pSetWrapper );

    frantic::logging::null_progress_logger logger;
    frantic::particles::save_particle_stream( pSourceStream, pOutWrapper, logger );
}

particle_set_interface_ptr read_particle_set( frantic::particles::streams::particle_istream& sourceStream ) {
    const frantic::channels::channel_map& streamMap = sourceStream.get_channel_map();

    frantic::channels::channel_map extraChannels;
    for( std::size_t i = 0, iEnd = streamMap.channel_count(); i < iEnd; ++i ) {
        const frantic::channels::channel& ch = streamMap[i];

        if( ch.name() != _T("Position") && ch.name() != _T("Velocity") && ch.name() != _T("ID") &&
            ch.name() != _T("Age") && ch.name() != _T("LifeSpan") && ch.name() != _T( "AdvectionOffset" ) &&
            ch.name() != _T( "FieldVelocity" ) )
            extraChannels.define_channel( ch.name(), ch.arity(), ch.data_type() );
    }
    extraChannels.end_channel_definition();

    boost::shared_ptr<particle_set> pResult =
        boost::make_shared<particle_set>( extraChannels, static_cast<const char*>( NULL ) );

    pResult->insert_particles( sourceStream );

    return pResult;
}

inline frantic::channels::channel_map particle_set::get_extra_channels() const {
    return m_particleChannelData.get_channel_map();
}

} // namespace stoke
