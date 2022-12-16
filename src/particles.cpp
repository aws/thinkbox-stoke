// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <ember/ember_compiler.hpp>
#include <ember/particles.hpp>

#include <frantic/channels/channel_map_lerp.hpp>
#include <frantic/diagnostics/profiling_section.hpp>

#pragma warning( push, 3 )
#pragma warning( disable : 4100 4512 )
#include <tbb/atomic.h>
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#pragma warning( pop )

#include <boost/bind.hpp>
#include <boost/random.hpp>

#include <algorithm>

namespace ember {

using namespace frantic::channels;
using namespace frantic::graphics;

/**
 * An implementation of the Fowler-Noll-Vo 32bit hash function (FNV-1A).
 * See http://en.wikipedia.org/wiki/Fowler-Noll-Vo_hash_function
 * This version will compute the continuation of a hash, assuming it has been started
 * by some other function. (ie. below)
 *
 * TODO: Try the MurmurHash hashing function, which is apparently faster.
 */
inline boost::uint32_t continue_hash( boost::uint32_t prevHash, boost::uint32_t nextVal ) {
    static const boost::uint32_t FNV_PRIME = 16777619;

    prevHash ^= ( nextVal & 0xFF );
    prevHash *= FNV_PRIME;
    prevHash ^= ( ( nextVal >> 8 ) & 0xFF );
    prevHash *= FNV_PRIME;
    prevHash ^= ( ( nextVal >> 16 ) & 0xFF );
    prevHash *= FNV_PRIME;
    prevHash ^= ( ( nextVal >> 24 ) & 0xFF );
    prevHash *= FNV_PRIME;

    return prevHash;
}

field_sample_particle_istream::field_sample_particle_istream()
    : m_profiler( _T("Particle Sampling Stream") ) {
    m_min[0] = m_min[1] = m_min[2] = 0;
    m_size[0] = m_size[1] = m_size[2] = 0;
    m_coord[0] = m_coord[1] = m_coord[2] = 0;
    m_spacing = 1.f;

    m_minDensity = 0.f;
    m_maxDensity = 1.f;
    m_compensateDensity = true;
    m_densityChannel = _T("Density");
    m_densityAccessor.reset( 0.f );

    m_randValues.resize( 1, 0.5f );

    m_particleIndex = -1;
    m_sampleIndex = m_maxSamples = 0;
}

field_sample_particle_istream::~field_sample_particle_istream() { this->close(); }

void field_sample_particle_istream::set_field( const movable<expression_type>& expr ) {
    m_expr = boost::move( expr.get() );

    if( m_expr.get_output_map().has_channel( m_densityChannel ) )
        m_densityAccessor = m_expr.get_output_map().get_cvt_accessor<float>( m_densityChannel );

    m_outMap = m_expr.get_output_map();

    // NOTE: the expression should NOT define a position, since that doesn't make sense for a field...
    if( !m_outMap.has_channel( _T("Position") ) )
        m_outMap.append_channel<vec3>( _T("Position") );

    m_outAdaptor.set( m_outMap, m_outMap );
    m_posAccessor = m_outMap.get_accessor<vec3>( _T("Position") );

    // Assign the initial default values.
    m_defaultParticle.reset( new char[m_outMap.structure_size()] );
    m_outMap.construct_structure( m_defaultParticle.get() );
}

void field_sample_particle_istream::set_domain( int bounds[], float spacing ) {
    m_min[0] = bounds[0];
    m_size[0] = bounds[1] - bounds[0];
    m_min[1] = bounds[2];
    m_size[1] = bounds[3] - bounds[2];
    m_min[2] = bounds[4];
    m_size[2] = bounds[5] - bounds[4];
    m_spacing = spacing;

    m_maxSamples = m_size[0] * m_size[1] * m_size[2];
}

void field_sample_particle_istream::set_density_field( const frantic::tstring& channelName ) {
    m_densityChannel = channelName;
    m_densityAccessor.reset( 0.f );

    // We may not have set_field() yet, so see if the expression is empty.
    if( !m_expr.empty() ) {
        if( m_expr.get_output_map().has_channel( m_densityChannel ) )
            m_densityAccessor = m_expr.get_output_map().get_cvt_accessor<float>( m_densityChannel );
    }
}

void field_sample_particle_istream::set_density_compensation( bool enabled ) { m_compensateDensity = enabled; }

void field_sample_particle_istream::set_minimum_density( float minDensity ) { m_minDensity = minDensity; }

void field_sample_particle_istream::set_maximum_density( float maxDensity ) { m_maxDensity = maxDensity; }

void field_sample_particle_istream::set_random_seed( unsigned seed, int quant ) {
    m_randValues.resize( static_cast<std::size_t>( std::max( 1, quant ) ) );

    boost::mt19937 gen( seed );
    boost::uniform_01<float> range;
    boost::variate_generator<boost::mt19937, boost::uniform_01<float>> rng( gen, range );

    std::generate( m_randValues.begin(), m_randValues.end(), rng );
}

void field_sample_particle_istream::close() {
    FF_LOG( stats ) << _T("Ember particle sampling inspected: ") << m_sampleIndex
                    << _T(" sample locations and generated: ") << m_particleIndex << _T(" particles\n") << m_profiler
                    << _T("\nThat's ") << ( m_profiler.total_time() / (unsigned long long)m_particleIndex )
                    << "ms per particle" << std::endl;
}

frantic::tstring field_sample_particle_istream::name() const { return _T("field_sample_particle_istream"); }

std::size_t field_sample_particle_istream::particle_size() const { return this->get_channel_map().structure_size(); }

boost::int64_t field_sample_particle_istream::particle_count() const { return -1; }

boost::int64_t field_sample_particle_istream::particle_index() const { return m_particleIndex; }

boost::int64_t field_sample_particle_istream::particle_count_left() const { return -1; }

boost::int64_t field_sample_particle_istream::particle_progress_count() const { return m_maxSamples; }

boost::int64_t field_sample_particle_istream::particle_progress_index() const { return m_sampleIndex; }

boost::int64_t field_sample_particle_istream::particle_count_guess() const { return m_maxSamples; }

void field_sample_particle_istream::set_channel_map( const frantic::channels::channel_map& particleChannelMap ) {
    if( particleChannelMap != m_outMap ) {
        boost::scoped_array<char> newDefault( new char[particleChannelMap.structure_size()] );

        particleChannelMap.construct_structure( newDefault.get() );

        frantic::channels::channel_map_adaptor tempAdaptor( particleChannelMap, m_outMap );

        tempAdaptor.copy_structure( newDefault.get(), m_defaultParticle.get() );

        m_outMap = particleChannelMap;
        m_posAccessor = m_outMap.get_accessor<vec3>( _T("Position") );
        m_outAdaptor.set( m_outMap, m_expr.get_output_map() );
        m_defaultParticle.swap( newDefault );
    }
}

const frantic::channels::channel_map& field_sample_particle_istream::get_channel_map() const { return m_outMap; }

const frantic::channels::channel_map& field_sample_particle_istream::get_native_channel_map() const {
    return m_expr.get_output_map();
}

void field_sample_particle_istream::set_default_particle( char* rawParticleBuffer ) {
    m_defaultParticle.reset( new char[m_outMap.structure_size()] );

    m_outMap.copy_structure( m_defaultParticle.get(), rawParticleBuffer );
}

static const boost::uint32_t FNV_OFFSET_TB1 = 0xc9de4929; // Random numbers generated from random.org
static const boost::uint32_t FNV_OFFSET_TB2 = 0x07b00928; // Random numbers generated from random.org
static const boost::uint32_t FNV_OFFSET_TB3 = 0x55a92477; // Random numbers generated from random.org

bool field_sample_particle_istream::get_particle( char* rawParticleBuffer ) {
    frantic::diagnostics::scoped_profile spGetParticles( m_profiler );

    char* tempBuffer =
        !m_outAdaptor.is_identity() ? (char*)alloca( m_expr.get_output_map().structure_size() ) : rawParticleBuffer;

    float densityFactor = m_compensateDensity ? ( m_spacing * m_spacing * m_spacing ) : 1.f;

loopStart:
    if( m_coord[0] >= m_size[0] ) {
        if( ++m_coord[1] >= m_size[1] ) {
            if( ++m_coord[2] >= m_size[2] )
                return false;
            m_coord[1] = 0;
        }
        m_coord[0] = 0;
    }

    int x = ( m_coord[0] + m_min[0] );
    int y = ( m_coord[1] + m_min[1] );
    int z = ( m_coord[2] + m_min[2] );

    ++m_coord[0];    // Update m_coord for the next loop iteration (or call to get_particle())
    ++m_sampleIndex; // Update sample index since we may inspect several sample positions before finding a valid
                     // particle.

    vec3 pos;

    pos.x = ( (float)x + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB1, x ), y ), z ) %
                                      m_randValues.size()] ) *
            m_spacing;
    pos.y = ( (float)y + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB2, y ), z ), x ) %
                                      m_randValues.size()] ) *
            m_spacing;
    pos.z = ( (float)z + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB3, z ), x ), y ) %
                                      m_randValues.size()] ) *
            m_spacing;

    m_expr.eval( tempBuffer, pos );

    float density = densityFactor * m_densityAccessor.get( tempBuffer );

    // If this density was not in the range for seeding, move to the next voxel and try again.
    if( density <= m_minDensity )
        goto loopStart;

    m_densityAccessor.set( tempBuffer, std::min( density, m_maxDensity ) );

    if( tempBuffer != rawParticleBuffer )
        m_outAdaptor.copy_structure( rawParticleBuffer, tempBuffer, m_defaultParticle.get() );

    m_posAccessor( rawParticleBuffer ) = pos;

    ++m_particleIndex;

    return true;
}

class voxel_range {
  public:
    int start[3], size[3], runSize, grainSize;

    void get_end( int ( &coord )[3] ) {
        std::div_t y = std::div( ( start[0] + runSize ), size[0] );
        std::div_t z = std::div( ( start[1] + y.quot ), size[1] );

        coord[0] = y.rem;
        coord[1] = z.rem;
        coord[2] = start[2] + z.quot;
    }

    voxel_range( const int ( &_start )[3], const int ( &_size )[3], int _runsize, int _grainSize = 1 ) {
        start[0] = _start[0];
        start[1] = _start[1];
        start[2] = _start[2];
        size[0] = _size[0];
        size[1] = _size[1];
        size[2] = _size[2];
        runSize = _runsize;
        grainSize = _grainSize;
    }

    voxel_range( voxel_range& rhs, tbb::split ) {
        runSize = rhs.runSize / 2;
        grainSize = rhs.grainSize;

        size[0] = rhs.size[0];
        size[1] = rhs.size[1];
        size[2] = rhs.size[2];

        rhs.runSize -= runSize;
        rhs.get_end( start );
    }

    bool empty() const { return runSize <= 0; }

    bool is_divisible() const { return runSize > grainSize; }
};

// This design is going to exhibit poor behavior with regards to cache conflicts btw. processors since each
// each processor will be interleaving writes to these locations. Perhaps its ok since we don't read from there?
class lockable_buffer {
    tbb::atomic<char*> dest;
    std::size_t stride;

  public:
    lockable_buffer( char* _dest, std::size_t _stride )
        : stride( _stride ) {
        dest = _dest;
    }

    char* get_next_location() { return dest.fetch_and_add( static_cast<std::ptrdiff_t>( stride ) ); }

    char* peek_next_location() { return dest; }
};

bool field_sample_particle_istream::get_particles( char* rawParticleBuffer, std::size_t& numParticles ) {
    frantic::diagnostics::scoped_profile spGetParticles( m_profiler );

    std::size_t numGenerated = 0,
                numWanted = std::min( numParticles, static_cast<std::size_t>( m_maxSamples - m_sampleIndex ) );
    char* bufferStart = rawParticleBuffer;

    do {
        std::size_t nextWanted = numWanted - numGenerated;

        voxel_range theRange( m_coord, m_size, static_cast<int>( nextWanted ), 200 );
        lockable_buffer theBuffer( bufferStart, m_outMap.structure_size() );

#if 1
        tbb::parallel_for(
            theRange,
            boost::bind( &field_sample_particle_istream::do_get_particles, this, _1, boost::ref( theBuffer ) ),
            tbb::auto_partitioner() );
#else
        this->do_get_particles( theRange, theBuffer );
#endif

        // We touched 'nextWanted' locations on this application, and made [0,'nextWanted'] particles.
        m_sampleIndex += static_cast<boost::int64_t>( nextWanted );

        // Update m_coord to be the next coordinate to sample on the next iteration (or call to get_particles()).
        theRange.get_end( m_coord );

        std::size_t nextGenerated =
            static_cast<std::size_t>( theBuffer.peek_next_location() - bufferStart ) / m_outMap.structure_size();

        numGenerated += nextGenerated;
        bufferStart += nextGenerated * m_outMap.structure_size();

        // TODO: We could have a max number of tries ...
        // Loop while we have sample locations left, and we haven't sufficiently filled the input buffer.
    } while( m_sampleIndex < m_maxSamples && numGenerated * 2 < numWanted );

    numParticles = numGenerated;

    m_particleIndex += numGenerated;

    return m_sampleIndex < m_maxSamples;
}

void field_sample_particle_istream::do_get_particles( const voxel_range& range, lockable_buffer& buffer ) const {
    int coord[] = { range.start[0], range.start[1], range.start[2] };

    char* tempBuffer = (char*)alloca( m_expr.get_output_map().structure_size() );

    float densityFactor = m_compensateDensity ? ( m_spacing * m_spacing * m_spacing ) : 1.f;

    vec3 pos;

    for( int i = 0; i < range.runSize; ++i ) {
        if( coord[0] >= range.size[0] ) {
            if( ++coord[1] >= range.size[1] ) {
                ++coord[2]; // Don't need to check against range.size[2] since that will be handled in range.runSize.
                coord[1] = 0;
            }
            coord[0] = 0;
        }

        int x = ( coord[0] + m_min[0] );
        int y = ( coord[1] + m_min[1] );
        int z = ( coord[2] + m_min[2] );

        // We've used the current coord values, so move it to the next voxel for the following iteration.
        ++coord[0];

        pos.x = ( (float)x + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB1, x ), y ), z ) %
                                          m_randValues.size()] ) *
                m_spacing;
        pos.y = ( (float)y + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB2, y ), z ), x ) %
                                          m_randValues.size()] ) *
                m_spacing;
        pos.z = ( (float)z + m_randValues[continue_hash( continue_hash( continue_hash( FNV_OFFSET_TB3, z ), x ), y ) %
                                          m_randValues.size()] ) *
                m_spacing;

        m_expr.eval( tempBuffer, pos );

        float density = densityFactor * m_densityAccessor.get( tempBuffer );

        // If this density was not in the range for seeding, move to the next voxel and try again.
        if( density > m_minDensity ) {
            char* dest = buffer.get_next_location();

            m_densityAccessor.set( tempBuffer, std::min( density, m_maxDensity ) );

            m_outAdaptor.copy_structure( dest, tempBuffer, m_defaultParticle.get() );

            const_cast<vec3&>( m_posAccessor( dest ) ) = pos;
        }
    }
}

} // namespace ember
