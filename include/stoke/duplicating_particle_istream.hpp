// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <frantic/particles/particle_array.hpp>
#include <frantic/particles/streams/particle_istream.hpp>
#include <frantic/particles/streams/shared_particle_container_particle_istream.hpp>

#include <boost/random.hpp>
#include <vector>

namespace frantic {
namespace particles {
namespace streams {

/**
 * This decorator stream will produce N particles (specified at construction time) from the delegate stream of M
 * particles by producing each particle floor(N/M) times. The remainder N%M particles will be distributed randomly
 * causing some particles to create floor(N/M)+1 particles.
 */
class duplicated_particle_istream : public delegated_particle_istream {
  public:
    duplicated_particle_istream( particle_istream_ptr pDelegate, boost::int64_t requestedCount,
                                 boost::uint32_t seed = 1234 );

    virtual ~duplicated_particle_istream();

    virtual boost::int64_t particle_count() const;
    virtual boost::int64_t particle_index() const;
    virtual boost::int64_t particle_count_left() const;
    virtual boost::int64_t particle_progress_count() const;
    virtual boost::int64_t particle_progress_index() const;
    virtual boost::int64_t particle_count_guess() const;

    virtual bool get_particle( char* rawParticleBuffer );
    virtual bool get_particles( char* buffer, std::size_t& numParticles );

  private:
    bool is_initialized();
    void initialize();

    void refill_data_cache();

    static const std::size_t CACHE_SIZE = 10000;

  private:
    // Store the set of particles that need to make an extra duplicate. This vector stores the particle index, sorted
    // from highest to lowest index so that we can pop of the end of the list when done. NOTE: Could use a different
    // data structure that compresses this information. I'm using a sorted vector for set membership, but there are
    // plenty of other options.
    std::vector<boost::int64_t> m_duplicateList;

    boost::int64_t m_particleIndex, m_particleCount, m_sourceParticleIndex;
    boost::int64_t m_duplicateCount, m_currentDuplicatesLeft;
    boost::uint32_t m_seed;

    frantic::particles::particle_array m_dataCache;
    frantic::particles::particle_array::const_iterator m_currentSource;
};

inline duplicated_particle_istream::duplicated_particle_istream( particle_istream_ptr pDelegate,
                                                                 boost::int64_t requestedCount, boost::uint32_t seed )
    : delegated_particle_istream( pDelegate )
    , m_dataCache( pDelegate->get_channel_map() ) {
    m_particleIndex = m_sourceParticleIndex = -1;
    m_particleCount = std::max( 0i64, requestedCount );
    m_duplicateCount = m_currentDuplicatesLeft = 0;
    m_seed = seed;
}

inline duplicated_particle_istream::~duplicated_particle_istream() {}

inline bool duplicated_particle_istream::is_initialized() { return m_particleIndex >= 0; }

namespace detail {
template <class T>
class increment_generator {
    T m_value;

  public:
    increment_generator( T baseValue )
        : m_value( baseValue ) {}

    T operator()() { return m_value++; }
};

template <class Generator>
class rand_shuffler {
    Generator& m_randGen;

    rand_shuffler& operator=( const rand_shuffler& rhs ); // disabled.

  public:
    rand_shuffler( Generator& randGen )
        : m_randGen( randGen ) {}

    std::ptrdiff_t operator()( std::ptrdiff_t maxTarg ) {
        return boost::variate_generator<Generator&, boost::uniform_int<std::ptrdiff_t>>(
            m_randGen, boost::uniform_int<std::ptrdiff_t>( 0, maxTarg - 1 ) )();
    }
};
} // namespace detail

inline void duplicated_particle_istream::initialize() {
    m_dataCache.reset( m_delegate->get_channel_map() );

    boost::int64_t sourceCount = m_delegate->particle_count();
    boost::int64_t numExtra;

    if( sourceCount < 0 ) {
        m_dataCache.insert_particles( m_delegate );

        sourceCount = static_cast<boost::int64_t>( m_dataCache.size() );
    } else {
        m_dataCache.reserve( CACHE_SIZE );

        this->refill_data_cache();
    }

    if( sourceCount == 0 ) {
        // We can't generate any particles, so we just set our state in such a way that get_particle() will do nothing.
        m_particleIndex = m_particleCount = 0;
    } else {
        m_duplicateCount = m_particleCount / sourceCount;

        numExtra = m_particleCount % sourceCount;

        if( numExtra > 0 ) {
            // TODO: If this number is small, it would be quicker to generate random numbers and discard duplicates.
            boost::mt19937 generator( m_seed );

            m_duplicateList.reserve( static_cast<std::size_t>( sourceCount ) );

            std::generate_n( std::back_inserter( m_duplicateList ), sourceCount,
                             detail::increment_generator<std::size_t>( 0 ) );

            detail::rand_shuffler<boost::mt19937> shuffleImpl( generator );

            std::random_shuffle( m_duplicateList.begin(), m_duplicateList.end(), shuffleImpl );

            m_duplicateList.resize( static_cast<std::size_t>(
                numExtra ) ); // Num extra will always be <= sourceCount so this only shrinks the vector.

            // Sort in descending order, so we can look at m_duplicateList.back() and see the next particle to duplicate
            // then pop it off.
            std::sort( m_duplicateList.begin(), m_duplicateList.end(), std::greater<boost::int64_t>() );
        }

        m_currentSource = m_dataCache.begin();
        m_sourceParticleIndex = 0;
        m_currentDuplicatesLeft = m_duplicateCount;

        if( !m_duplicateList.empty() && m_duplicateList.back() == m_sourceParticleIndex ) {
            ++m_currentDuplicatesLeft;
            m_duplicateList.pop_back();
        }
    }
}

inline void duplicated_particle_istream::refill_data_cache() {
    // We initialized the cache's capacity to hold CACHE_SIZE particles (or all the source particles, but then this is a
    // no-op).
    m_dataCache.resize( m_dataCache.capacity() );

    std::size_t numParticles;
    bool eos;

    do {
        numParticles = m_dataCache.size();
        eos = !m_delegate->get_particles( m_dataCache.at( 0 ), numParticles );
    } while( !eos && numParticles == 0 );

    m_dataCache.resize( numParticles );
}

inline boost::int64_t duplicated_particle_istream::particle_count() const { return m_particleCount; }

inline boost::int64_t duplicated_particle_istream::particle_index() const { return m_particleIndex; }

inline boost::int64_t duplicated_particle_istream::particle_count_left() const {
    return m_particleCount - m_particleIndex - 1;
}

inline boost::int64_t duplicated_particle_istream::particle_progress_count() const { return m_particleCount; }

inline boost::int64_t duplicated_particle_istream::particle_progress_index() const { return m_particleIndex; }

inline boost::int64_t duplicated_particle_istream::particle_count_guess() const { return m_particleCount; }

inline bool duplicated_particle_istream::get_particle( char* rawParticleBuffer ) {
    if( !this->is_initialized() )
        this->initialize();

    if( ++m_particleIndex >= m_particleCount )
        return false;

    // Need a while loop here to account for when the requested count is less than the source count which means most
    // source particles have m_currentDuplicatesLeft == 0.
    while( m_currentDuplicatesLeft == 0 ) {
        // Advance to the next cached particle.
        // NOTE: m_currentSource must always be valid, and != m_dataCache.end()
        ++m_currentSource;
        ++m_sourceParticleIndex;

        if( m_currentSource == m_dataCache.end() ) {
            this->refill_data_cache();

            // NOTE: This should never occur, because we set everything up to produce exactly 'm_particleCount'
            // particles but this means we failed!
            if( m_dataCache.size() == 0 ) {
                m_particleCount = m_particleIndex;
                return false;
            }

            m_currentSource = m_dataCache.begin();
        }

        m_currentDuplicatesLeft = m_duplicateCount;

        if( !m_duplicateList.empty() && m_duplicateList.back() == m_sourceParticleIndex ) {
            ++m_currentDuplicatesLeft;
            m_duplicateList.pop_back();
        }
    }

    m_dataCache.get_channel_map().copy_structure( rawParticleBuffer, *m_currentSource );

    --m_currentDuplicatesLeft;

    return true;
}

// Its a real pain to implement this multi-threaded AND maintain the same order of particles generated. I'm not
// convinced the complicated implementation is worth it, so I'm using the naive implementation and waiting until
// measurements prove otherwise.
inline bool duplicated_particle_istream::get_particles( char* buffer, std::size_t& numParticles ) {
    std::size_t bufferStride = m_delegate->get_channel_map().structure_size();

    for( std::size_t i = 0, iEnd = numParticles; i < iEnd; ++i, buffer += bufferStride ) {
        if( !this->get_particle( buffer ) ) {
            numParticles = i;
            return false;
        }
    }

    return true;
}

} // namespace streams
} // namespace particles
} // namespace frantic
