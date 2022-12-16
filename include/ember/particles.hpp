// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ember/ember_compiler.hpp>

#include <frantic/magma/magma_movable.hpp>
#include <frantic/magma/nodes/magma_input_particles_interface.hpp>

#include <frantic/particles/particle_array.hpp>
#include <frantic/particles/particle_kdtree.hpp>
#include <frantic/particles/streams/particle_istream.hpp>

#include <boost/integer.hpp>
#include <boost/thread/tss.hpp>

#include <tbb/blocked_range.h>

namespace frantic {
namespace diagnostics {
class profiling_section;
}
} // namespace frantic

namespace ember {

using frantic::magma::movable;
using frantic::magma::quat;
using frantic::magma::vec3;

class lockable_buffer;
class voxel_range;

class field_sample_particle_istream : public frantic::particles::streams::particle_istream {
  public:
    typedef ember::ember_compiler expression_type;

  public:
    field_sample_particle_istream();

    virtual ~field_sample_particle_istream();

    void set_field( const movable<expression_type>& expr );

    void set_domain( int bounds[], float spacing );

    void set_density_field( const frantic::tstring& channelName );

    void set_density_compensation( bool enabled );

    void set_minimum_density( float minDensity );

    void set_maximum_density( float maxDensity );

    // Set the random seed. We precompute and select random values procedurally so a given voxel always is sampled at
    // the same point, regardless of change in the size of the domain.
    void set_random_seed( unsigned seed, int quant = 3 * 2048 );

  public:
    virtual void close();

    // The stream can return its filename or other identifier for better error messages.
    virtual frantic::tstring name() const;

    // TODO: We should add a verbose_name function, which all wrapping streams are required to mark up in some way

    // This is the size of the particle structure which will be loaded, in bytes.
    virtual std::size_t particle_size() const;

    // Returns the number of particles, or -1 if unknown
    virtual boost::int64_t particle_count() const;
    virtual boost::int64_t particle_index() const;
    virtual boost::int64_t particle_count_left() const;

    virtual boost::int64_t particle_progress_count() const;
    virtual boost::int64_t particle_progress_index() const;

    // If a stream does not know how many particles it has, it can optionally override this function
    // to produce a guess of how many there will be. This guess will be used to pre-allocate storage
    // for this many particles if the user is concerned about memory performance.
    virtual boost::int64_t particle_count_guess() const;

    // This allows you to change the particle layout that's being loaded on the fly, in case it couldn't
    // be set correctly at creation time.
    virtual void set_channel_map( const frantic::channels::channel_map& particleChannelMap );

    // This is the particle channel map which specifies the byte layout of the particle structure that is being used.
    virtual const frantic::channels::channel_map& get_channel_map() const;

    // This is the particle channel map which specifies the byte layout of the input to this stream.
    // NOTE: This value is allowed to change after the following conditions:
    //    * set_channel_map() is called (for example, the empty_particle_istream equates the native map with the
    //    external map)
    virtual const frantic::channels::channel_map& get_native_channel_map() const;

    /** This provides a default particle which should be used to fill in channels of the requested channel map
     *	which are not supplied by the native channel map.
     *	IMPORTANT: Make sure the buffer you pass in is at least as big as particle_size() bytes.
     */
    virtual void set_default_particle( char* rawParticleBuffer );

    // This reads a particle into a buffer matching the channel_map.
    // It returns true if a particle was read, false otherwise.
    // IMPORTANT: Make sure the buffer you pass in is at least as big as particle_size() bytes.
    virtual bool get_particle( char* rawParticleBuffer );

    // This reads a group of particles. Returns false if the end of the source
    // was reached during the read.
    virtual bool get_particles( char* rawParticleBuffer, std::size_t& numParticles );

  private:
    expression_type m_expr;
    int m_min[3], m_size[3];
    float m_spacing;
    float m_minDensity, m_maxDensity;
    bool m_compensateDensity;

    frantic::tstring m_densityChannel;
    std::vector<float> m_randValues;

    int m_coord[3];

    frantic::diagnostics::profiling_section m_profiler;

  private:
    boost::int64_t m_sampleIndex,
        m_maxSamples; // Records the number of samples made, and the maximum number (ie. m_size[0]*m_size[1]*m_size[2]).
    boost::int64_t m_particleIndex; // Records the number of particles generated so far.

    frantic::channels::channel_map m_outMap;
    frantic::channels::channel_map_adaptor m_outAdaptor;
    frantic::channels::channel_accessor<vec3>
        m_posAccessor; // Access the position value *after* adapting to the output stream.
    frantic::channels::channel_cvt_accessor<float>
        m_densityAccessor; // Access the density value *before* adapting to the output stream.

    boost::scoped_array<char> m_defaultParticle;

  private:
    void do_get_particles( const voxel_range& range, lockable_buffer& buffer ) const;
};

} // namespace ember
