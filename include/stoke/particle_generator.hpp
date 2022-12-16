// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include <stoke/advector_interface.hpp>
#include <stoke/particle_generator_interface.hpp>
#include <stoke/particle_set_interface.hpp>

#include <frantic/particles/streams/particle_istream.hpp>

#include <boost/random.hpp>

namespace stoke {

class particle_generator : public particle_generator_interface {
  public:
    particle_generator();

    virtual ~particle_generator();

    virtual void set_random_seed( boost::uint32_t seed );

    virtual boost::uint32_t get_random_seed() const;

    virtual bool is_generator_rate_enabled() const;

    virtual void set_generator_rate_enabled( bool enabled );

    virtual boost::int64_t get_generator_rate();

    virtual void set_generator_rate( boost::int64_t numPerFrame );

    virtual field_interface_ptr get_initial_velocity_field();

    virtual void set_initial_velocity_field( field_interface_ptr pVelocityField );

    virtual float get_initial_lifespan_min();

    virtual float get_initial_lifespan_max();

    virtual void set_initial_lifespan_min( float minSeconds );

    virtual void set_initial_lifespan_max( float maxSeconds );

    virtual float get_diffusion_constant() const;

    virtual void set_diffusion_constant( float kDiffuse );

    virtual id_generator_interface_ptr get_id_allocator() const;

    virtual void set_id_allocator( id_generator_interface_ptr pIDAllocator );

    virtual void get_generator_channels( frantic::channels::channel_map& outMap, bool defineObjectID = false,
                                         bool defineNodeHandle = false );

    virtual void update( const time_interface& updateTime ) = 0;

    virtual void generate_next_particles( particle_set_interface_ptr pParticleSet, float timeStepSeconds );

  protected:
    typedef frantic::particles::particle_istream_ptr particle_istream_ptr;

    /**
     * This is used in application of the template method design pattern. generate_next_particles() will call this
     * method to generate the particle stream. Further processing will then occur in generate_next_particles().
     * @param requestedChannels The channels requested from the source.
     * @return A particle_istream generating the particles for the current timestep.
     */
    virtual particle_istream_ptr
    generate_next_particles_impl( const frantic::channels::channel_map& requestedChannels ) = 0;

    /**
     * If generate_next_particles_impl() returns a stream with IDs, this function will be called for each ID that is
     * used. Not all particles from the stream are used in some circumstances so this virtual function is used to report
     * the ones that were actually used. This is used with generators that create new particles from existing ones to
     * limit the potential seeding particles to only those that were not previously used.
     *
     * @note The default implementation does nothing.
     *
     * @param id The ID of the particle that was used to generate a particle.
     */
    virtual void mark_particle_id( boost::int64_t id );

    void set_ignore_ids( bool ignoreIDs );

    bool get_ignore_ids() const;

    void set_jitter_radius( float jitterRadius );

  private:
    boost::uint32_t m_seed;
    boost::mt19937 m_randEng;

    boost::int64_t m_rate;
    bool m_useRate;
    bool m_ignoreIDs;

    float m_lifespanMin, m_lifespanMax;
    float m_jitterRadius;

    id_generator_interface_ptr m_idAllocator; // Object that creates IDs for particles as they are created.

    field_interface_ptr m_velocityField; // Used to advect new particle for their first partial frame.
    advector_interface_ptr m_advector;   // Used to advect new particle for their first partial frame.
};

inline void particle_generator::set_ignore_ids( bool ignoreIDs ) { m_ignoreIDs = ignoreIDs; }

inline bool particle_generator::get_ignore_ids() const { return m_ignoreIDs; }

inline void particle_generator::set_jitter_radius( float jitterRadius ) { m_jitterRadius = jitterRadius; }

} // namespace stoke
