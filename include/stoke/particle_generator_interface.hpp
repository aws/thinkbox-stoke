// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/data_types.hpp>
#include <stoke/field_interface.hpp>
#include <stoke/id_generator_interface.hpp>
#include <stoke/particle_set_interface.hpp>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

#include <memory>

namespace frantic {
namespace channels {
class channel_map;
}
} // namespace frantic

namespace stoke {

class time_interface;

class particle_generator_interface {
  public:
    virtual ~particle_generator_interface() {}

    virtual void set_random_seed( boost::uint32_t seed ) = 0;

    virtual boost::uint32_t get_random_seed() const = 0;

    virtual bool is_generator_rate_enabled() const = 0;

    virtual void set_generator_rate_enabled( bool enabled ) = 0;

    virtual boost::int64_t get_generator_rate() = 0;

    virtual void set_generator_rate( boost::int64_t numPerFrame ) = 0;

    virtual field_interface_ptr get_initial_velocity_field() = 0;

    virtual void set_initial_velocity_field( field_interface_ptr pVelocityField ) = 0;

    virtual float get_initial_lifespan_min() = 0;

    virtual float get_initial_lifespan_max() = 0;

    virtual void set_initial_lifespan_min( float minSeconds ) = 0;

    virtual void set_initial_lifespan_max( float maxSeconds ) = 0;

    virtual float get_diffusion_constant() const = 0;

    virtual void set_diffusion_constant( float kDiffuse ) = 0;

    virtual id_generator_interface_ptr get_id_allocator() const = 0;

    virtual void set_id_allocator( id_generator_interface_ptr pIDAllocator ) = 0;

    virtual void get_generator_channels( frantic::channels::channel_map& outMap ) = 0;

    // virtual void reset_simulation() = 0;
    virtual void update( const time_interface& updateTime ) = 0;

    /**
     * Generates new particles into the provided particle set. The particles are created at in the range [currentTime,
     * currentTime + timeStepSeconds] and advected appropriately such that they are valid for 'currentTime +
     * timeStepSeconds'. \param pParticleSet New particles are placed in this container. \param velocityField The
     * velocity field that affects the newly generated particles. \param timeStepSeconds The amount of time to birth the
     * particles over.
     */
    virtual void generate_next_particles( particle_set_interface_ptr pParticleSet,
                                          /*const field_interface& velocityField,*/ float timeStepSeconds ) = 0;
};

typedef boost::shared_ptr<particle_generator_interface> particle_generator_interface_ptr;

} // namespace stoke
