// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/field_interface.hpp>
#include <stoke/id_generator_interface.hpp>
#include <stoke/particle_generator_interface.hpp>

namespace stoke {

class dummy_particle_generator : public particle_generator_interface {
  public:
    dummy_particle_generator();

    virtual ~dummy_particle_generator();

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

    virtual void get_generator_channels( frantic::channels::channel_map& outMap );

    virtual void update( const time_interface& updateTime );

    virtual void generate_next_particles( particle_set_interface_ptr pParticleSet, float timeStepSeconds );
};

inline dummy_particle_generator::dummy_particle_generator() {}

inline dummy_particle_generator::~dummy_particle_generator() {}

inline void dummy_particle_generator::set_random_seed( boost::uint32_t /*seed*/ ) {}

inline boost::uint32_t dummy_particle_generator::get_random_seed() const { return 0; }

inline bool dummy_particle_generator::is_generator_rate_enabled() const { return false; }

inline void dummy_particle_generator::set_generator_rate_enabled( bool /*enabled*/ ) {}

inline boost::int64_t dummy_particle_generator::get_generator_rate() { return 0; }

inline void dummy_particle_generator::set_generator_rate( boost::int64_t /*numPerFrame*/ ) {}

inline field_interface_ptr dummy_particle_generator::get_initial_velocity_field() { return field_interface_ptr(); }

inline void dummy_particle_generator::set_initial_velocity_field( field_interface_ptr /*pVelocityField*/ ) {}

inline float dummy_particle_generator::get_initial_lifespan_min() { return 0.f; }

inline float dummy_particle_generator::get_initial_lifespan_max() { return 0.f; }

inline void dummy_particle_generator::set_initial_lifespan_min( float /*minSeconds*/ ) {}

inline void dummy_particle_generator::set_initial_lifespan_max( float /*maxSeconds*/ ) {}

inline float dummy_particle_generator::get_diffusion_constant() const { return 0.f; }

inline void dummy_particle_generator::set_diffusion_constant( float /*kDiffuse*/ ) {}

inline id_generator_interface_ptr dummy_particle_generator::get_id_allocator() const {
    return id_generator_interface_ptr();
}

inline void dummy_particle_generator::set_id_allocator( id_generator_interface_ptr /*pIDAllocator*/ ) {}

inline void dummy_particle_generator::get_generator_channels( frantic::channels::channel_map& /*outMap*/ ) {}

inline void dummy_particle_generator::update( const time_interface& ) {}

inline void dummy_particle_generator::generate_next_particles( particle_set_interface_ptr /*pParticleSet*/,
                                                               float /*timeStepSeconds*/ ) {}

} // namespace stoke
