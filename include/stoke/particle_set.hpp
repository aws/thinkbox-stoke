// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/particle_set_interface.hpp>

#include <frantic/particles/particle_array.hpp>

#include <boost/shared_array.hpp>

#include <vector>

namespace frantic {
namespace particles {
namespace streams {
class particle_istream;
}
} // namespace particles
} // namespace frantic

namespace stoke {

class particle_set : public particle_set_interface {
  public:
    particle_set( const frantic::channels::channel_map& extraChannels, const char* pDefaultValues = NULL );

    virtual ~particle_set();

    /**
     * Creates and appends a new particle to the set.
     * @param p The position of the new particle.
     * @param v The initial velocity of the new particle.
     * @param age The initial age of the particle. This should be a small, negative random number in order to simulate
     * sub-frame birthing and to avoid banding by birthing all particles at once.
     * @param lifespan The number of seconds a particle will be "alive" before it is considered dead and potentially
     * deleted.
     * @param id The unique ID associated with the new particle.
     */
    virtual void create_particle( const vec3& p, const vec3& v, float age, float lifespan, id_type id,
                                  const vec3& advectionOffset );

    void insert_particles( frantic::particles::streams::particle_istream& stream );

    // From particle_set_interface
    virtual index_type get_count() const;

    virtual vec3 get_position( index_type i ) const;

    virtual vec3 get_velocity( index_type i ) const;

    virtual void set_position( index_type i, const vec3& p );

    virtual void set_velocity( index_type i, const vec3& v );

    virtual float get_age( index_type i ) const;

    virtual void set_age( index_type i, float a );

    virtual float get_lifespan( index_type i ) const;

    virtual void set_lifespan( index_type i, float lifespan );

    virtual id_type get_id( index_type i ) const;

    virtual vec3 get_advection_offset( index_type i ) const;

    virtual void set_advection_offset( index_type i, const vec3& advectionOffset );

    virtual vec3 get_field_velocity( index_type i ) const;

    virtual void delete_particles_if( const predicate& predicate );

    virtual void advect_particles( const advector_interface& advectionField, const field_interface& velocityField,
                                   float timeStepSeconds );

    virtual void update_age( float timeStepSeconds );

    virtual void update_advection_offset( const advector_interface& advector, const field_interface& velocityField,
                                          float timeStepSeconds );

    virtual void apply_advection_offset();

    virtual std::size_t get_num_channels() const;

    virtual channel_id get_particle_channel( const frantic::tstring& channelName,
                                             std::pair<frantic::channels::data_type_t, std::size_t>* pOutType );

    virtual channel_id get_particle_channel( std::size_t channelIndex, frantic::tstring* pOutChannelName,
                                             std::pair<frantic::channels::data_type_t, std::size_t>* pOutType );

    virtual void* get_particle_data_ptr( index_type i, channel_id channelID );

    virtual particle_set_interface_ptr clone() const;

    virtual std::size_t get_memory_usage() const;

    virtual frantic::channels::channel_map get_extra_channels() const;

  private:
    friend class particle_set_impl;

  private:
    struct particle_t {
        vec3 p, v;
        float age, lifespan;
        id_type id; // Should the id be in the particle, or somewhere else? It isn't used in normal calculations...
        vec3 advectionOffset;
        vec3 fieldVelocity;
    };

    std::vector<particle_t> m_particleData;

    boost::shared_array<char> m_defaultChannelData;

    frantic::particles::particle_array
        m_particleChannelData; // Non-standard channel data is kept here in a parallel array to 'm_particleData'.
};

} // namespace stoke
