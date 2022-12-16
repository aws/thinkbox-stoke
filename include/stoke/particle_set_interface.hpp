// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/data_types.hpp>

#include <frantic/channels/channel_map.hpp>
#include <frantic/channels/named_channel_data.hpp>

#include <boost/shared_ptr.hpp>

#include <memory>

namespace frantic {
namespace particles {
namespace streams {
class particle_ostream;
class particle_istream;
} // namespace streams
} // namespace particles
} // namespace frantic

namespace stoke {

class advector_interface;
class field_interface;

class particle_set_interface {
  public:
    typedef boost::int64_t index_type;
    typedef index_type id_type;
    typedef std::size_t channel_id;

    typedef boost::shared_ptr<particle_set_interface> ptr_type;

    static const channel_id INVALID_CHANNEL = static_cast<channel_id>( -1 );

  public:
    virtual ~particle_set_interface() {}

    virtual void create_particle( const vec3& p, const vec3& v, float age, float lifespan, id_type id,
                                  const vec3& advectionOffset ) = 0;

    virtual index_type get_count() const = 0;

    virtual vec3 get_position( index_type i ) const = 0;

    virtual vec3 get_velocity( index_type i ) const = 0;

    virtual void set_position( index_type i, const vec3& p ) = 0;

    virtual void set_velocity( index_type i, const vec3& v ) = 0;

    virtual float get_age( index_type i ) const = 0;

    // Kinda sketchy to set the age ... Maybe we can advance all particles?
    virtual void set_age( index_type i, float a ) = 0;

    virtual float get_lifespan( index_type i ) const = 0;

    virtual void set_lifespan( index_type i, float lifespan ) = 0;

    virtual id_type get_id( index_type i ) const = 0;

    virtual vec3 get_advection_offset( index_type i ) const = 0;

    virtual void set_advection_offset( index_type i, const vec3& advectionOffset ) = 0;

    virtual vec3 get_field_velocity( index_type i ) const = 0;

    class predicate {
      public:
        virtual ~predicate() {}

        virtual bool evaluate( particle_set_interface& pi, index_type i ) const = 0;
    };

    /**
     * Delete all particles matching a predicate.
     * @param predicate A subclass of the predicate interface that indicates which particles to delete.
     */
    virtual void delete_particles_if( const predicate& predicate ) = 0;

    virtual void advect_particles( const advector_interface& advector, const field_interface& velocityField,
                                   float timeStepSeconds ) = 0;

    virtual void update_age( float timeStepSeconds ) = 0;

    virtual void update_advection_offset( const advector_interface& advector, const field_interface& velocityField,
                                          float timeStepSeconds ) = 0;

    virtual void apply_advection_offset() = 0;

    /**
     * Retrieves the number of extra channels added to this particle set.
     */
    virtual std::size_t get_num_channels() const = 0;

    /**
     * Retrieves the channel_id, name and type information for a channel by index.
     * @param channelIndex The index of the channel. Must be in [ 0, this->get_num_channels() ).
     * @param[out] pOutChannelName If non-null, the name of the channel is stored here.
     * @param[out] pOutType If non-null, the type of the channel is stored here.
     * @return The ID of the channel, or INVALID_CHANNEL if channelIndex is out of bounds.
     */
    virtual channel_id get_particle_channel( std::size_t channelIndex, frantic::tstring* pOutChannelName,
                                             std::pair<frantic::channels::data_type_t, std::size_t>* pOutType ) = 0;

    /**
     * Retrieves the channel_id and type information for a named channel.
     * @param channelName The name of the channel to look for.
     * @param[out] pOutType If non-null, the type of the channel is stored here.
     * @return The ID of the channel, or INVALID_CHANNEL if not found.
     */
    virtual channel_id get_particle_channel( const frantic::tstring& channelName,
                                             std::pair<frantic::channels::data_type_t, std::size_t>* pOutType ) = 0;

    /**
     * Retrieves a pointer to the channels's data for a particle.
     * @param i The index of the particle.
     * @param channelID The ID of the channel to get the ptr to.
     * @return A pointer to the particle's data value for the channel.
     */
    virtual void* get_particle_data_ptr( index_type i, channel_id channelID ) = 0;

    /**
     * Creates a new particle_set_interface instance and copies all particle data into it.
     * @return A new instance of a subclass of particle_set_interface.
     */
    virtual ptr_type clone() const = 0;

    /**
     * Retrieves an estimate of the total memory usage by the particle data.
     * @return The estimated total memory usage in bytes.
     */
    virtual std::size_t get_memory_usage() const = 0;

    /**
     * Retrieves the channel_map used to define the extra channels associated with this particle_set.
     * @return the described channel_map.
     */
    virtual frantic::channels::channel_map get_extra_channels() const = 0;
};

typedef particle_set_interface::ptr_type particle_set_interface_ptr;

void write_particle_set( particle_set_interface& pset, frantic::particles::streams::particle_ostream& streamSink );

particle_set_interface_ptr read_particle_set( frantic::particles::streams::particle_istream& sourceStream );

} // namespace stoke
