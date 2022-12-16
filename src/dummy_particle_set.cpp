// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "stdafx.h"

#include <stoke/dummy_particle_set.hpp>

using namespace stoke;

dummy_particle_set::dummy_particle_set() {}

inline dummy_particle_set::~dummy_particle_set() {}

inline void dummy_particle_set::create_particle( const vec3& /*p*/, const vec3& /*v*/, float /*age*/,
                                                 float /*lifespan*/, id_type /*id */,
                                                 const vec3& /*advectionOffset*/ ) {}

inline dummy_particle_set::index_type dummy_particle_set::get_count() const { return 0; }

inline vec3 dummy_particle_set::get_position( index_type /*i*/ ) const { return vec3(); }

inline vec3 dummy_particle_set::get_velocity( index_type /*i*/ ) const { return vec3(); }

inline void dummy_particle_set::set_position( index_type /*i*/, const vec3& /*p*/ ) {}

inline void dummy_particle_set::set_velocity( index_type /*i*/, const vec3& /*v*/ ) {}

inline float dummy_particle_set::get_age( index_type /*i*/ ) const { return 0.f; }

inline void dummy_particle_set::set_age( index_type /*i*/, float /*a*/ ) {}

inline float dummy_particle_set::get_lifespan( index_type /*i*/ ) const { return 0.f; }

inline void dummy_particle_set::set_lifespan( index_type /*i*/, float /*lifespan*/ ) {}

inline dummy_particle_set::id_type dummy_particle_set::get_id( index_type /*i*/ ) const { return 0; }

vec3 dummy_particle_set::get_advection_offset( index_type /*i*/ ) const { return vec3( 0.0f ); }

void dummy_particle_set::set_advection_offset( index_type /*i*/, const vec3& /*advectionOffset*/ ) {}

vec3 dummy_particle_set::get_field_velocity( index_type /*i*/ ) const { return vec3( 0.0f ); }

inline void dummy_particle_set::delete_particles_if( const predicate& /*predicate*/ ) {}

inline void dummy_particle_set::advect_particles( const advector_interface& /*advectionField*/,
                                                  const field_interface& /*velocityField*/,
                                                  float /*timeStepSeconds*/ ) {}

inline void dummy_particle_set::update_age( float timeStepSeconds ) {}

inline void dummy_particle_set::update_advection_offset( const advector_interface& /*advector*/,
                                                         const field_interface& /*velocityField*/,
                                                         float /*timeStepSeconds*/ ) {}

inline void dummy_particle_set::apply_advection_offset() {}

inline std::size_t dummy_particle_set::get_num_channels() const { return 0; }

inline dummy_particle_set::channel_id
dummy_particle_set::get_particle_channel( const frantic::tstring& /*channelName*/,
                                          std::pair<frantic::channels::data_type_t, std::size_t>* /*pOutType*/ ) {
    return 0;
}

inline dummy_particle_set::channel_id
dummy_particle_set::get_particle_channel( std::size_t /*channelIndex*/, frantic::tstring* /*pOutChannelName*/,
                                          std::pair<frantic::channels::data_type_t, std::size_t>* /*pOutType*/ ) {
    return 0;
}

inline void* dummy_particle_set::get_particle_data_ptr( index_type /*i*/, channel_id /*channelID*/ ) { return NULL; }

inline particle_set_interface_ptr dummy_particle_set::clone() const { return boost::make_shared<dummy_particle_set>(); }

inline std::size_t dummy_particle_set::get_memory_usage() const { return 1024; }

inline frantic::channels::channel_map dummy_particle_set::get_extra_channels() const {
    frantic::channels::channel_map result;
    result.end_channel_definition();
    return result;
}