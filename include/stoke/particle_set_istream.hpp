// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/particle_set_interface.hpp>

#include <frantic/particles/streams/particle_istream.hpp>

#include <boost/scoped_array.hpp>
#include <boost/tuple/tuple.hpp>

#include <vector>

namespace stoke {

class particle_set_istream : public frantic::particles::streams::particle_istream {
  public:
    explicit particle_set_istream( particle_set_interface_ptr pParticleSet );

    virtual ~particle_set_istream();

    virtual void close();

    virtual frantic::tstring name() const;

    virtual std::size_t particle_size() const;

    virtual boost::int64_t particle_count() const;
    virtual boost::int64_t particle_index() const;
    virtual boost::int64_t particle_count_left() const;
    virtual boost::int64_t particle_progress_count() const;
    virtual boost::int64_t particle_progress_index() const;

    virtual void set_channel_map( const frantic::channels::channel_map& particleChannelMap );

    virtual const frantic::channels::channel_map& get_channel_map() const;
    virtual const frantic::channels::channel_map& get_native_channel_map() const;

    virtual void set_default_particle( char* rawParticleBuffer );

    virtual bool get_particle( char* rawParticleBuffer );
    virtual bool get_particles( char* rawParticleBuffer, std::size_t& numParticles );

  private:
    particle_set_interface::index_type m_particleIndex, m_particleCount;

    frantic::channels::channel_map m_outMap, m_nativeMap;
    frantic::channels::channel_accessor<vec3> m_posAccessor;
    frantic::channels::channel_cvt_accessor<vec3> m_velAccessor;
    frantic::channels::channel_cvt_accessor<float> m_ageAccessor;
    frantic::channels::channel_cvt_accessor<float> m_lifespanAccessor;
    frantic::channels::channel_cvt_accessor<boost::int64_t> m_idAccessor;
    frantic::channels::channel_cvt_accessor<vec3> m_advectionOffsetAccessor;
    frantic::channels::channel_cvt_accessor<vec3> m_fieldVelocityAccessor;

    // If the particle_set does not provide a channel named "NormalizedAge" we can derive it from Age and LifeSpan.
    bool m_deriveNormalizedAge;
    frantic::channels::channel_cvt_accessor<float> m_normalizedAgeAccessor;

    typedef boost::tuple<particle_set_interface::channel_id, frantic::channels::channel_general_accessor,
                         frantic::channels::channel_type_convertor_function_t>
        extra_channel_type;

    std::vector<extra_channel_type> m_extraChannels;

    boost::scoped_array<char> m_defaultParticle;

    particle_set_interface_ptr m_pParticleSet;
};

} // namespace stoke
