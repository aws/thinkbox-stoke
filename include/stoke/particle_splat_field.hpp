// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/field_interface.hpp>

#include <ember/staggered_grid.hpp>

#include <frantic/particles/streams/particle_istream.hpp>

namespace stoke {

class particle_splat_field : public field_interface {
  public:
    particle_splat_field( float spacing, const int ( *bounds )[6], int boundsPadding, bool removeDivergence );

    virtual ~particle_splat_field();

    virtual float get_velocity_scale();

    virtual void set_velocity_scale( float newScale );

    // Subclasses should call this->set_particles()
    virtual void update( const time_interface& updateTime ) = 0;

    // virtual void reset_simulation();

    // virtual void advance_simulation();

    virtual vec3 evaluate_velocity( const vec3& p ) const;

  protected:
    void set_particles( frantic::particles::particle_istream_ptr );

  private:
    float m_velocityScale;

    float m_spacing;

    int m_boundsPadding;

    bool m_useAutoBounds;

    bool m_removeDivergence;

    ember::staggered_grid m_gridStorage;
};

} // namespace stoke
