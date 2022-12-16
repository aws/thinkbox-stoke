// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/field_interface.hpp>

namespace stoke {

class additive_velocity_field : public field_interface {
  public:
    template <class ForwardIterator>
    additive_velocity_field( ForwardIterator begin, ForwardIterator end );

    virtual ~additive_velocity_field();

    virtual float get_velocity_scale();

    virtual void set_velocity_scale( float newScale );

    virtual void update( const time_interface& updateTime );

    // virtual void reset_simulation();

    // virtual void advance_simulation();

    virtual vec3 evaluate_velocity( const vec3& p ) const;

  private:
    std::vector<field_interface_ptr> m_implFields;

    float m_velocityScale;
};

template <class ForwardIterator>
additive_velocity_field::additive_velocity_field( ForwardIterator begin, ForwardIterator end )
    : m_implFields( begin, end )
    , m_velocityScale( 1.f ) {}

} // namespace stoke
