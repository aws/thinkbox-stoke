// Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <stoke/field_interface.hpp>

namespace stoke {

class dummy_field : public field_interface {
  public:
    dummy_field();

    virtual ~dummy_field();

    virtual float get_velocity_scale();

    virtual void set_velocity_scale( float newScale );

    virtual void update( const time_interface& updateTime );

    // virtual void reset_simulation();

    // virtual void advance_simulation();

    virtual vec3 evaluate_velocity( const vec3& p ) const;
};

inline dummy_field::dummy_field() {}

inline dummy_field::~dummy_field() {}

inline float dummy_field::get_velocity_scale() { return 1.f; }

inline void dummy_field::set_velocity_scale( float ) {}

inline void dummy_field::update( const time_interface& ) {}

inline vec3 dummy_field::evaluate_velocity( const vec3& ) const { return vec3( 0, 0, 0 ); }

} // namespace stoke
